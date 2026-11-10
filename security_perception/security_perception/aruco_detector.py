"""ArUco detection helpers.

Works with both OpenCV APIs:
  * 4.7+        cv2.aruco.ArucoDetector
  * 4.6 (Ubuntu 24.04 python3-opencv)   cv2.aruco.detectMarkers
"""
import math

import cv2
import numpy as np


class ArucoDetector:

    def __init__(self, dictionary_name='DICT_4X4_50'):
        aruco = cv2.aruco
        dict_id = getattr(aruco, dictionary_name)
        if hasattr(aruco, 'getPredefinedDictionary'):
            self.dictionary = aruco.getPredefinedDictionary(dict_id)
        else:
            self.dictionary = aruco.Dictionary_get(dict_id)

        if hasattr(aruco, 'ArucoDetector'):
            params = aruco.DetectorParameters()
            self._detector = aruco.ArucoDetector(self.dictionary, params)
            self._params = None
        else:
            self._detector = None
            self._params = (aruco.DetectorParameters_create()
                            if hasattr(aruco, 'DetectorParameters_create')
                            else aruco.DetectorParameters())

    def detect(self, frame_bgr):
        """Return a list of dicts: marker_id, center_x, center_y, width_px, corners."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                      parameters=self._params)
        if ids is None:
            return []
        results = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners.reshape(4, 2)
            cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
            # average of the top and bottom edge lengths, in pixels
            width = (math.dist(pts[0], pts[1]) + math.dist(pts[3], pts[2])) / 2.0
            results.append({'marker_id': int(marker_id), 'center_x': cx, 'center_y': cy,
                            'width_px': float(width), 'corners': pts})
        return results


def focal_length_px(image_width, horizontal_fov_rad):
    return (image_width / 2.0) / math.tan(horizontal_fov_rad / 2.0)


def bearing_from_pixel(center_x, image_width, horizontal_fov_rad):
    """Angle to the marker from the camera axis, left positive (ROS convention)."""
    fx = focal_length_px(image_width, horizontal_fov_rad)
    return -math.atan2(center_x - image_width / 2.0, fx)


def distance_from_marker_size(width_px, marker_size_m, image_width, horizontal_fov_rad):
    """Pinhole estimate: distance = real size * focal length / pixel size."""
    if width_px <= 1:
        return None
    return marker_size_m * focal_length_px(image_width, horizontal_fov_rad) / width_px


def lidar_range_at(scan, angle, window_rad, max_range):
    """Smallest valid LiDAR range within +-window around angle (laser frame)."""
    if scan is None or scan.angle_increment == 0:
        return None
    best = None
    a = angle - window_rad
    while a <= angle + window_rad:
        wrapped = math.atan2(math.sin(a), math.cos(a))
        i = int(round((wrapped - scan.angle_min) / scan.angle_increment))
        if 0 <= i < len(scan.ranges):
            r = scan.ranges[i]
            if math.isfinite(r) and scan.range_min < r <= min(scan.range_max, max_range):
                best = r if best is None else min(best, r)
        a += abs(scan.angle_increment)
    return best


def marker_pose_in_base(corners, marker_size, image_width, image_height,
                        horizontal_fov_rad, camera_offset_x=0.0):
    """Marker pose seen from the robot, from its 4 corners (solvePnP).

    Uses a pinhole camera built from the field of view (no calibration file).
    Returns (x, y, normal_yaw) in the robot base frame:
        x, y        marker centre, x forward and y left of the base origin
        normal_yaw  direction the marker face points to (pi when the robot
                    looks straight at it), only the horizontal part is used
    or None when solvePnP fails.
    """
    fx = focal_length_px(image_width, horizontal_fov_rad)
    camera = np.array([[fx, 0.0, image_width / 2.0],
                       [0.0, fx, image_height / 2.0],
                       [0.0, 0.0, 1.0]])
    s = marker_size / 2.0
    object_pts = np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]])
    image_pts = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    # A small, almost flat marker has two poses that fit the image (one mirrored).
    # Take every candidate and keep the best one whose face points at the camera.
    candidates = []
    if hasattr(cv2, 'solvePnPGeneric') and hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE'):
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(object_pts, image_pts, camera, None,
                                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
        errs = np.asarray(errs).flatten() if errs is not None else [0.0] * n
        candidates += [(float(e), r, t) for r, t, e in zip(rvecs, tvecs, errs)]
    ok, rvec, tvec = cv2.solvePnP(object_pts, image_pts, camera, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if ok:
        proj, _ = cv2.projectPoints(object_pts, rvec, tvec, camera, None)
        candidates.append((float(np.abs(proj.reshape(4, 2) - image_pts).mean()), rvec, tvec))
    best = None
    for err, r, t in sorted(candidates, key=lambda c: c[0]):
        rm, _ = cv2.Rodrigues(r)
        if rm[2, 2] < 0 and float(np.asarray(t).flatten()[2]) > 0:   # facing the camera
            best = (rm, t)
            break
    if best is None:
        return None
    rot, tvec = best
    tx, _, tz = tvec.flatten()
    # camera frame: x right, y down, z forward  ->  base frame: x forward, y left
    x = camera_offset_x + float(tz)
    y = -float(tx)
    normal_cam = rot[:, 2]                     # marker z axis points out of its face
    normal_yaw = math.atan2(-normal_cam[0], normal_cam[2])
    return x, y, normal_yaw

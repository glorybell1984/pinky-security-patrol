"""ArUco detection helpers.

Works with both OpenCV APIs:
  * 4.7+        cv2.aruco.ArucoDetector
  * 4.6 (Ubuntu 24.04 python3-opencv)   cv2.aruco.detectMarkers
"""
import math

import cv2


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

"""ArUco detection helpers (OpenCV 4.6 API)."""
import cv2

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMETERS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(DICTIONARY, PARAMETERS)


def detect(frame_bgr):
    """Return a list of (marker_id, center_x_px) for markers in the frame."""
    corners, ids, _ = DETECTOR.detectMarkers(frame_bgr)
    if ids is None:
        return []
    results = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        center_x = float(marker_corners[0][:, 0].mean())
        results.append((int(marker_id), center_x))
    return results


def bearing_from_pixel(center_x, image_width, horizontal_fov_rad):
    """Angle to the marker in the camera frame (left positive)."""
    offset = (center_x - image_width / 2.0) / (image_width / 2.0)
    return -offset * horizontal_fov_rad / 2.0

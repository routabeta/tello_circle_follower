import cv2
import numpy as np

def find_circles(frame: np.ndarray, scale: float, maxRadius: int, minRadius: int):
    """Object recognition to find located circles
    
    Returns a 2D n x 3 list containing (x, y, r) for all circles found, or None"""

    shrunk = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
    gray = cv2.cvtColor(shrunk, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=130,
        param2=60,
        minRadius=minRadius,
        maxRadius=maxRadius
    )

    if circles is not None:
        return circles[0]
    return None

def tracker_init(frame: np.ndarray, x: int, y: int, r: int):
    """Initialize the tracker with the most recent known location of the circle

    Returns the tracker object"""

    bbox = [x - r, y - r, 2 * r, 2 * r]
    tracker = cv2.TrackerKCF_create()
    tracker.init(frame, bbox)
    return tracker
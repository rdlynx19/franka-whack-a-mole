import pytest
import numpy as np
import cv2
from unittest.mock import MagicMock
from whack_a_mole.constants import COLORS
from whack_a_mole.mole_detection import OpenCVClient

# --------------- Begin Citation [1] -------------------#
@pytest.fixture
def opencv_client():
    return OpenCVClient()

def test_initialization(opencv_client):
    assert opencv_client.clipping_distance == 1400
    assert opencv_client.crop == [(1, 1), (1, 1)]
    assert opencv_client.freq == 30.0
    assert opencv_client._camera_intrinsics.size == 0
    assert opencv_client._color_image.size == 0
    assert opencv_client._depth_image.size == 0

def test_camera_intrinsics(opencv_client):
    intrinsics = np.array([600, 0, 320, 0, 600, 240, 0, 0, 1])
    opencv_client.camera_intrinsics = intrinsics
    assert np.array_equal(opencv_client.camera_intrinsics, intrinsics)

def test_color_image(opencv_client):
    color_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    opencv_client.color_image = color_image
    assert np.array_equal(opencv_client.color_image, color_image)

def test_depth_image(opencv_client):
    depth_image = np.random.randint(0, 65535, (480, 640), dtype=np.uint16)
    opencv_client.depth_image = depth_image
    assert np.array_equal(opencv_client.depth_image, depth_image)

def test_find_centroid_cropped(opencv_client):
    # Create a binary image with a single contour in the center
    masked_image = np.zeros((100, 100), dtype=np.uint8)
    cv2.rectangle(masked_image, (40, 40), (60, 60), 255, -1)  # White rectangle
    crop_indices = (30, 30, 70, 70)

    x, y = opencv_client.find_centroid_cropped(masked_image, crop_indices)
    assert (x, y) == (50, 50)  # Centroid should be in the center

def test_find_centroid_cropped_no_contours(opencv_client):
    masked_image = np.zeros((100, 100), dtype=np.uint8)  # Empty image
    crop_indices = (30, 30, 70, 70)

    x, y = opencv_client.find_centroid_cropped(masked_image, crop_indices)
    assert (x, y) == (0, 0)  # No contours, centroid should be (0, 0)

def test_get_3d_coordinates_at_pixel(opencv_client):
    intrinsics = np.array([600, 0, 320, 0, 600, 240, 0, 0, 1])
    opencv_client.camera_intrinsics = intrinsics

    depth_image = np.zeros((480, 640), dtype=np.uint16)
    depth_image[240, 320] = 2000  # Depth value at pixel (320, 240) is 2000 mm
    opencv_client.depth_image = depth_image

    X, Y, Z = opencv_client.get_3d_coordinates_at_pixel(320, 240)
    assert Z == 2.0  # Depth in meters
    assert np.isclose(X, 0.0)  # X should be close to 0
    assert np.isclose(Y, 0.0)  # Y should be close to 0

def test_get_3d_coordinates_at_pixel_no_depth(opencv_client):
    intrinsics = np.array([600, 0, 320, 0, 600, 240, 0, 0, 1])
    opencv_client.camera_intrinsics = intrinsics

    depth_image = np.zeros((480, 640), dtype=np.uint16)  # No depth data
    opencv_client.depth_image = depth_image

    X, Y, Z = opencv_client.get_3d_coordinates_at_pixel(320, 240)
    assert (X, Y, Z) == (-1, -1, -1)  # Invalid depth data
# --------------- End Citation [1] -------------------#
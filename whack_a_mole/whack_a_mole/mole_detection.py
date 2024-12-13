"""
OpenCV Client for Mole Detection
================================

This module implements the `OpenCVClient` class, which handles image processing for detecting moles in the Whack-A-Mole game using OpenCV.

Classes
-------
OpenCVClient
    A class to handle OpenCV operations for mole detection, such as finding color centroids, detecting illumination, and mapping 2D pixel coordinates to 3D camera coordinates.
"""

import cv2
import numpy as np
from whack_a_mole.constants import COLORS


class OpenCVClient:
    """
    A class to handle OpenCV operations for mole detection.

    **Attributes**:
    - `clipping_distance` (`int`): The clipping distance for the depth image in millimeters.
    - `crop` (`list`): The crop region for the image defined as two points [(x1, y1), (x2, y2)].
    - `freq` (`float`): The frequency of the running average calculation.
    - `_running_avg` (`dict`): A dictionary to store the running averages of centroids for colors.
    - `_camera_intrinsics` (`np.array`): The intrinsic parameters of the camera.
    - `_color_image` (`np.array`): The captured color image.
    - `_depth_image` (`np.array`): The captured depth image.

    **Methods**:
    - `camera_intrinsics`: Getter and setter for the camera intrinsics.
    - `color_image`: Getter and setter for the color image.
    - `depth_image`: Getter and setter for the depth image.
    - `running_avg`: Getter and setter for the running average.
    - `detect_illumination(color)`: Detects the illumination for the given color.
    - `find_centroid_cropped(masked_image, crop_indices)`: Finds the centroid of the largest contour within a crop region.
    - `get_3d_coordinates_at_pixel(x, y, frame_name)`: Converts 2D pixel coordinates to 3D camera coordinates in meters.
    """

    def __init__(self, freq=30.0, clipping_distance=1400, crop=[(1, 1), (1, 1)]):
        """
        Initializes the OpenCVClient with default or user-provided parameters.

        :arg freq: The frequency for running average calculations (default: 30.0 Hz).
        :type freq: float
        :arg clipping_distance: The clipping distance for the depth image in millimeters (default: 1400).
        :type clipping_distance: int
        :arg crop: The crop region for the image (default: [(1, 1), (1, 1)]).
        :type crop: list
        """
        self.clipping_distance = clipping_distance
        self.crop = crop
        self.freq = freq
        self._running_avg = {
            "GREEN": [],
            "YELLOW": [],
            "BLUE": [],
            "RED": [],
        }
        self.running_avg = (
            np.zeros((len(COLORS.keys()), int(self.freq * 10), 2))
            + (np.array(self.crop[0]) + np.array(self.crop[1])) / 2
        )

        self._camera_intrinsics = np.array([])
        self._color_image = np.array([])
        self._depth_image = np.array([])

    @property
    def camera_intrinsics(self):
        """
        Getter for the camera intrinsics.

        :return: The camera intrinsics as a numpy array.
        :rtype: np.array
        """
        return self._camera_intrinsics

    @camera_intrinsics.setter
    def camera_intrinsics(self, value):
        """
        Setter for the camera intrinsics.

        :arg value: The intrinsic parameters of the camera.
        :type value: np.array
        """
        self._camera_intrinsics = value

    @property
    def color_image(self):
        """
        Getter for the color image.

        :return: The color image.
        :rtype: np.array
        """
        return self._color_image

    @color_image.setter
    def color_image(self, value):
        """
        Setter for the color image.

        :arg value: The captured color image.
        :type value: np.array
        """
        self._color_image = value

    @property
    def depth_image(self):
        """
        Getter for the depth image.

        :return: The depth image.
        :rtype: np.array
        """
        return self._depth_image

    @depth_image.setter
    def depth_image(self, value):
        """
        Setter for the depth image.

        :arg value: The captured depth image.
        :type value: np.array
        """
        self._depth_image = value

    @property
    def running_avg(self):
        """
        Getter for the running average.

        :return: The running average of centroids for each color.
        :rtype: dict
        """
        return self._running_avg

    @running_avg.setter
    def running_avg(self, value):
        """
        Setter for the running average.

        :arg value: The running average values for centroids.
        :type value: dict
        """
        self._running_avg = value

    def detect_illumination(self, color: str):
        """
        Detects the illumination of the specified color and updates the running average.

        :arg color: The color of the object (e.g., "GREEN", "YELLOW").
        :type color: str
        """
        color_index = COLORS[color]
        median_centroid = np.median(self.running_avg[color_index], axis=0)
        median_centroid = np.array(median_centroid, dtype=int)

    def find_centroid_cropped(self, masked_image, crop_indices):
        """
        Finds the centroid of the largest contour within the specified crop region.

        :arg masked_image: The binary masked image for processing.
        :type masked_image: np.array
        :arg crop_indices: A tuple (x1, y1, x2, y2) defining the crop region.
        :type crop_indices: tuple

        :return: The (x, y) coordinates of the centroid in the original image.
        :rtype: tuple(int, int)
        """
        x1, y1, x2, y2 = crop_indices
        # Crop the masked image based on the indices
        cropped_image = masked_image[y1:y2, x1:x2]

        # Find the largest contour in the cropped image
        image = np.array(cropped_image, dtype=np.uint8)
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = -1
        largest_index = -1
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                largest_index = i

        if (largest_area == 0) or (largest_area == -1):
            return 0, 0

        moments = cv2.moments(contours[largest_index])
        if moments["m00"] == 0:
            return 0, 0

        x_c = int(moments["m10"] / moments["m00"])
        y_c = int(moments["m01"] / moments["m00"])
        return x1 + x_c, y1 + y_c

    def get_3d_coordinates_at_pixel(self, x, y, frame_name=""):
        """
        Converts the 2D pixel coordinates (x, y) to 3D camera coordinates in meters.

        :arg x: The x-coordinate in pixels.
        :type x: int
        :arg y: The y-coordinate in pixels.
        :type y: int
        :arg frame_name: The name of the frame to associate with the coordinates (optional).
        :type frame_name: str

        :return: The 3D coordinates (X, Y, Z) in meters or (-1, -1, -1) if depth is not available.
        :rtype: tuple(float, float, float)
        """
        # Ensure depth image is available
        if self.camera_intrinsics.shape[0] == 0:
            return -1, -1, -1

        # Get depth in meters
        depth_in_meters = self.depth_image[y, x] / 1000
        if depth_in_meters == 0:
            return -1, -1, -1

        # Get camera intrinsics
        f_x, f_y = self.camera_intrinsics[0], self.camera_intrinsics[4]
        c_x, c_y = self.camera_intrinsics[2], self.camera_intrinsics[5]

        # Calculate 3D coordinates
        X = (x - c_x) * depth_in_meters / f_x
        Y = (y - c_y) * depth_in_meters / f_y
        Z = depth_in_meters
        return X, Y, Z

"""Python library for CV operations for the Whack-a-Mole game."""

import cv2

import numpy as np

from whack_a_mole.constants import COLORS


class OpenCVClient:
    """
    A class to handle the OpenCV operations for the mole detection.

    Attributes:
    - clipping_distance (int): The clipping distance for the depth image.
    - crop (list): The crop region for the image.
    - _running_avg (dict): A dictionary to store the running average of the
        centroids.
    - _camera_intrinsics (np.array): The camera intrinsics.
    - _color_image (np.array): The color image.
    - _depth_image (np.array): The depth image.

    Methods:
    - find_centroid_cropped(masked_image, crop_indices):
        Find the centroid of the largest contour within a specified crop
        region.
    - find_color_centroid(lower_HSV, higher_HSV):
        Returns the pixel indecies of the centroid of a color defined in
        lower_HSV , higher_HSV range
    - get_3d_coordinates_at_pixel(x, y, frame_name):
        Convert the pixel coordinates (x, y) to 3D camera coordinates
        in meters.

    """

    def __init__(
            self, freq=30.0,
            clipping_distance=1400,
            crop=[(1, 1), (1, 1)]):
        """Initialize the OpenCV client."""
        self.clipping_distance = clipping_distance
        self.crop = crop
        self.freq = freq
        self._running_avg = {
            'GREEN': [],
            'YELLOW': [],
            'BLUE': [],
            'RED': [],
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
        """Getter for the camera intrinsics."""
        return self._camera_intrinsics

    @camera_intrinsics.setter
    def camera_intrinsics(self, value):
        """Setter for the camera intrinsics."""
        self._camera_intrinsics = value

    @property
    def color_image(self):
        """Getter for the color image."""
        return self._color_image

    @color_image.setter
    def color_image(self, value):
        """Setter for the color image."""
        self._color_image = value

    @property
    def depth_image(self):
        """Getter for the depth image."""
        return self._depth_image

    @depth_image.setter
    def depth_image(self, value):
        """Setter for the depth image."""
        self._depth_image = value

    @property
    def running_avg(self):
        """Getter for the running average."""
        return self._running_avg

    @running_avg.setter
    def running_avg(self, value):
        """Setter for the running average."""
        self._running_avg = value

    def detect_illumination(self, color: str):
        """
        Detect the illumination of the color and broadcasts.

        Args:
            color (str): The color of the object

        """
        color_index = COLORS[color]
        median_centroid = np.median(self.running_avg[color_index], axis=0)
        median_centroid = np.array(median_centroid, dtype=int)

    def find_centroid_cropped(self, masked_image, crop_indices):
        """
        Find the centroid of the largest contour in specified crop region.

        Args:
        - masked_image (np.array): The masked image to process.
        - crop_indices (tuple): A tuple (x1, y1, x2, y2) specifying the crop
            region.
        Returns:
        - (int, int): The (x, y) coordinates of the centroid in the original
            image.
        """
        # Unpack the crop region indices
        x1, y1, x2, y2 = crop_indices

        # Crop the masked image based on the provided indices
        cropped_image = masked_image[y1:y2, x1:x2]

        # Convert cropped image to numpy array (if necessary)
        image = np.array(cropped_image, dtype=np.uint8)

        # Find all contours in the cropped region
        contours, _ = cv2.findContours(
            image,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour area
        largest_area = -1
        largest_index = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            # Check if the current contour has the largest area so far
            if area > largest_area:
                largest_area = area
                largest_index = i

        # If no valid contour was found (largest_area remains 0)
        if (largest_area == 0) or (largest_area == -1):
            return 0, 0

        # Get the centroid in the cropped region
        moments = cv2.moments(contours[largest_index])
        if moments['m00'] == 0:  # To avoid division by zero
            return 0, 0

        # Calculate the centroid in the cropped image
        x_c = int(moments['m10'] / moments['m00'])
        y_c = int(moments['m01'] / moments['m00'])

        # Convert the cropped image centroid back to original image coordinates
        original_x_c = x1 + x_c
        original_y_c = y1 + y_c
        return original_x_c, original_y_c

    def get_3d_coordinates_at_pixel(self, x, y, frame_name=''):
        """Convert the pixel coordinates (x, y) to 3D coordinates in meters."""
        if self.camera_intrinsics.shape[0] == 0:
            return -1, -1, -1

        # Get the depth value at the specified pixel (x, y)
        depth_in_meters = self.depth_image[y, x] / 1000
        if depth_in_meters == 0:  # No depth data at this pixel
            return -1, -1, -1

        # Extract intrinsic parameters (in a 3x3 matrix form)
        f_x, f_y = (
            self.camera_intrinsics[0],
            self.camera_intrinsics[4],
        )  # Focal lengths (in pixels)

        c_x, c_y = (
            self.camera_intrinsics[2],
            self.camera_intrinsics[5],
        )  # Optical center (in pixels)

        # Convert to 3D coordinates (X, Y, Z) in camera frame
        X = (x - c_x) * depth_in_meters / f_x
        Y = (y - c_y) * depth_in_meters / f_y
        Z = depth_in_meters  # Depth in meters
        return X, Y, Z

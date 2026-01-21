import threading
from sys import platform

import cv2
import numpy as np
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2


class DisplayImagesAsync(object):
    """Display the images Spot sees from all five cameras."""

    def __init__(self, april_tag_tracker):
        self._april_tag_tracker = april_tag_tracker
        self._thread = None
        self._started = False
        self._sources = []

    def get_image(self):
        """Retrieve current images (with bounding boxes) from the fiducial detector."""
        images = self._april_tag_tracker.image
        image_by_source = []
        for s_name in self._sources:
            if s_name in images:
                image_by_source.append(images[s_name])
            else:
                image_by_source.append(np.array([]))
        return image_by_source

    def start(self):
        """Initialize the thread to display the images."""
        if self._started:
            return None
        self._sources = self._april_tag_tracker.image_sources_list
        self._started = True
        self._thread = threading.Thread(target=self.update)
        self._thread.start()
        return self

    def update(self):
        """Update the images being displayed to match that seen by the robot."""
        while self._started:
            images = self.get_image()
            for i, image in enumerate(images):
                if image.size != 0:
                    original_height, original_width = image.shape[:2]
                    resized_image = cv2.resize(
                        image,
                        (int(original_width * 0.5), int(original_height * 0.5)),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    cv2.imshow(self._sources[i], resized_image)
                    cv2.moveWindow(
                        self._sources[i],
                        max(
                            int(i * original_width * 0.5),
                            int(i * original_height * 0.5),
                        ),
                        0,
                    )
                    cv2.waitKey(1)

    def stop(self):
        """Stop the thread and the image displays."""
        self._started = False
        cv2.destroyAllWindows()

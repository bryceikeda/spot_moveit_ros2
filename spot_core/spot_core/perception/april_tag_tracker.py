"""Detect fiducial tags."""

import logging
from sys import platform

import cv2
import numpy as np
from PIL import Image
from bosdyn.api import image_pb2
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.image import build_image_request
from spot_core.perception.display_images_async import DisplayImagesAsync
from apriltag import apriltag
from bosdyn.client.image import ImageClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.client.math_helpers import Quat
from scipy.spatial.transform import Rotation as R
from bosdyn.client.math_helpers import Quat
from spot_core.perception.display_images_async import DisplayImagesAsync

# pylint: disable=no-member
LOGGER = logging.getLogger()

# Use this length to make sure we're commanding the head of the robot
# to a position instead of the center.
BODY_LENGTH = 1.1


class AprilTagTracker(object):
    """Detect and follow a fiducial with Spot."""

    def __init__(self, image_client):
        # Robot instance variable.
        self._image_client = image_client

        # Camera intrinsics for the current camera source being analyzed.
        self._intrinsics = None

        # Transform from the robot's camera frame to the baselink frame.
        # It is a math_helpers.SE3Pose.
        self._camera_tform_body = None

        # Transform from the robot's baselink to the world frame.
        # It is a math_helpers.SE3Pose.
        self._body_tform_world = None

        # Dictionary mapping camera source to it's latest image taken.
        self._image = dict()

        # List of all possible camera sources.
        self._source_names = [
            src.name
            for src in self._image_client.list_image_sources()
            if (
                src.image_type == image_pb2.ImageSource.IMAGE_TYPE_VISUAL
                and "depth" not in src.name
            )
        ]
        print(self._source_names)

        # Dictionary mapping camera source to previously computed extrinsics.
        self._camera_to_extrinsics_guess = self.populate_source_dict()

        # Start live image viewer
        self.image_viewer = DisplayImagesAsync(self)

    @property
    def image(self):
        """Return the current image associated with each source name."""
        return self._image

    @property
    def image_sources_list(self):
        """Return the list of camera sources."""
        return self._source_names

    def start_image_viewer(self):
        self.image_viewer.start()

    def stop_image_viewer(self):
        self.image_viewer.stop()

    def populate_source_dict(self):
        """Fills dictionary of the most recently computed camera extrinsics with the camera source.
        The initial boolean indicates if the extrinsics guess should be used."""
        camera_to_extrinsics_guess = dict()
        for src in self._source_names:
            # Dictionary values: use_extrinsics_guess bool, (rotation vector, translation vector) tuple.
            camera_to_extrinsics_guess[src] = (False, (None, None))
        return camera_to_extrinsics_guess

    def detect_fiducials(self):
        """Detect all fiducials from all camera sources and return their world poses."""
        detections = []

        for source_name in self._source_names:
            # Get the latest image from the current source camera
            img_req = build_image_request(
                source_name,
                quality_percent=100,
                image_format=image_pb2.Image.FORMAT_RAW,
            )
            image_response = self._image_client.get_image([img_req])[0]
            self._camera_tform_body = get_a_tform_b(
                image_response.shot.transforms_snapshot,
                image_response.shot.frame_name_image_sensor,
                BODY_FRAME_NAME,
            )
            self._body_tform_world = get_a_tform_b(
                image_response.shot.transforms_snapshot,
                BODY_FRAME_NAME,
                VISION_FRAME_NAME,
            )

            # Camera intrinsics for the given source camera
            self._intrinsics = image_response.source.pinhole.intrinsics
            width = image_response.shot.image.cols
            height = image_response.shot.image.rows

            # Detect fiducials in the image
            detected_fiducials = self.detect_fiducial_in_image(
                image_response.shot.image, (width, height), source_name
            )

            if not detected_fiducials:
                continue

            # Get tvec/rvec for all bounding boxes
            for fiducial in detected_fiducials:
                (tvec, rvec) = self.pixel_coords_to_camera_coords(
                    fiducial["bbox"], self._intrinsics, source_name
                )
                pose = self.compute_fiducial_in_world_frame(tvec, rvec)
                detections.append(
                    {"id": fiducial["id"], "frame_id": "vision", "pose": pose}
                )

        return detections

    def detect_fiducial_in_image(self, image, dim, source_name):
        """Detect all fiducials in a single image and return their bounding boxes and IDs."""
        # Convert image bytes to grayscale
        image_grey = np.array(
            Image.frombytes(
                "P", (int(dim[0]), int(dim[1])), data=image.data, decoder_name="raw"
            )
        )

        # Initialize AprilTag detector
        detector = apriltag(family="tag36h11")
        detections = detector.detect(image_grey)

        detected_fiducials = []
        for i in range(len(detections)):
            bbox = detections[i]["lb-rb-rt-lt"]
            tag_id = detections[i]["id"]

            # Draw bounding box
            cv2.polylines(image_grey, [np.int32(bbox)], True, (0, 0, 0), 2)

            detected_fiducials.append({"bbox": bbox, "id": tag_id})

        # Save the image with bounding boxes for display
        self._image[source_name] = self.rotate_image(image_grey, source_name)

        return detected_fiducials

    def bbox_to_image_object_pts(self, bbox):
        """Determine the object points and image points for the bounding box.
        The origin in object coordinates = top left corner of the fiducial.
        Order both points sets following: (TL,TR, BL, BR)"""
        fiducial_height_and_width = 146  # mm
        obj_pts = np.array(
            [
                [0, 0],
                [fiducial_height_and_width, 0],
                [0, fiducial_height_and_width],
                [fiducial_height_and_width, fiducial_height_and_width],
            ],
            dtype=np.float32,
        )
        # insert a 0 as the third coordinate (xyz)
        obj_points = np.insert(obj_pts, 2, 0, axis=1)

        # ['lb-rb-rt-lt']
        img_pts = np.array(
            [
                [bbox[3][0], bbox[3][1]],
                [bbox[2][0], bbox[2][1]],
                [bbox[0][0], bbox[0][1]],
                [bbox[1][0], bbox[1][1]],
            ],
            dtype=np.float32,
        )
        return obj_points, img_pts

    def pixel_coords_to_camera_coords(self, bbox, intrinsics, source_name):
        """Compute 3D camera coordinates for multiple fiducials from their bounding boxes."""
        camera = self.make_camera_matrix(intrinsics)

        obj_points, img_points = self.bbox_to_image_object_pts(bbox)

        if self._camera_to_extrinsics_guess[source_name][0]:
            # Use previous extrinsics as initial guess
            old_rvec, old_tvec = self._camera_to_extrinsics_guess[source_name][1]
            _, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera,
                np.zeros((5, 1)),
                old_rvec,
                old_tvec,
                True,
                cv2.SOLVEPNP_ITERATIVE,
            )
        else:
            # No previous guess
            _, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, camera, np.zeros((5, 1))
            )

        # Update extrinsics guess for next frame
        self._camera_to_extrinsics_guess[source_name] = (True, (rvec, tvec))

        return (tvec, rvec)

    def compute_fiducial_in_world_frame(self, tvec, rvec):
        """Transform the fiducial position and orientation from camera coordinates to world coordinates."""
        # Apply correction to translation vector
        fiducial_rt_camera_frame = np.array(
            [
                float(tvec[0][0]) / 1000.0,
                float(tvec[1][0]) / 1000.0,
                float(tvec[2][0]) / 1000.0,
            ]
        )

        # Convert rotation vector to rotation matrix
        rvec_array = np.array([float(rvec[0][0]), float(rvec[1][0]), float(rvec[2][0])])
        rot_mat_camera, _ = cv2.Rodrigues(rvec_array)

        # Create SE3Pose for fiducial in camera frame
        from bosdyn.client.math_helpers import SE3Pose, Quat

        fiducial_rot_camera = R.from_matrix(rot_mat_camera)
        fiducial_quat_camera = fiducial_rot_camera.as_quat()  # [x, y, z, w]

        fiducial_tform_camera = SE3Pose(
            x=fiducial_rt_camera_frame[0],
            y=fiducial_rt_camera_frame[1],
            z=fiducial_rt_camera_frame[2],
            rot=Quat(
                w=fiducial_quat_camera[3],
                x=fiducial_quat_camera[0],
                y=fiducial_quat_camera[1],
                z=fiducial_quat_camera[2],
            ),
        )

        # Transform through the chain: camera -> body -> world
        body_tform_fiducial = self._camera_tform_body.inverse() * fiducial_tform_camera
        fiducial_tform_world = self._body_tform_world.inverse() * body_tform_fiducial

        # Extract position and orientation
        position = [
            fiducial_tform_world.x,
            fiducial_tform_world.y,
            fiducial_tform_world.z,
        ]
        quat = fiducial_tform_world.rot
        orientation = [quat.x, quat.y, quat.z, quat.w]
        return {
            "position": position,
            "orientation": orientation,
        }

    @staticmethod
    def rotate_image(image, source_name):
        """Rotate the image so that it is always displayed upright."""
        if source_name == "frontleft_fisheye_image":
            image = cv2.rotate(image, rotateCode=0)
        elif source_name == "right_fisheye_image":
            image = cv2.rotate(image, rotateCode=1)
        elif source_name == "frontright_fisheye_image":
            image = cv2.rotate(image, rotateCode=0)
        return image

    @staticmethod
    def make_camera_matrix(ints):
        """Transform the ImageResponse proto intrinsics into a camera matrix."""
        camera_matrix = np.array(
            [
                [ints.focal_length.x, ints.skew.x, ints.principal_point.x],
                [ints.skew.y, ints.focal_length.y, ints.principal_point.y],
                [0, 0, 1],
            ]
        )
        return camera_matrix

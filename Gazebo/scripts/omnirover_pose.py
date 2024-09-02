""""
Obtain the pose of the camera and omnirover

For testing the GSoC 2024 Camera Tracking project


Gazebo Topics
-------------

Static pose information.
/world/playpen/pose/info

The dynamic pose info does not provide any information about the model tree
and provides relative poses rather than world poses.
/world/playpen/dynamic_pose/info

/gui/camera/pose

Plugin: OdometryPublisher
pose: pose of model in the ENU world frame adjusted for optional offset.
odometry: pose and twist in the ENU world frame.
odometry_wkth_covariance: odometry with estimated covariance in ENU frame. 

/model/omni4rover/odometry
/model/omni4rover/odometry_with_covariance
/model/omni4rover/pose

Plugin: JointStatePublisher
joint_state: gz.msgs.Model
/world/playpen/model/omni4rover/joint_state

Usage
-----

python ./src/ardupilot_sitl_models/Gazebo/scripts/omnirover_pose.py


"""

import numpy as np
import time
import unittest

from threading import Lock
from threading import Thread

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model

from gz.transport13 import Node
from gz.transport13 import SubscribeOptions

from transforms3d import affines
from transforms3d import quaternions


mutex = Lock()


class Pose:
    def __init__(self, world, body, position, orientation):
        self.world = world
        self.body = body
        self.position = position
        self.orientation = orientation

    def __str__(self):
        s = (
            "world: "
            + str(self.world)
            + "\n"
            + "body: "
            + str(self.body)
            + "\n"
            + "position: "
            + str(self.position)
            + "\n"
            + "orientation: "
            + str(self.orientation)
        )
        return s


class PoseMonitor:
    """
    Calculate camera and target pose from pose_v messages.
    """

    def __init__(self):
        self.WORLD_NAME = "playpen"
        self.TARGET_NAME = "omni4rover"
        self.CAMERA_NAME = "mount"

        self._node = Node()

        # subscribe to pose_v messages
        self._target_do_print_msg = True
        self._target_pose_v_msg = None
        self._target_pose_v_topic = f"/model/{self.TARGET_NAME}/pose"
        self._target_pose_v_sub = self._node.subscribe(
            Pose_V, self._target_pose_v_topic, self.target_pose_v_cb
        )

        self._camera_do_print_msg = True
        self._camera_pose_v_msg = None
        self._camera_pose_v_topic = f"/model/{self.CAMERA_NAME}/pose"
        self._camera_pose_v_sub = self._node.subscribe(
            Pose_V, self._camera_pose_v_topic, self.camera_pose_v_cb
        )

        # create update thread
        self._update_thread = Thread(target=self.update)
        self._update_thread.run()

    def target_pose_v_cb(self, msg: Pose_V):
        with mutex:
            self._target_pose_v_msg = msg

        if self._target_do_print_msg:
            self._target_do_print_msg = False
            print("target pose")
            for pose in self._target_pose_v_msg.pose:
                print(pose.name)

    def camera_pose_v_cb(self, msg: Pose_V):
        with mutex:
            self._camera_pose_v_msg = msg

        if self._camera_do_print_msg:
            self._camera_do_print_msg = False
            print("camera pose")
            for pose in self._camera_pose_v_msg.pose:
                print(pose.name)

    def update(self):
        update_rate = 1.0
        update_period = 1.0 / update_rate
        while True:
            if self._camera_pose_v_msg is not None:
                world = self.WORLD_NAME
                body = "mount::gimbal::pitch_link::camera"
                self.body_to_world(self._camera_pose_v_msg, world, body)

            if self._target_pose_v_msg is not None:
                world = self.WORLD_NAME
                body = "omni4rover"
                self.body_to_world(self._target_pose_v_msg, world, body)

            time.sleep(update_period)

    def body_to_world(self, pose_v_msg, world, body):
        """
        Calculate the transform from body to world from a pose_v_msg
        """
        # create dictionaries for the poses
        pose_dict = {}
        for pose in pose_v_msg.pose:
            pose_dict[pose.name] = pose

        child = body
        print(f"{child}")
        parent = None
        X = np.identity(4)
        # print(X)
        while parent != world:
            pose = pose_dict[child]

            for data in pose.header.data:
                if data.key == "frame_id":
                    parent = data.value[0]
                    break

            # print(f"child:  {child}")
            # print(f"parent: {parent}\n")
            print(f"--> {child}")
            child = parent

            # convert pose to an affine transform
            X_p_c = np.array(self.pose_to_tf(pose))
            X = np.matmul(X_p_c, X)
            # print(X_p_c)
            # print(X)

        print(f"--> {parent}")

        t, r, z, s = affines.decompose(X)
        q = quaternions.mat2quat(r)
        # print(f"t: {t}")
        # print(f"q: {q}")

        pose = Pose(world, body, t, q)
        print(pose)
        print()

    def pose_to_tf(self, pose):
        """
        Convert a pose message into an transforms3d.affine
        """
        t = [pose.position.x, pose.position.y, pose.position.z]
        q = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        r = quaternions.quat2mat(q)
        z = [1.0, 1.0, 1.0]
        a = affines.compose(t, r, z)
        # print(f"p: {t}")
        # print(f"q: {q}")
        # print(f"r: {r}")
        # print(f"a: {a}")
        return a


def main():
    pose_monitor = PoseMonitor()
    loop_rate_hz = 50.0
    loop_duration_s = 1.0 / loop_rate_hz

    try:
        while True:
            time.sleep(loop_duration_s)

    except KeyboardInterrupt:
        print("Exiting")


if __name__ == "__main__":
    main()

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

import time
import unittest

from threading import Lock

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model

from gz.transport13 import Node
from gz.transport13 import SubscribeOptions


mutex = Lock()


class Pose:
    def __init__(self):
        self.name = ""
        self.frame_id = ""
        self.child_frame_id = ""
        self.children = {}
        self.parents = {}


class PoseMonitor:
    """
    Simple open loop controller for the OmniRover
    """

    def __init__(self):
        WORLD_NAME = "playpen"
        TARGET_NAME = "omni4rover"
        CAMERA_NAME = "mount"

        self._node = Node()

        # poses
        self._target_pose_msg = None
        self._target_pose_topic = f"/model/{TARGET_NAME}/pose"
        self._target_pose_sub = self._node.subscribe(
            Pose_V, self._target_pose_topic, self.target_pose_cb
        )
        # print(self._target_pose_sub)

        self._camera_print_msg = True
        self._camera_pose_msg = None
        self._camera_pose_topic = f"/model/{CAMERA_NAME}/pose"
        self._camera_pose_sub = self._node.subscribe(
            Pose_V, self._camera_pose_topic, self.camera_pose_cb
        )
        # print(self._camera_pose_sub)

        # joint states (model)
        # self._target_model_msg = None
        # self._target_model_topic = f"/world/{WORLD_NAME}/model/{TARGET_NAME}/joint_state"
        # self._target_model_sub = self._node.subscribe(
        #     Model, self._target_model_topic, self.target_model_cb
        # )
        # print(self._target_model_sub)

        # self._camera_model_msg = None
        # self._camera_model_topic = f"/world/{WORLD_NAME}/model/{CAMERA_NAME}/joint_state"
        # self._camera_model_sub = self._node.subscribe(
        #     Model, self._camera_model_topic, self.camera_model_cb
        # )
        # print(self._camera_model_sub)

        # self._clock_msg = None
        # self._clock_topic = "/world/playpen/clock"
        # self._clock_sub = self._node.subscribe(Clock, self._clock_topic, self.clock_cb)
        # print(self._clock_sub)

    def target_pose_cb(self, msg: Pose_V):
        with mutex:
            self._target_pose_msg = msg

        pose = self._target_pose_msg.pose[0]
        # print("target pose")
        # print(pose.position)

    def camera_pose_cb(self, msg: Pose_V):
        with mutex:
            self._camera_pose_msg = msg

        if self._camera_print_msg:
            self._camera_print_msg = False
            print("camera pose")
            for pose in self._camera_pose_msg.pose:
                print(pose.name)

    # def target_model_cb(self, msg: Model):
    #     with mutex:
    #         self._target_model_msg = msg
    #
    #     model = self._target_model_msg
    #     print("target model")
    #     print(model)

    # def camera_model_cb(self, msg: Model):
    #     with mutex:
    #         self._camera_model_msg = msg
    #
    #     model = self._camera_model_msg
    #     print("camera model")
    #     print(model)

    # def clock_cb(self, msg: Clock):
    #     with mutex:
    #         self._clock_msg = msg
    #         print(msg)


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

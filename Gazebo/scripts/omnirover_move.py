"""
Move the omnirover

For testing the GSoC 2024 Camera Tracking project
"""

import time

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node


class OmniRoverController:
    """
    Simple open loop controller for the OmniRover
    """

    def __init__(self):
        # model name
        MODEL_NAME = "omni4rover"

        self._node = Node()
        bl_cmd_vel_topic = f"/model/{MODEL_NAME}/joint/back_left_wheel_joint/cmd_vel"
        br_cmd_vel_topic = f"/model/{MODEL_NAME}/joint/back_right_wheel_joint/cmd_vel"
        fl_cmd_vel_topic = f"/model/{MODEL_NAME}/joint/front_left_wheel_joint/cmd_vel"
        fr_cmd_vel_topic = f"/model/{MODEL_NAME}/joint/front_right_wheel_joint/cmd_vel"

        self._pub_bl_vel_cmd = self._node.advertise(bl_cmd_vel_topic, Double)
        self._pub_br_vel_cmd = self._node.advertise(br_cmd_vel_topic, Double)
        self._pub_fl_vel_cmd = self._node.advertise(fl_cmd_vel_topic, Double)
        self._pub_fr_vel_cmd = self._node.advertise(fr_cmd_vel_topic, Double)

        self._bl_vel_cmd = Double()
        self._br_vel_cmd = Double()
        self._fl_vel_cmd = Double()
        self._fr_vel_cmd = Double()

    def move_x(self, speed):
        self._bl_vel_cmd.data = speed
        self._br_vel_cmd.data = speed
        self._fl_vel_cmd.data = speed
        self._fr_vel_cmd.data = speed

        self._pub_bl_vel_cmd.publish(self._bl_vel_cmd)
        self._pub_br_vel_cmd.publish(self._br_vel_cmd)
        self._pub_fl_vel_cmd.publish(self._fl_vel_cmd)
        self._pub_fr_vel_cmd.publish(self._fr_vel_cmd)

    def move_y(self, speed):
        self._bl_vel_cmd.data = speed
        self._br_vel_cmd.data = speed * -1.0
        self._fl_vel_cmd.data = speed * -1.0
        self._fr_vel_cmd.data = speed

        self._pub_bl_vel_cmd.publish(self._bl_vel_cmd)
        self._pub_br_vel_cmd.publish(self._br_vel_cmd)
        self._pub_fl_vel_cmd.publish(self._fl_vel_cmd)
        self._pub_fr_vel_cmd.publish(self._fr_vel_cmd)

    def turn(self, speed):
        self._bl_vel_cmd.data = speed
        self._br_vel_cmd.data = speed * -1.0
        self._fl_vel_cmd.data = speed
        self._fr_vel_cmd.data = speed * -1.0

        self._pub_bl_vel_cmd.publish(self._bl_vel_cmd)
        self._pub_br_vel_cmd.publish(self._br_vel_cmd)
        self._pub_fl_vel_cmd.publish(self._fl_vel_cmd)
        self._pub_fr_vel_cmd.publish(self._fr_vel_cmd)

    def stop(self):
        self._bl_vel_cmd.data = 0.0
        self._br_vel_cmd.data = 0.0
        self._fl_vel_cmd.data = 0.0
        self._fr_vel_cmd.data = 0.0

        self._pub_bl_vel_cmd.publish(self._bl_vel_cmd)
        self._pub_br_vel_cmd.publish(self._br_vel_cmd)
        self._pub_fl_vel_cmd.publish(self._fl_vel_cmd)
        self._pub_fr_vel_cmd.publish(self._fr_vel_cmd)


def main():
    controller = OmniRoverController()
    loop_rate_hz = 50.0
    loop_duration_s = 1.0 / loop_rate_hz

    try:
        speed = 10.0
        counter = 0
        delay_factor = 200
        while True:
            # move in a square
            if counter < 1 * delay_factor:
                controller.move_y(speed * -1.0)
            elif counter < 2 * delay_factor:
                controller.move_x(speed * -1.0)
            elif counter < 3 * delay_factor:
                controller.move_y(speed)
            elif counter < 4 * delay_factor:
                controller.move_x(speed)
            else:
                counter = -1

            counter += 1
            time.sleep(loop_duration_s)

    except KeyboardInterrupt:
        print("Exiting: stopping rover")

    # stop before exiting
    controller.stop()


if __name__ == "__main__":
    main()

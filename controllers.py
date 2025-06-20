import numpy as np

from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import Joy

import time

import rospy

class SpeedController:
    def __init__(self, kp:float = 0.50, ki:float = 0.04, kd:float = 0.00, ke:float = 0.00, target:float=0):
        self.kp:float = kp
        self.ki:float = ki
        self.kd:float = kd
        self.ke:float = ke

        self.wheel_rad  = 0.054
        self.wheel_circ = self.wheel_rad * 2 * np.pi
        self.raw_speed_multiplier = -self.wheel_circ / 60. / 7.73

        self.target:float = target

        self.buttons = None
        self.is_running_auto = False

        self.vesc_sub = rospy.Subscriber('/vesc/sensors/core', VescStateStamped, self.vesc_callback)
        self.joy_sub  = rospy.Subscriber('/vesc/joy', Joy, self.joy_callback)

        self.prev_time:float = time.time()
        self.prev_err:float = 0
        self.integral:float = 0
        self.feedback = 0.0

        self.output_power = 0.0

    def joy_callback(self, msg:Joy):
        self.buttons = msg.buttons
        was_running_auto = self.is_running_auto
        self.is_running_auto = self.buttons[5]

        if self.is_running_auto and not was_running_auto:
            self.reset()

    def vesc_callback(self, msg:VescStateStamped):
        self.feedback = msg.state.speed * self.raw_speed_multiplier
        self.output_power = self.update()

    def reset(self) -> None:
        self.integral = 0

    def set_target(self, target:float) -> None:
        if target != self.target:
            self.reset()
        self.target = target
    
    def update(self) -> float:
        curr_time = time.time()
        dt = curr_time - self.prev_time
        err = self.target - self.feedback

        self.integral += err*dt
        derivative:float = (err - self.prev_err) / dt if dt > 0 else 0
        output = (self.kp * self.target +
                  self.ki * self.integral +
                  self.kd * derivative +
                  self.ke * err)

        self.prev_err = err
        self.prev_time = curr_time

        return output


def state_pub():
    rospy.init_node('parallel_park_controller', anonymous=True)

    pparker = ParallelParkSupport()
    try:
        pparker.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    state_pub()



#!/usr/bin/python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Float32

from drone_connection.ar_controller import DroneController


class MotionController:
    def __init__(self):
        self.drone_controller = DroneController()

        self.delta = 1
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        # Liftoff and land
        self.sub_takeoff = rospy.Subscriber('/pxldrone/takeoff', Empty, self.take_off)
        self.sub_land    = rospy.Subscriber('/pxldrone/land', Empty, self.land)

        # Perfect control
        self.sub_pitch = rospy.Subscriber('/pxldrone/pitch', Float32, self.pitch_callback)
        self.sub_yaw   = rospy.Subscriber('/pxldrone/yaw_velocity', Float32, self.yaw_velocity_callback)
        self.sub_roll  = rospy.Subscriber('/pxldrone/roll', Float32, self.roll_callback)
        self.sub_z     = rospy.Subscriber('/pxldrone/z_velocity', Float32, self.z_velocity_callback)

    @classmethod
    def check_boundaries(cls, value):
        if value < -1.0:
            return -1.0
        elif 1 < value:
            return 1.0

        return value

    def pitch_callback(self, amount):
        self.pitch = self.check_boundaries(amount.data)
        self.set_command()

    def yaw_velocity_callback(self, amount):
        self.yaw_velocity = self.check_boundaries(amount.data)
        self.set_command()

    def roll_callback(self, amount):
        self.roll = self.check_boundaries(amount.data)
        self.set_command()

    def z_velocity_callback(self, amount):
        self.z_velocity = self.check_boundaries(amount.data)
        self.set_command()

    def set_command(self):
        self.drone_controller.set_command(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def take_off(self, empty):
        self.drone_controller.send_takeoff()

    def land(self, empty):
        self.drone_controller.send_land()

    def pitch_forward(self, empty):
        self.pitch += self.delta
        self.set_command()
        self.hover(Empty)

    def pitch_backward(self, empty):
        self.pitch -= self.delta
        self.set_command()

    def yaw_left(self, empty):
        self.yaw_velocity += self.delta
        self.set_command()

    def yaw_right(self, empty):
        self.yaw_velocity -= self.delta
        self.set_command()

    def roll_left(self, empty):
        self.roll += self.delta
        self.set_command()

    def roll_right(self, empty):
        self.roll -= self.delta
        self.set_command()

    def ascent(self, empty):
        self.z_velocity += self.delta
        self.set_command()

    def descent(self, empty):
        self.z_velocity -= self.delta
        self.set_command()

    def hover(self, empty):
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        self.set_command()

    def Autonomous(self, empty):
        pass

    def CameraSwitch(self, empty):
        pass

    def IncreaseSpeed(self, empty):
        self.delta += 0.1

    def DecreaseSpeed(self, empty):
        self.delta += 0.1

if __name__ == "__main__":
    rospy.init_node('motion_controller')

    motion_controller = MotionController()
    rospy.spin()

    rospy.signal_shutdown('motion_controller stopped.')

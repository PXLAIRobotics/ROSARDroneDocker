#
# Author: Tim Dupont <Tim.Dupont@PXL.BE>
#

import rospy

from geometry_msgs.msg import Twist  	  # for sending commands to the drone
from std_msgs.msg import Empty       	  # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback

from ar_status import DroneStatus

COMMAND_PERIOD = 100  # ms


class DroneController(object):

    def __init__(self):
        self.status = -1
        self.counter = 0

        self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receive_navdata)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)

        self.pub_command = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.send_command)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.send_land)

    def receive_navdata(self, navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment
        self.status = navdata.state

        if self.counter == 0:
            pass
            #rospy.loginfo("Received navdata")
            #print "Status: ", self.status

        self.counter += 1

        if self.counter == 100:
            self.counter = 0

    def send_takeoff(self):
        if self.status == DroneStatus.Landed:
            self.pub_takeoff.publish(Empty())

    def send_land(self):
        self.pub_land.publish(Empty())

    def send_reset(self): # In case of emergency
        self.pub_reset.publish(Empty())

    def set_command(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def send_command(self, event):
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            #rospy.loginfo("Sending command: %s", self.command)
            self.pub_command.publish(self.command)

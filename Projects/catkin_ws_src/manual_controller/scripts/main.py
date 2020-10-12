#!/usr/bin/python

from PySide2 import QtGui, QtCore, QtWidgets

import rospkg
import subprocess # For sound
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_srvs import srv

from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata

from threading import Lock

from key_mapper import KeyMapping

CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms



class DroneStatus(object):
    Emergency = 0
    Inited = 1
    Landed = 2
    Flying = 3
    Hovering = 4
    Test = 5
    TakingOff = 6
    GotoHover = 7
    Landing = 8
    Looping = 9


class Cockpit(QtWidgets.QMainWindow):
    StatusMessages = {
        DroneStatus.Emergency : 'Emergency',
        DroneStatus.Inited    : 'Initialized',
        DroneStatus.Landed    : 'Landed',
        DroneStatus.Flying    : 'Flying',
        DroneStatus.Hovering  : 'Hovering',
        DroneStatus.Test      : 'Test (?)',
        DroneStatus.TakingOff : 'Taking Off',
        DroneStatus.GotoHover : 'Going to Hover Mode',
        DroneStatus.Landing   : 'Landing',
        DroneStatus.Looping   : 'Looping (?)'
    }

    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'

    def __init__(self):
        super(Cockpit, self).__init__()

        self.setWindowTitle('Cockpit')
        
        self.label_image = QtWidgets.QLabel()
        self.setCentralWidget(self.label_image)

        self.pub_takeoff = rospy.Publisher('/pxldrone/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/pxldrone/land', Empty, queue_size=1)

        self.pub_pitch = rospy.Publisher('/pxldrone/pitch', Float32, queue_size=1)
        self.pub_yaw   = rospy.Publisher('/pxldrone/yaw_velocity', Float32, queue_size=1)
        self.pub_roll  = rospy.Publisher('/pxldrone/roll', Float32, queue_size=1)
        self.pub_z     = rospy.Publisher('/pxldrone/z_velocity', Float32, queue_size=1)

        self.sub_video = rospy.Subscriber('/ardrone/image_raw', Image, self.receive_image)
        self.sub_navdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.receive_navdata)

        self.status = "Not connected"
        self.image = None
        self.lock_image = Lock()

        self.status_message = ''

        self.new_communication = False
        self.connected = True

        self.timer_connection = QtCore.QTimer(self)
        self.timer_connection.timeout.connect(self.connection_callback)
        self.timer_connection.start(CONNECTION_CHECK_PERIOD)

        self.timer_redraw = QtCore.QTimer(self)
        self.timer_redraw.timeout.connect(self.redraw_callback)
        self.timer_redraw.start(GUI_UPDATE_PERIOD)

    def receive_image(self, data):
        self.new_communication = True
        self.lock_image.acquire()
        try:
            self.image = data
        finally:
            self.lock_image.release()

    def receive_navdata(self, data):
        msg = ""
        if data.state in self.StatusMessages:
            msg = self.StatusMessages[data.state]
        else:
            msg = self.UnknownMessage

        self.status = '{} (Battery: {}%)'.format(msg, int(data.batteryPercent))

    def connection_callback(self):
        self.connected = self.new_communication
        self.new_communication = False

    def redraw_callback(self):
        if self.image is not None:
            self.lock_image.acquire()

            try:
                format = QtGui.QImage.Format_RGB888
                image_converted = QtGui.QImage(self.image.data, self.image.width, self.image.height, format)
                pixmap = QtGui.QPixmap.fromImage(image_converted)
                # self.image = None
            finally:
                self.lock_image.release()

            # We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.

            self.label_image.resize(pixmap.width(), pixmap.height())
            self.label_image.setPixmap(pixmap)

            self.statusBar().showMessage(self.status if self.connected else self.DisconnectedMessage)

    def keyPressEvent(self, event):
        key = event.key()

        if self.connected and not event.isAutoRepeat():
            if key == KeyMapping.Emergency:
                pass
            elif key == KeyMapping.Takeoff:
                self.pub_takeoff.publish(Empty())
            elif key == KeyMapping.Land:
                self.pub_land.publish(Empty())
            else:
                if key == KeyMapping.YawLeft:
                    self.pub_yaw.publish(1.0)
                elif key == KeyMapping.YawRight:
                    self.pub_yaw.publish(-1.0)
                elif key == KeyMapping.PitchForward:
                    self.pub_pitch.publish(1.0)
                elif key == KeyMapping.PitchBackward:
                    self.pub_pitch.publish(-1.0)
                elif key == KeyMapping.RollLeft:
                    self.pub_roll.publish(1.0)
                elif key == KeyMapping.RollRight:
                    self.pub_roll.publish(-1.0)
                elif key == KeyMapping.IncreaseAltitude:
                    self.pub_z.publish(1.0)
                elif key == KeyMapping.DecreaseAltitude:
                    self.pub_z.publish(-1.0)
                elif key == KeyMapping.CameraSwitch:
                    rospy.ServiceProxy('/ardrone/togglecam', srv.Empty)()
                elif key == KeyMapping.FlatTrim:
                    rospy.ServiceProxy('/ardrone/flattrim', srv.Empty)()

    def keyReleaseEvent(self, event):
        key = event.key()

        if self.connected and not event.isAutoRepeat():
            if key == KeyMapping.YawLeft:
                self.pub_yaw.publish(0.0)
            elif key == KeyMapping.YawRight:
                self.pub_yaw.publish(0.0)
            elif key == KeyMapping.PitchForward:
                self.pub_pitch.publish(0.0)
            elif key == KeyMapping.PitchBackward:
                self.pub_pitch.publish(0.0)
            elif key == KeyMapping.RollLeft:
                self.pub_roll.publish(0.0)
            elif key == KeyMapping.RollRight:
                self.pub_roll.publish(0.0)
            elif key == KeyMapping.IncreaseAltitude:
                self.pub_z.publish(0.0)
            elif key == KeyMapping.DecreaseAltitude:
                self.pub_z.publish(0.0)

if __name__ == "__main__":
    path = rospkg.RosPack().get_path("manual_controller")

    import sys
    rospy.init_node('manual_controller')
    app = QtWidgets.QApplication(sys.argv)
    cockpit = Cockpit()
    cockpit.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)

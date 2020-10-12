try:
    from PySide import QtCore
except ImportError:
    from PySide2 import QtCore

class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key_Up
    PitchBackward    = QtCore.Qt.Key_Down
    RollLeft         = QtCore.Qt.Key_W
    RollRight        = QtCore.Qt.Key_E
    YawLeft          = QtCore.Qt.Key_Left
    YawRight         = QtCore.Qt.Key_Right
    IncreaseAltitude = QtCore.Qt.Key_S
    DecreaseAltitude = QtCore.Qt.Key_D
    Takeoff          = QtCore.Qt.Key_T
    Land             = QtCore.Qt.Key_L
    Autonomous       = QtCore.Qt.Key_A
    CameraSwitch     = QtCore.Qt.Key_C
    FlatTrim         = QtCore.Qt.Key_F
    Emergency        = QtCore.Qt.Key_Space

import rospy
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import dlib
from imutils import face_utils
import math

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1300, 850)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QtCore.QSize(1300, 850))
        MainWindow.setMaximumSize(QtCore.QSize(1300, 850))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(238, 238, 236))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        MainWindow.setPalette(palette)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        MainWindow.setFont(font)
        MainWindow.setWindowOpacity(1.0)
        MainWindow.setStyleSheet("background-color: rgb(238, 238, 236);\n")
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        MainWindow.setDocumentMode(False)
        MainWindow.setUnifiedTitleAndToolBarOnMac(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(9, 9, 481, 141))
        self.layoutWidget.setObjectName("layoutWidget")
        self.position_layout = QtWidgets.QFormLayout(self.layoutWidget)
        self.position_layout.setContentsMargins(0, 0, 0, 0)
        self.position_layout.setObjectName("position_layout")
        self.label_position = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_position.setFont(font)
        self.label_position.setStyleSheet("color: rgb(61, 52, 139)")
        self.label_position.setObjectName("label_position")
        self.position_layout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_position)
        self.label_x = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_x.setFont(font)
        self.label_x.setStyleSheet("color: rgb(230, 175, 46);")
        self.label_x.setObjectName("label_x")
        self.position_layout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_x)
        self.line_x = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_x.sizePolicy().hasHeightForWidth())
        self.line_x.setSizePolicy(sizePolicy)
        self.line_x.setMinimumSize(QtCore.QSize(15, 15))
        self.line_x.setMaximumSize(QtCore.QSize(300, 50))
        self.line_x.setStyleSheet("background-color: rgb(172, 190, 216);")
        self.line_x.setObjectName("line_x")
        self.position_layout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.line_x)
        self.label_y = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_y.setFont(font)
        self.label_y.setStyleSheet("color: rgb(230, 175, 46);")
        self.label_y.setObjectName("label_y")
        self.position_layout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_y)
        self.line_y = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_y.sizePolicy().hasHeightForWidth())
        self.line_y.setSizePolicy(sizePolicy)
        self.line_y.setMaximumSize(QtCore.QSize(300, 50))
        self.line_y.setStyleSheet("background-color: rgb(172, 190, 216);")
        self.line_y.setObjectName("line_y")
        self.position_layout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.line_y)
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(680, 600, 411, 151))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.control_layout = QtWidgets.QGridLayout(self.layoutWidget1)
        self.control_layout.setContentsMargins(0, 0, 0, 0)
        self.control_layout.setObjectName("control_layout")
        self.stop_button = QtWidgets.QPushButton(self.layoutWidget1)
        self.stop_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                       "color: rgb(238, 238, 236);r")
        icon = QtGui.QIcon.fromTheme("stop")
        self.stop_button.setIcon(icon)
        self.stop_button.setObjectName("stop_button")
        self.control_layout.addWidget(self.stop_button, 2, 1, 1, 1)
        self.backward_button = QtWidgets.QPushButton(self.layoutWidget1)
        self.backward_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                           "color: rgb(238, 238, 236);r")
        icon = QtGui.QIcon.fromTheme("down")
        self.backward_button.setIcon(icon)
        self.backward_button.setObjectName("backward_button")
        self.control_layout.addWidget(self.backward_button, 3, 1, 1, 1)
        self.right_button = QtWidgets.QPushButton(self.layoutWidget1)
        self.right_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                        "color: rgb(238, 238, 236);r")
        icon = QtGui.QIcon.fromTheme("forward")
        self.right_button.setIcon(icon)
        self.right_button.setObjectName("right_button")
        self.control_layout.addWidget(self.right_button, 2, 2, 1, 1)
        self.left_button = QtWidgets.QPushButton(self.layoutWidget1)
        self.left_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                       "color: rgb(238, 238, 236);r")
        icon = QtGui.QIcon.fromTheme("back")
        self.left_button.setIcon(icon)
        self.left_button.setObjectName("left_button")
        self.control_layout.addWidget(self.left_button, 2, 0, 1, 1)
        self.forward_button = QtWidgets.QPushButton(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        font.setKerning(True)
        self.forward_button.setFont(font)
        self.forward_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                          "color: rgb(238, 238, 236);r")
        icon = QtGui.QIcon.fromTheme("up")
        self.forward_button.setIcon(icon)
        self.forward_button.setObjectName("forward_button")
        self.control_layout.addWidget(self.forward_button, 1, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        font.setStrikeOut(False)
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.label.setFont(font)
        self.label.setStyleSheet("color: rgb(230, 175, 46);")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.control_layout.addWidget(self.label, 0, 1, 1, 1)
        self.layoutWidget2 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget2.setGeometry(QtCore.QRect(10, 150, 481, 111))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.vertical_layout = QtWidgets.QFormLayout(self.layoutWidget2)
        self.vertical_layout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.vertical_layout.setContentsMargins(0, 0, 0, 0)
        self.vertical_layout.setSpacing(6)
        self.vertical_layout.setObjectName("vertical_layout")
        self.label_velocity = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_velocity.setFont(font)
        self.label_velocity.setStyleSheet("color: rgb(61, 52, 139)")
        self.label_velocity.setObjectName("label_velocity")
        self.vertical_layout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_velocity)
        self.label_linear = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_linear.setFont(font)
        self.label_linear.setStyleSheet("color: rgb(230, 175, 46);")
        self.label_linear.setObjectName("label_linear")
        self.vertical_layout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_linear)
        self.line_linear = QtWidgets.QLineEdit(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_linear.sizePolicy().hasHeightForWidth())
        self.line_linear.setSizePolicy(sizePolicy)
        self.line_linear.setMaximumSize(QtCore.QSize(300, 50))
        self.line_linear.setStyleSheet("background-color: rgb(172, 190, 216);")
        self.line_linear.setObjectName("line_linear")
        self.vertical_layout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.line_linear)
        self.label_angular = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_angular.setFont(font)
        self.label_angular.setStyleSheet("color: rgb(230, 175, 46);")
        self.label_angular.setObjectName("label_angular")
        self.vertical_layout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.label_angular)
        self.line_angular = QtWidgets.QLineEdit(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_angular.sizePolicy().hasHeightForWidth())
        self.line_angular.setSizePolicy(sizePolicy)
        self.line_angular.setMaximumSize(QtCore.QSize(300, 50))
        self.line_angular.setStyleSheet("background-color: rgb(172, 190, 216);")
        self.line_angular.setObjectName("line_angular")
        self.vertical_layout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.line_angular)
        self.map_button = QtWidgets.QPushButton(self.centralwidget)
        self.map_button.setGeometry(QtCore.QRect(810, 540, 190, 25))
        self.map_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                      "color: rgb(238, 238, 236);r")
        self.map_button.setObjectName("map_button")

        self.label_map_view = QtWidgets.QLabel(self.centralwidget)
        self.label_map_view.setGeometry(QtCore.QRect(540, 30, 709, 497))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_map_view.sizePolicy().hasHeightForWidth())
        self.label_map_view.setSizePolicy(sizePolicy)
        self.label_map_view.setMinimumSize(QtCore.QSize(709, 497))
        self.label_map_view.setMaximumSize(QtCore.QSize(709, 497))
        self.label_map_view.setStyleSheet("")
        self.label_map_view.setText("")
        self.label_map_view.setPixmap(QtGui.QPixmap("../catkin_ws/src/project_ws/src/house_gazebo.png"))
        self.label_map_view.setAlignment(QtCore.Qt.AlignLeading | QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.label_map_view.setObjectName("label_map_view")
        self.label_map = QtWidgets.QLabel(self.centralwidget)
        self.label_map.setGeometry(QtCore.QRect(870, 0, 41, 17))
        self.label_map.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_map.setFont(font)
        self.label_map.setStyleSheet("color: rgb(61, 52, 139)")
        self.label_map.setAlignment(QtCore.Qt.AlignCenter)
        self.label_map.setObjectName("label_map")
        self.label_camera = QtWidgets.QLabel(self.centralwidget)
        self.label_camera.setGeometry(QtCore.QRect(20, 280, 501, 451))
        self.label_camera.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_camera.setText("")

        self.label_camera.setAlignment(QtCore.Qt.AlignCenter)
        self.label_camera.setOpenExternalLinks(False)
        self.label_camera.setObjectName("label_camera")
        self.camera_button = QtWidgets.QPushButton(self.centralwidget)
        self.camera_button.setGeometry(QtCore.QRect(55, 250, 111, 25))
        self.camera_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                         "color: rgb(238, 238, 236);r")
        self.camera_button.setIconSize(QtCore.QSize(16, 16))
        self.camera_button.setAutoDefault(False)
        self.camera_button.setDefault(False)
        self.camera_button.setFlat(False)
        self.camera_button.setObjectName("camera_button")
        
        self.camera_cancel_button = QtWidgets.QPushButton(self.centralwidget)
        self.camera_cancel_button.setGeometry(QtCore.QRect(230, 250, 111, 25))
        self.camera_cancel_button.setStyleSheet("background-color: rgb(61, 52, 139);\n"
                                         "color: rgb(238, 238, 236);r")
        self.camera_cancel_button.setIconSize(QtCore.QSize(16, 16))
        self.camera_cancel_button.setAutoDefault(False)
        self.camera_cancel_button.setDefault(False)
        self.camera_cancel_button.setFlat(False)
        self.camera_cancel_button.setObjectName("camera_cancel_button")
        
        
        
        self.goal1_button = QtWidgets.QPushButton(self.centralwidget)
        self.goal1_button.setGeometry(QtCore.QRect(933, 133, 14, 14))
        self.goal1_button.setAutoFillBackground(False)
        self.goal1_button.setStyleSheet("background-color: rgb(0, 128, 0);")
        self.goal1_button.setText("")
        self.goal1_button.setObjectName("goal1_button")
        self.goal2_button = QtWidgets.QPushButton(self.centralwidget)
        self.goal2_button.setGeometry(QtCore.QRect(1167, 300, 14, 14))
        self.goal2_button.setAutoFillBackground(False)
        self.goal2_button.setStyleSheet("background-color: rgb(0, 128, 0);")
        self.goal2_button.setText("")
        self.goal2_button.setObjectName("goal2_button")
        self.goal3_button = QtWidgets.QPushButton(self.centralwidget)
        self.goal3_button.setGeometry(QtCore.QRect(607, 367, 14, 14))
        self.goal3_button.setAutoFillBackground(False)
        self.goal3_button.setStyleSheet("background-color: rgb(0, 128, 0);")
        self.goal3_button.setText("")
        self.goal3_button.setObjectName("goal3_button")
        self.label_movement = QtWidgets.QLabel(self.centralwidget)
        self.label_movement.setGeometry(QtCore.QRect(570, 570, 171, 25))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_movement.setFont(font)
        self.label_movement.setStyleSheet("color: rgb(230, 175, 46);")
        self.label_movement.setObjectName("label_movement")
        self.line_movement = QtWidgets.QLineEdit(self.centralwidget)
        self.line_movement.setGeometry(QtCore.QRect(750, 570, 300, 25))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_movement.sizePolicy().hasHeightForWidth())
        self.line_movement.setSizePolicy(sizePolicy)
        self.line_movement.setMaximumSize(QtCore.QSize(300, 50))
        self.line_movement.setStyleSheet("background-color: rgb(172, 190, 216);")
        self.line_movement.setObjectName("line_movement")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1300, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        p = "../catkin_ws/src/project_ws/src/shape_predictor_68_face_landmarks.dat"
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(p)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robot Controller Interface"))
        self.label_position.setText(_translate("MainWindow", "Position Indicator"))
        self.label_x.setText(_translate("MainWindow", "Position x:"))
        self.label_y.setText(_translate("MainWindow", "Position y:"))
        self.stop_button.setText(_translate("MainWindow", "Stop"))
        self.backward_button.setText(_translate("MainWindow", "Backward"))
        self.right_button.setText(_translate("MainWindow", "Right"))
        self.left_button.setText(_translate("MainWindow", "Left"))
        self.forward_button.setText(_translate("MainWindow", "Forward"))
        self.label.setText(_translate("MainWindow", "Robot Controller"))
        self.label_velocity.setText(_translate("MainWindow", "Velocity Indicator"))
        self.label_linear.setText(_translate("MainWindow", "Linear Velocity:"))
        self.label_angular.setText(_translate("MainWindow", "Angular Velocity:"))
        self.map_button.setText(_translate("MainWindow", "Stop Autonomous Driving"))
        self.label_map.setText(_translate("MainWindow", "MAP"))
        self.camera_button.setText(_translate("MainWindow", "Start Camera"))
        self.label_movement.setText(_translate("MainWindow", "Direction of Movement"))
        self.camera_cancel_button.setText(_translate("MainWindow", "Stop Camera"))

        rospy.init_node('yendoc')
        self.pub = rospy.Publisher('/vehiclediffdrive/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        rospy.Subscriber('/vehiclediffdrive/odom', Odometry, self.odomCallback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()

        self.stop_button.clicked.connect(self.stop)
        self.stop_button.setShortcut("s")
        self.forward_button.clicked.connect(self.forward)
        self.forward_button.setShortcut("w")
        self.backward_button.clicked.connect(self.backward)
        self.backward_button.setShortcut("x")
        self.right_button.clicked.connect(self.turnRight)
        self.right_button.setShortcut("d")
        self.left_button.clicked.connect(self.turnLeft)
        self.left_button.setShortcut("a")
        self.line_angular.setText(str(0.0))
        self.line_linear.setText(str(0.0))
        self.line_x.setText(str(0.0))
        self.line_y.setText(str(0.0))
        self.camera_button.clicked.connect(self.viewCam)
        self.camera_cancel_button.clicked.connect(self.buttonStopCamera)
        self.map_button.clicked.connect(self.cancel_movement)
        self.goal1_button.clicked.connect(self.moveToGoal1)
        self.goal2_button.clicked.connect(self.moveToGoal2)
        self.goal3_button.clicked.connect(self.moveToGoal3)
        
    def moveToGoal1(self, msg):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = 1.0
        self.goal.target_pose.pose.position.y = 3.0
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.odomCallback(msg)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.signal_shutdown("Action Server DOWN!")
        else:
            return self.client.get_result()
            print("wait", wait)

    def moveToGoal2(self, msg):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = 6.0
        self.goal.target_pose.pose.position.y = -1.0
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.odomCallback(msg)
        wait = self.client.wait_for_result()
        while wait:
            self.line_linear.setText(str(round(self.vel_msg.linear.x)))
            self.line_angular.setText(str(round(self.vel_msg.angular.z)))
        if not wait:
            rospy.signal_shutdown("Action Server DOWN!")
        else:
            return self.client.get_result()
            print("wait", wait)

    def moveToGoal3(self, msg):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = -6.0
        self.goal.target_pose.pose.position.y = -2.0
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.odomCallback(msg)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.signal_shutdown("Action Server DOWN!")
        else:
            return self.client.get_result()
            print("wait", wait)

    def head_control(self, head):
        if head == 0:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        elif head == 1:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = -0.3
        elif head == 2:
            self.vel_msg.linear.x = 0.1
            self.vel_msg.angular.z = 0
        elif head == 3:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = +0.3
        elif head == 4:
            self.vel_msg.linear.x = -0.1
            self.vel_msg.angular.z = 0
        self.line_linear.setText(str(round(self.vel_msg.linear.x)))
        self.line_angular.setText(str(round(self.vel_msg.angular.z)))
        self.pub.publish(self.vel_msg)
   

    def cancel_movement(self):
        self.client.cancel_goal()
        rospy.loginfo("goal has been canceled")

    def velCallback(self, vel_msg):
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def odomCallback(self, msg):
        self.line_x.setText(str(round(msg.pose.pose.position.x, 4)))
        self.line_y.setText(str(round(msg.pose.pose.position.y, 4)))
        radius = 5
        color = (0, 0, 255)
        thickness = -1
        pixX=int(((round(msg.pose.pose.position.x, 1)+5.7)*709)/11.4)
        pixY=int((abs(3.4-round(msg.pose.pose.position.y, 1))*497)/6.8)
        image_input = cv2.imread("../catkin_ws/src/project_ws/src/house_gazebo.png")
        self.image_output = cv2.circle(image_input, (pixX,pixY), radius, color, thickness)     
        self.src = cv2.cvtColor(self.image_output, cv2.COLOR_BGR2RGB)          
        h, w, ch = self.src.shape
        bytesPerLine = ch * w
        self.qimage = QtGui.QImage(self.src.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)  
        self.label_map_view.setPixmap(QPixmap.fromImage(self.qimage))

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.pub.publish(self.vel_msg)
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def forward(self):
        self.vel_msg.linear.x = 0.1
        self.vel_msg.angular.z = 0.0
        self.pub.publish(self.vel_msg)
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def backward(self):
        self.vel_msg.linear.x = -0.1
        self.vel_msg.angular.z = 0.0
        self.pub.publish(self.vel_msg)
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def turnRight(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = -0.3
        self.pub.publish(self.vel_msg)
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def turnLeft(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.3
        self.pub.publish(self.vel_msg)
        self.line_linear.setText(str(self.vel_msg.linear.x))
        self.line_angular.setText(str(self.vel_msg.angular.z))

    def viewCam(self):
        self.cap = cv2.VideoCapture(0)
        while True:
            _, image = self.cap.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            rects = self.detector(gray, 0)
            for (i, rect) in enumerate(rects):
                shape = self.predictor(gray, rect)
                shape = face_utils.shape_to_np(shape)

                for (x, y) in shape:
                    cv2.circle(image, (x, y), 2, (0, 255, 0), -1)

                sag = math.sqrt(pow((shape[36, 0] - shape[27, 0]), 2) + pow((shape[36, 1] - shape[27, 1]), 2))
                sol = math.sqrt(pow((shape[45, 0] - shape[27, 0]), 2) + pow((shape[45, 1] - shape[27, 1]), 2))
                const = ((shape[45, 1] - shape[27, 1]) + (shape[36, 1] - shape[27, 1])) / 2
                if (sol < sag + 7 and sag < sol) or (sag < sol + 7 and sol < sag):
                    if const > 10.0:
                        print("YUKARI")
                        self.line_movement.setText("FORWARD")
                        self.head_control(2)
                    elif const < 0.0:
                        print("ASAGI")
                        self.line_movement.setText("BACKWARD")
                        self.head_control(4)
                elif sol < sag + 7:
                    print("SOL")
                    self.line_movement.setText("LEFT")
                    self.head_control(3)
                elif sag < sol + 7:
                    print("SAĞ")
                    self.line_movement.setText("RIGHT")
                    self.head_control(1)

                # print(shape)
                cv2.line(image, (shape[36, 0], shape[36, 1]), (shape[27, 0], shape[27, 1]), (255, 0, 0), 2)
                cv2.line(image, (shape[45, 0], shape[45, 1]), (shape[27, 0], shape[27, 1]), (0, 255, 0), 2)
            # show the output image with the face detections + facial landmarks
            cv2.namedWindow("Face Detection")        # Create a named window
            cv2.moveWindow("Face Detection", 100, 400)  # Move it to (40,30)           
            thumbnail = cv2.resize(image, (450, 300), interpolation=cv2.INTER_AREA)
            cv2.imshow("Face Detection", thumbnail)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    def buttonStopCamera(self):
        cv2.destroyAllWindows()
        self.cap.release()

    def controlTimer(self):
        if not self.timer.isActive():
            self.cap = cv2.VideoCapture(0)
            self.timer.start(10)
            self.camera_button.setText("Stop Camera")
        else:
            self.timer.stop()
            self.cap.release()
            self.camera_button.setText("Start Camera")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

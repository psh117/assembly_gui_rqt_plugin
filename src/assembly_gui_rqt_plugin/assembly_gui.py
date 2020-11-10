from __future__ import print_function
import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from assembly_msgs.srv import IdleControl, IdleControlRequest
from franka_gripper.msg import GraspAction, MoveAction, GraspEpsilon, MoveGoal, GraspGoal
from jrk_motor_service.srv import JrkCmdRequest, JrkCmd
from franka_msgs.srv import SetLoad, SetLoadRequest
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from assembly_dxl_gripper.srv import Move, MoveRequest
import actionlib

class AssemblyGuiPlugin(Plugin):

    def __init__(self, context):
        super(AssemblyGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('AssemblyGuiPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('assembly_gui_rqt_plugin'), 'resource', 'AssemblyGuiPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('AssemblyGuiPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.drill_proxy = rospy.ServiceProxy('/jrk_cmd', JrkCmd)
        self.idle_proxy = rospy.ServiceProxy('/assembly_dual_controller/idle_control',IdleControl)
        self.switch_proxy = rospy.ServiceProxy('/panda_dual/controller_manager/switch_controller',SwitchController)
        self.left_load_proxy = rospy.ServiceProxy('/panda_dual/panda_left/set_load',SetLoad)
        self.right_load_proxy = rospy.ServiceProxy('/panda_dual/panda_right/set_load',SetLoad)
        self.top_load_proxy = rospy.ServiceProxy('/panda_dual/panda_top/set_load',SetLoad)
        self.dxl_gripper_proxy = rospy.ServiceProxy('/assembly_dxl_gripper/move',Move)

        self.error_recov_client = actionlib.SimpleActionClient('/panda_dual/error_recovery',ErrorRecoveryAction)
        self.panda_gripper_move_client = actionlib.SimpleActionClient('/panda_left_gripper/move',MoveAction)
        self.panda_gripper_grasp_client = actionlib.SimpleActionClient('/panda_left_gripper/grasp',GraspAction)

        # connect        
        self._widget.btn_drill_long_stop.pressed.connect(self.btn_drill_long_stop_on_click)
        self._widget.btn_drill_long_run.pressed.connect(self.btn_drill_long_run_on_click)
        self._widget.btn_drill_short_stop.pressed.connect(self.btn_drill_short_stop_on_click)
        self._widget.btn_drill_short_run.pressed.connect(self.btn_drill_short_run_on_click)
        self._widget.btn_unlock_error.pressed.connect(self.btn_unlock_error_on_click)

        self._widget.btn_left_gripper_close.pressed.connect(self.btn_left_gripper_close_on_click)
        self._widget.btn_left_gripper_open.pressed.connect(self.btn_left_gripper_open_on_click)
        self._widget.btn_left_init_joint.pressed.connect(self.btn_left_init_joint_on_click)
        self._widget.btn_left_set_load_drill.pressed.connect(self.btn_left_set_load_drill_on_click)
        self._widget.btn_left_set_load_zero.pressed.connect(self.btn_left_set_load_zero_on_click)

        self._widget.btn_right_gripper_close.pressed.connect(self.btn_right_gripper_close_on_click)
        self._widget.btn_right_gripper_open.pressed.connect(self.btn_right_gripper_open_on_click)
        self._widget.btn_right_init_joint.pressed.connect(self.btn_right_init_joint_on_click)
        self._widget.btn_right_set_load_drill.pressed.connect(self.btn_right_set_load_drill_on_click)
        self._widget.btn_right_set_load_zero.pressed.connect(self.btn_right_set_load_zero_on_click)

        self._widget.btn_top_gripper_close.pressed.connect(self.btn_top_gripper_close_on_click)
        self._widget.btn_top_gripper_open.pressed.connect(self.btn_top_gripper_open_on_click)
        self._widget.btn_top_init_joint.pressed.connect(self.btn_top_init_joint_on_click)
        self._widget.btn_top_set_load_drill.pressed.connect(self.btn_top_set_load_drill_on_click)
        self._widget.btn_top_set_load_zero.pressed.connect(self.btn_top_set_load_zero_on_click)

        self._widget.btn_init_joint_all.pressed.connect(self.btn_init_joint_all_on_click)

    def btn_top_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        print(req)
        self.switch_proxy(stop)
        rospy.sleep(0.5)
        self.top_load_proxy(req)
        self.switch_proxy(start)

    def btn_top_set_load_drill_on_click(self):
        req = SetLoadRequest(mass=1.114,
                             F_x_center_load = [0.00, 0.0, 0.0415])
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        print(req)
        self.switch_proxy(stop)
        rospy.sleep(0.5)
        self.top_load_proxy(req)
        self.switch_proxy(start)

    def btn_top_init_joint_on_click(self):
        pass

    def btn_top_gripper_open_on_click(self):
        req = MoveRequest(hand = ['panda_top'],
                          max_current = [0.0],
                          length = [0.075])
        print(req)
        self.dxl_gripper_proxy(req)

    def btn_top_gripper_close_on_click(self):
        req = MoveRequest(hand = ['panda_top'],
                          max_current = [self._widget.spinbox_top_gripper_force.value()],
                          length = [self._widget.spinbox_top_gripper_width.value()])
        print(req)
        self.dxl_gripper_proxy(req)

    def btn_right_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        print(req)
        self.switch_proxy(stop)
        rospy.sleep(0.5)
        self.right_load_proxy(req)
        self.switch_proxy(start)

    def btn_right_set_load_drill_on_click(self):
        req = SetLoadRequest(mass=1.114,
                             F_x_center_load = [-0.01, 0.0, 0.0415])
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        print(req)
        self.switch_proxy(stop)
        rospy.sleep(0.5)
        self.right_load_proxy(req)
        self.switch_proxy(start)

    def btn_right_init_joint_on_click(self):
        pass

    def btn_right_gripper_open_on_click(self):
        req = MoveRequest(hand = ['panda_right'],
                          max_current = [0.0],
                          length = [0.075])
        print(req)
        self.dxl_gripper_proxy(req)

    def btn_right_gripper_close_on_click(self):
        req = MoveRequest(hand = ['panda_right'],
                          max_current = [self._widget.spinbox_right_gripper_force.value()],
                          length = [self._widget.spinbox_right_gripper_width.value()])
        print(req)
        self.dxl_gripper_proxy(req)

    def btn_left_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        print(req)
        self.switch_proxy(stop)
        rospy.sleep(0.5)
        self.left_load_proxy(req)
        self.switch_proxy(start)

    def btn_left_set_load_drill_on_click(self):
        print("---not working---")

    def btn_left_init_joint_on_click(self):
        pass

    def btn_init_joint_all_on_click(self):
        pass

    def btn_left_gripper_open_on_click(self):
        req = MoveGoal(width=0.075,speed=0.1)
        print(req)
        self.panda_gripper_move_client.send_goal_and_wait(req)

    def btn_left_gripper_close_on_click(self):
        epsilon = GraspEpsilon(inner=0.027, outer=0.027)
        req = GraspGoal(width=self._widget.spinbox_left_gripper_width.value(),
                          epsilon=epsilon,
                          speed=0.1,
                          force=self._widget.spinbox_left_gripper_force.value())
        print(req)
        self.panda_gripper_grasp_client.send_goal_and_wait(req)

    def btn_drill_long_run_on_click(self):
        req = JrkCmdRequest()
        req.name = 'right_long'
        req.target_value = self._widget.spinbox_drill_long_power.value()
        print(req)
        self.drill_proxy(req)

    def btn_drill_long_stop_on_click(self):
        req = JrkCmdRequest()
        req.name = 'right_long'
        req.target_value = 0.0
        print(req)
        self.drill_proxy(req)

    def btn_drill_short_run_on_click(self):
        req = JrkCmdRequest()
        req.name = 'top_long'
        req.target_value = self._widget.spinbox_drill_short_power.value()
        print(req)
        self.drill_proxy(req)

    def btn_drill_short_stop_on_click(self):
        req = JrkCmdRequest()
        req.name = 'top_long'
        req.target_value = 0.0
        print(req)
        self.drill_proxy(req)

    def btn_unlock_error_on_click(self):
        idle_control_l0 = IdleControlRequest()
        idle_control_l0.arm_name = 'panda_left'
        idle_control_r0 = IdleControlRequest()
        idle_control_r0.arm_name = 'panda_right'
        idle_control_t0 = IdleControlRequest()
        idle_control_t0.arm_name = 'panda_top'
        idle_control_l2 = IdleControlRequest()
        idle_control_l2.arm_name = 'panda_left'
        idle_control_l2.mode = 2
        idle_control_r2 = IdleControlRequest()
        idle_control_r2.arm_name = 'panda_right'
        idle_control_r2.mode = 2
        idle_control_t2 = IdleControlRequest()
        idle_control_t2.arm_name = 'panda_top'
        idle_control_t2.mode = 2
        error_recovery_goal = ErrorRecoveryActionGoal()

        self.idle_proxy(idle_control_l0)
        self.idle_proxy(idle_control_r0)
        self.idle_proxy(idle_control_t0)
        self.idle_proxy(idle_control_l2)
        self.idle_proxy(idle_control_r2)
        self.idle_proxy(idle_control_t2)
        self.error_recov_client.send_goal_and_wait(error_recovery_goal)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
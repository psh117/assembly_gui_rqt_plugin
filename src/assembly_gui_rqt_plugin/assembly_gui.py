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
from actionlib_msgs.msg import GoalStatus
from assembly_task_manager.params.default_parameters import short_bolt_drill_load, long_bolt_drill_load, side_left_load, side_right_load
from assembly_task_manager.params.default_parameters import long_load, short_load, middle_load, bottom_load, chair_load

# ## when testing -- not using default_parameters
# short_bolt_drill_load = SetLoadRequest(mass=0.0, F_x_center_load = [0.0, 0.0, 0.0])
# long_bolt_drill_load = short_bolt_drill_load
# side_left_load = short_bolt_drill_load
# side_right_load = short_bolt_drill_load
# long_load = short_bolt_drill_load
# short_load = short_bolt_drill_load
# middle_load = short_bolt_drill_load
# bottom_load = short_bolt_drill_load
# chair_load = short_bolt_drill_load

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

        item_list = ['zero','drill_short','drill_long','side_left','side_right','long','short','middle','bottom','chair']
        self.load_dict = {}
        self.load_dict['zero'] = SetLoadRequest(mass=0.0, F_x_center_load = [0.0, 0.0, 0.0])
        self.load_dict['drill_short'] = short_bolt_drill_load
        self.load_dict['drill_long'] = long_bolt_drill_load
        self.load_dict['side_left'] = side_left_load
        self.load_dict['side_right'] = side_right_load
        self.load_dict['long'] = long_load
        self.load_dict['short'] = short_load
        self.load_dict['middle'] = middle_load
        self.load_dict['bottom'] = bottom_load
        self.load_dict['chair'] = chair_load

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
        self._widget.btn_left_set_load.pressed.connect(self.btn_left_set_load_on_click)
        self._widget.btn_left_set_load_zero.pressed.connect(self.btn_left_set_load_zero_on_click)

        self._widget.btn_right_gripper_close.pressed.connect(self.btn_right_gripper_close_on_click)
        self._widget.btn_right_gripper_open.pressed.connect(self.btn_right_gripper_open_on_click)
        self._widget.btn_right_init_joint.pressed.connect(self.btn_right_init_joint_on_click)
        self._widget.btn_right_set_load.pressed.connect(self.btn_right_set_load_on_click)
        self._widget.btn_right_set_load_zero.pressed.connect(self.btn_right_set_load_zero_on_click)

        self._widget.btn_top_gripper_close.pressed.connect(self.btn_top_gripper_close_on_click)
        self._widget.btn_top_gripper_open.pressed.connect(self.btn_top_gripper_open_on_click)
        self._widget.btn_top_init_joint.pressed.connect(self.btn_top_init_joint_on_click)
        self._widget.btn_top_set_load.pressed.connect(self.btn_top_set_load_on_click)
        self._widget.btn_top_set_load_zero.pressed.connect(self.btn_top_set_load_zero_on_click)

        self._widget.btn_init_joint_all.pressed.connect(self.btn_init_joint_all_on_click)
        for item in item_list:
            self._widget.combo_left_load_item.addItem(item)
            self._widget.combo_right_load_item.addItem(item)
            self._widget.combo_top_load_item.addItem(item)

        self._widget.textBrowser.append("---------------------------------------")
        self._widget.textBrowser.append("author: KIM-HC, psh117")
        self._widget.textBrowser.append("---------------------------------------")

    def btn_top_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.top_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("TOP: set_load_zero")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- TOP: set_load_zero")

    def btn_top_set_load_on_click(self):
        item = self._widget.combo_top_load_item.currentText()
        req = self.load_dict[item]
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.top_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("TOP: set_load_"+item)
            self._widget.textBrowser.append(str(self.load_dict[item]))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- TOP: set_load")

    def btn_top_init_joint_on_click(self):
        pass

    def btn_top_gripper_open_on_click(self):
        req = MoveRequest(hand = ['panda_top'],
                          max_current = [0.0],
                          length = [0.075])
        try:
            self.dxl_gripper_proxy(req)
            self._widget.textBrowser.append("TOP: gripper_open")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- TOP: gripper_open")

    def btn_top_gripper_close_on_click(self):
        current = self._widget.spinbox_top_gripper_force.value()
        length = self._widget.spinbox_top_gripper_width.value()
        req = MoveRequest(hand = ['panda_top'],
                          max_current = [current],
                          length = [length])
        try:
            self.dxl_gripper_proxy(req)
            self._widget.textBrowser.append("TOP: gripper_close | current: "+str(current)+" | length: "+str(length))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- TOP: gripper_close")

    def btn_right_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.right_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("RIGHT: set_load_zero")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- RIGHT: set_load_zero")

    def btn_right_set_load_on_click(self):
        item = self._widget.combo_right_load_item.currentText()
        req = self.load_dict[item]
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.right_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("RIGHT: set_load_"+item)
            self._widget.textBrowser.append(str(self.load_dict[item]))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- RIGHT: set_load_"+item)

    def btn_right_init_joint_on_click(self):
        pass

    def btn_right_gripper_open_on_click(self):
        req = MoveRequest(hand = ['panda_right'],
                          max_current = [0.0],
                          length = [0.075])
        try:
            self.dxl_gripper_proxy(req)
            self._widget.textBrowser.append("RIGHT: gripper_open")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- RIGHT: gripper_open")

    def btn_right_gripper_close_on_click(self):
        current = self._widget.spinbox_right_gripper_force.value()
        length = self._widget.spinbox_right_gripper_width.value()
        req = MoveRequest(hand = ['panda_right'],
                          max_current = [current],
                          length = [length])
        try:
            self.dxl_gripper_proxy(req)
            self._widget.textBrowser.append("RIGHT: gripper_close | current: "+str(current)+" | length: "+str(length))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- RIGHT: gripper_close")

    def btn_left_set_load_zero_on_click(self):
        req = SetLoadRequest(mass=0.0)
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.left_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("LEFT: set_load_zero")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- LEFT: set_load_zero")

    def btn_left_set_load_on_click(self):
        item = self._widget.combo_left_load_item.currentText()
        req = self.load_dict[item]
        stop = SwitchControllerRequest(stop_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        start = SwitchControllerRequest(start_controllers=['assembly_triple_controller'],
                                       strictness = 2)
        try:
            self.switch_proxy(stop)
            rospy.sleep(0.5)
            self.left_load_proxy(req)
            self.switch_proxy(start)
            self._widget.textBrowser.append("LEFT: set_load_"+item)
            self._widget.textBrowser.append(str(self.load_dict[item]))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- LEFT: set_load_"+item)

    def btn_left_init_joint_on_click(self):
        pass

    def btn_init_joint_all_on_click(self):
        pass

    def btn_left_gripper_open_on_click(self):
        req = MoveGoal(width=0.075,speed=0.1)

        if self.panda_gripper_grasp_client.wait_for_server(rospy.Duration(1.0)):
            self.panda_gripper_move_client.send_goal_and_wait(req)
            self._widget.textBrowser.append("LEFT: gripper_open")
        else:
            self._widget.textBrowser.append("SERVER NOT WORKING -- LEFT: gripper_open")


    def btn_left_gripper_close_on_click(self):
        force = self._widget.spinbox_left_gripper_force.value()
        length = self._widget.spinbox_left_gripper_width.value()
        epsilon = GraspEpsilon(inner=0.027, outer=0.027)
        req = GraspGoal(width=length,
                          epsilon=epsilon,
                          speed=0.1,
                          force=force)
        if self.panda_gripper_grasp_client.wait_for_server(rospy.Duration(1.0)):
            self.panda_gripper_grasp_client.send_goal_and_wait(req)
            self._widget.textBrowser.append("LEFT: gripper_close | force: "+str(force)+" | length: "+str(length))

        else:
            self._widget.textBrowser.append("SERVER NOT WORKING -- LEFT: gripper_close")


    def btn_drill_long_run_on_click(self):
        req = JrkCmdRequest()
        req.name = 'right_long'
        req.target_value = self._widget.spinbox_drill_long_power.value()
        try:
            self.drill_proxy(req)
            self._widget.textBrowser.append("DRILL_LONG: run | power: "+str(req.target_value))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- DRILL_LONG: run")

    def btn_drill_long_stop_on_click(self):
        req = JrkCmdRequest()
        req.name = 'right_long'
        req.target_value = 0.0
        try:
            self.drill_proxy(req)
            self._widget.textBrowser.append("DRILL_LONG: stop")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- DRILL_LONG: stop")

    def btn_drill_short_run_on_click(self):
        req = JrkCmdRequest()
        req.name = 'top_long'
        req.target_value = self._widget.spinbox_drill_short_power.value()
        try:
            self.drill_proxy(req)
            self._widget.textBrowser.append("DRILL_SHORT: run | power: "+str(req.target_value))
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- DRILL_SHORT: run")

    def btn_drill_short_stop_on_click(self):
        req = JrkCmdRequest()
        req.name = 'top_long'
        req.target_value = 0.0
        try:
            self.drill_proxy(req)
            self._widget.textBrowser.append("DRILL_SHORT: stop")
        except:
            self._widget.textBrowser.append("SERVER NOT WORKING -- DRILL_SHORT: stop")

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

        if self.error_recov_client.wait_for_server(rospy.Duration(1.0)):
            self.idle_proxy(idle_control_l0)
            self.idle_proxy(idle_control_r0)
            self.idle_proxy(idle_control_t0)
            self.idle_proxy(idle_control_l2)
            self.idle_proxy(idle_control_r2)
            self.idle_proxy(idle_control_t2)
            self.error_recov_client.send_goal_and_wait(error_recovery_goal)
            self._widget.textBrowser.append("UNLOCK_ERROR")

        else:
            self._widget.textBrowser.append("SERVER NOT WORKING -- UNLOCK_ERROR")



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
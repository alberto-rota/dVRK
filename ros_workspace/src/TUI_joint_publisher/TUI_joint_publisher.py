#!/usr/bin/env python3

# Copyright (C) 2023 Alberto Rota
# 
# This file is part of suj_publisher.
# 
# suj_publisher is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# suj_publisher is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with suj_publisher.  If not, see <http://www.gnu.org/licenses/>.

# Generic
import os
import json

# Composition
from textual.app import App, ComposeResult
# Widgets
from textual.widgets import Button, Header, Footer, Static, Input, Label, Switch
# Containers
from textual.containers import Container, Vertical, Horizontal

# ROS
import rospy
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState

# Inheritances
class PSM1(Container): pass
class PSM2(Container): pass
class MTML(Container): pass
class MTMR(Container): pass

class JointValue(Static): pass
class JointName(Static): pass
class ButtonUp(Button): pass
class ButtonDown(Button): pass

class Joint(Horizontal): pass
    
        
class PSM1_dof_up(Button): pass
class PSM1_dof_down(Button): pass
class PSM1_dof(Label): pass
# 
class PSM2_dof_up(Button): pass
class PSM2_dof_down(Button): pass
class PSM2_dof(Label): pass


class MTMR_dof_up(Button): pass
class MTMR_dof_down(Button): pass
class MTMR_dof(Label): pass

class MTML_dof_up(Button): pass
class MTML_dof_down(Button): pass
class MTML_dof(Label): pass


class DOF_value(Static): pass


# APP CLASS
class TUI_joint_publisher(App):
    
    CSS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)),"TUI_joint_publisher.css")
    BINDINGS = [
        ("q", "quit_app", "Quit"), # Press Q to quit
        ("d", "toggle_dark", "Dark Mode"), # Press R to reset
        ("r", "reset", "Reset"), # Press R to reset
        ("p", "publish", "Publish "), # Press R to reset
    ]
    
    names_joints = ["YAW","PITCH","INSERTION","OUTER-ROLL","OUTER-PITCH","OUTER-YAW"]
    names_wrenches = ["FORCE-X","FORCE-Y","FORCE-Z"]
    
    joints = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0], "psm1g":False,"psm2g":False,}
    wrenches = {"mtml": [0,0,0],  "mtmr": [0,0,0],}
    
    joints_zero = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0], "psm1g":False,"psm2g":False,}
    wrenches_zero = {"mtml": [0,0,0],  "mtmr": [0,0,0],}
    
    current_jnames = {}
    current_jvals = {}
        
    def on_mount(self) -> None:
        self.reset_values()
        self.write_joint_values()
        self.publish()
        
    def reset_values(self) -> None:
        self.joints = self.joints_zero.copy()
        self.wrenches = self.wrenches_zero.copy()
        self.write_joint_values()
        
    def write_joint_values(self) -> None:
        for arm in ["psm1", "psm2"]:
            for j, joint in enumerate(self.names_joints):
                self.query_one("#"+arm+"_"+joint).update("{:.4f}".format(self.joints[arm][j]))
        for arm in ["mtml", "mtmr"]:
            for w, wrench in enumerate(self.names_wrenches):
                self.query_one("#"+arm+"_"+wrench).update("{:.4f}".format(self.wrenches[arm][w]))
        self.query_one("#psm1_gripper").update("CLOSE" if self.joints["psm1g"] else "OPEN")
        self.query_one("#psm2_gripper").update("CLOSE" if self.joints["psm2g"] else "OPEN")
                       
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        hdebug = self.query_one("#debug_console") # Handle on the debug console
        [arm,joint,direction] = bmsg = button_id.split("_")
        
        increment = float(self.query_one("#increment").value)
        if arm in ["psm1g","psm2g"]:
            if direction=="up": increment="OPEN" 
            else: increment="CLOSE"
            hdebug.update(
                f"{arm[:-1].upper()} {joint.upper()} : {increment}"
            )
        else:
            hdebug.update(
                f"{arm.upper()} {joint.upper()} {direction.upper()} : {increment}"
            )
        
        if arm in ["psm1","psm2"]:
            if direction == "up":
                self.joints[arm][self.names_joints.index(joint)] += increment
            elif direction == "down":
                self.joints[arm][self.names_joints.index(joint)] -= increment
        if arm in ["mtml","mtmr"]:
            if direction == "up":
                self.wrenches[arm][self.names_wrenches.index(joint)] += increment
            elif direction == "down":
                self.wrenches[arm][self.names_wrenches.index(joint)] -= increment
        if arm in ["psm1g","psm2g"]:
            if direction == "up":
                self.joints[arm] = True
            elif direction == "down":
                self.joints[arm] = False
                
        self.write_joint_values()
        self.publish()
            
    def publish(self) -> None:
        rospy.init_node('TUI_joint_publisher', anonymous=True)
        rate = rospy.Rate(20) 
        publishers = {
            "psm1": rospy.Publisher("/dvrk/PSM1/state_joint_current", JointState, queue_size=10),
            "psm2": rospy.Publisher("/dvrk/PSM2/state_joint_current", JointState, queue_size=10),
        }
        for arm in publishers.keys():
            joints = JointState()
            joints.name = ['outer_yaw','outer_pitch','outer_insertion','outer_roll','outer_wrist_pitch','outer_wrist_yaw']
            joints.position = self.joints[arm]
            publishers[arm].publish(joints)
            rate.sleep()
                    
        publishers = {
            "mtml": rospy.Publisher("/dvrk/MTML/set_wrench_body_safe", Wrench, queue_size=10),
            "mtmr": rospy.Publisher("/dvrk/MTMR/set_wrench_body_safe", Wrench, queue_size=10),
        }
        for arm in publishers.keys():
            wrench = Wrench()
            wrench.force.x = self.wrenches[arm][0]
            wrench.force.y = self.wrenches[arm][1]
            wrench.force.z = self.wrenches[arm][2]
            publishers[arm].publish(wrench)
            rate.sleep()
        publishers = {
            "psm1g": rospy.Publisher("/dvrk/PSM1/state_jaw_current", JointState, queue_size=10),
            "psm2g": rospy.Publisher("/dvrk/PSM2/state_jaw_current", JointState, queue_size=10),
        }
        for arm in publishers.keys():
            jaw = JointState()
            jaw.name = ['jaw']
            if self.joints[arm]: jaw.position = [-2] 
            else: jaw.position = [1]   
            publishers[arm].publish(jaw)
            
    # App Composition
    def compose(self) -> ComposeResult:
        yield Header()
        yield Footer()
        
        with Horizontal(id="rightleft"):
            with Vertical(id="left"):
                yield Static("PSM1",id="psm1_name")
                with PSM1():
                    for n in self.names_joints:
                        yield JointName(n)
                        yield JointValue("{:.4f}".format(0.0),id="psm1_"+n)
                        yield ButtonUp("UP", variant="success", id="psm1_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="psm1_"+n+"_down")
                    yield JointName("Gripper")
                    yield JointValue("OPEN",id="psm1_gripper")                        
                    yield ButtonUp("OPEN", variant="default", id="psm1g_gripper_down")
                    yield ButtonDown("CLOSE", variant="primary", id="psm1g_gripper_up")
                        
                yield Static("MTML",id="mtml_name")
                with MTML():
                    for n in self.names_wrenches:
                        yield JointName(n)
                        yield JointValue("{:.4f}".format(0.0),id="mtml_"+n)
                        yield ButtonUp("UP", variant="success", id="mtml_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="mtml_"+n+"_down")
                        
            with Vertical(id="right"):
                yield Static("PSM2",id="psm2_name")
                with PSM2():
                    for n in self.names_joints:
                        yield JointName(n)
                        yield JointValue("{:.4f}".format(0.0),id="psm2_"+n)
                        yield ButtonUp("UP", variant="success", id="psm2_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="psm2_"+n+"_down")
                    yield JointName("Gripper")
                    yield JointValue("OPEN",id="psm2_gripper")                        
                    yield ButtonUp("OPEN", variant="default", id="psm2g_gripper_down")
                    yield ButtonDown("CLOSE", variant="primary", id="psm2g_gripper_up")

                yield Static("MTMR",id="mtmr_name")
                with MTMR():
                    for n in self.names_wrenches:
                        yield JointName(n)
                        yield JointValue("{:.4f}".format(0.0),id="mtmr_"+n)
                        yield ButtonUp("UP", variant="success", id="mtmr_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="mtmr_"+n+"_down")
        with Container(id="bottombar"):
            yield Static("INCREMENT = ", id="increment_label")
            yield Input("0.1", id="increment")
            yield Static("",id="debug_console")
        # self.write_joint_values()
    
    # ACTIONS
    def action_quit_app(self) -> None:
        app.exit()
        
    def action_toggle_dark(self) -> None:
        self.dark = not self.dark
    
    def action_reset(self) -> None: 
        self.reset_values()

    def action_publish(self) -> None:
        self.publish()

    def action_debug(self) -> None:
        pass

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = TUI_joint_publisher()
    app.run()
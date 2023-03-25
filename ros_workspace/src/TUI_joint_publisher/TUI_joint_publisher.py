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
from textual.widgets import Button, Header, Footer, Static, Input, Label
# Containers
from textual.containers import Container, Vertical, Horizontal

# ROS
# import rospy
# from sensor_msgs.msg import JointState

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
    ]
    
    names_joints = ["YAW","PITCH","INSERTION","OUTER-ROLL","OUTER-PITCH","OUTER-YAW"]
    names_wrenches = ["FORCE-X","FORCE-Y","FORCE-Z"]
    
    joints = {"psm1": [1,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
    wrenches = {"mtmr": [0,0,0], "mtml": [0,2,0]}
    
    current_jnames = {}
    current_jvals = {}

    def initialize_values(self):
        joints = self.query("PSM1_dof")
        for j in range(6):
            joints.nodes[j].value = self.names_joints[j]+" - "+str(self.joints["psm1"][j])
        joints = self.query("PSM2_dof")
        for j in range(6):
            joints.nodes[j].value = self.names_joints[j]+" - "+str(self.joints["psm2"][j])
        wrenches = self.query("MTML_dof")
        for j in range(3):
            wrenches.nodes[j].value = self.names_wrenches[j]+" - "+str(self.wrenches["mtml"][j])
        wrenches = self.query("MTMR_dof")
        for j in range(3):
            wrenches.nodes[j].value = self.names_wrenches[j]+" - "+str(self.wrenches["mtmr"][j])
        
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        bhandle = self.query_one("#"+button_id)
        jname, jvalue_str = str(bhandle.label).split("  -  ")
        increment = self.query_one("#increment").value
        bhandle.label = jname+"  -  "+str(float(jvalue_str)+0.1)
        
    def read_joints(self, arm: str) -> None:
        pass
        
    #     self.current_jnames = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
    #     self.current_jvals = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
        
    #     jnames = self.query("JointName")
    #     jvals = self.query("JointValue")
    #     for a,armname in enumerate(["psm1", "ecm", "psm2"]):
    #         for j in range(6):
    #             self.current_jnames[armname][j] = jnames.nodes[a*6+j].value
    #             self.current_jvals[armname][j] = float(jvals.nodes[a*6+j].value)        
    #     return {"names": self.current_jnames[arm], "values": self.current_jvals[arm]}
        
    # def publish_sujs(self) -> None:
    #     rospy.init_node('SUJ_publisher_console_node', anonymous=True)
    #     publishers = {
    #         "psm1": rospy.Publisher("/dvrk/SUJ/PSM1/state_joint_current", JointState, queue_size=10),
    #         "ecm": rospy.Publisher("/dvrk/SUJ/ECM/state_joint_current", JointState, queue_size=10),
    #         "psm2": rospy.Publisher("/dvrk/SUJ/PSM2/state_joint_current", JointState, queue_size=10),
    #     }
    #     rate = rospy.Rate(20) 
    #     for arm in publishers.keys():
    #         current_joints = self.read_joints(arm)
    #         joints = JointState()
    #         joints.name = current_joints["names"]
    #         joints.position = current_joints["values"]
    #         publishers[arm].publish(joints)
    #         rate.sleep()

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
                        yield JointValue(str(0.0),id="psm1_"+n)
                        yield ButtonUp("UP", variant="success", id="psm1_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="psm1_"+n+"_down")
                        
                yield Static("MTML",id="mtml_name")
                with MTML():
                    for n in self.names_wrenches:
                        yield JointName(n)
                        yield JointValue(str(0.0),id="mtml"+n)
                        yield ButtonUp("UP", variant="success", id="mtml_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="mtml_"+n+"_down")
                        
            with Vertical(id="right"):
                yield Static("PSM2",id="psm2_name")
                with PSM2():
                    for n in self.names_joints:
                        yield JointName(n)
                        yield JointValue(str(0.0),id="psm2_"+n)
                        yield ButtonUp("UP", variant="success", id="psm2_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="psm2_"+n+"_down")

                yield Static("MTMR",id="mtmr_name")
                with MTMR():
                    for n in self.names_wrenches:
                        yield JointName(n)
                        yield JointValue(str(0.0),id="mtmr"+n)
                        yield ButtonUp("UP", variant="success", id="mtmr_"+n+"_up")
                        yield ButtonDown("DOWN", variant="error", id="mtmr_"+n+"_down")
        with Container(id="bottombar"):
            yield Static("INCREMENT = ", id="increment_label")
            yield Input("0.1", id="increment")
            yield Static("",id="debug_console")
        
    # EVENT HANDLERS AND ACTION LISTENERS
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        bhandle = self.query_one("#"+button_id)
        hdebug = self.query_one("#debug_console") # Handle on the debug console
        bmsg = button_id.split("_")
        increment = float(self.query_one("#increment").value)
        hdebug.update(
            f"{bmsg[0].upper()} {bmsg[1].upper()} {bmsg[2].upper()} : {increment}"
        )
        # jname, jvalue_str = str(bhandle.label).split("  -  ")
        # bhandle.label = jname+"  -  "+str(float(jvalue_str)+0.1)
    
    # ACTIONS
    
    def action_quit_app(self) -> None:
        app.exit()
        
    def action_toggle_dark(self) -> None:
        self.dark = not self.dark
    
    def action_reset(self) -> None: 
        self.initialize_values()
        # self.reset()

    def action_reset_suj(self) -> None:  
        self.reload_defaults()

    def action_debug(self) -> None:
        print("IN DEBUG MODE")
        pass

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = TUI_joint_publisher()
    app.run()
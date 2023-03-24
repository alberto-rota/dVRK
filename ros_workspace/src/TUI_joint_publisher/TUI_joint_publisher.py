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

class PSM1_dof_up(Button): pass
class PSM1_dof_down(Button): pass
class PSM1_dof(Static): pass
# 
class PSM2_dof_up(Button): pass
class PSM2_dof_down(Button): pass
class PSM2_dof(Static): pass

class MTML(Container): pass
class MTMR(Container): pass

class MTMR_dof_up(Button): pass
class MTMR_dof_down(Button): pass
class MTMR_dof(Static): pass

class MTML_dof_up(Button): pass
class MTML_dof_down(Button): pass
class MTML_dof(Static): pass


class DOF_value(Static): pass


# APP CLASS
class TUI_joint_publisher(App):
    
    CSS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)),"TUI_joint_publisher.css")
    BINDINGS = [
        ("q", "quit_app", "Quit"), # Press Q to quit
        ("d", "toggle_dark", "Dark Mode"), # Press R to reset
        ("r", "reset", "Reset"), # Press R to reset
    ]
    
    names_joints = ["YAW","PITCH","INSERTION","OUTER_ROLL","OUTER_PITCH","OUTER_YAW"]
    names_wrenches = ["FORCE_X","FORCE_Y","FORCE_Z"]
    
    joints = {"psm1": [1,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
    wrenches = {"mtmr": [0,0,0], "mtml": [0,2,0]}
    
    current_jnames = {}
    current_jvals = {}

    def initialize_values(self):
        for arm in ["psm1", "psm2"]:
            for j,jname in enumerate(self.names_joints):
                self.query_one("#"+arm+"_"+jname).label = jname+"  -  "+str(self.joints[arm][j])
        for arm in ["mtml", "mtmr"]:
            for j,jname in enumerate(self.names_wrenches):
                self.query_one("#"+arm+"_"+jname).label = jname+"  -  "+str(self.wrenches[arm][j])
        
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        bhandle = self.query_one("#"+button_id)
        jname, jvalue_str = str(bhandle.label).split("  -  ")
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
                        yield PSM1_dof(str(0.00))
                        yield PSM1_dof_up("+") #n+"  -  "+str(0.00), id="psm1_"+n)
                        yield PSM1_dof_down("-") #n+"  -  "+str(0.00), id="psm1_"+n)
                        
                yield Static("MTML",id="mtml_name")
                with MTML():
                    for n in self.names_wrenches:
                        yield MTML_dof(str(0.0))
                        yield MTML_dof_up("+") #n+"  -  "+str(0.00), id="mtml_"+n)
                        yield MTML_dof_down("-") #n+"  -  "+str(0.00), id="mtml_"+n)
                        
            with Vertical(id="right"):
                yield Static("PSM2",id="psm2_name")
                with PSM2():
                    for n in self.names_joints:
                        yield PSM2_dof(str(0.0))
                        yield PSM2_dof_up("+") #n+"  -  "+str(0.00), id="psm2_"+n)
                        yield PSM2_dof_down("-") #n+"  -  "+str(0.00), id="psm2_"+n)
                yield Static("MTMR",id="mtmr_name")
                with MTMR():
                    for n in self.names_wrenches:
                        yield MTMR_dof(str(0.0))
                        yield MTMR_dof_up("+") #n+"  -  "+str(0.00), id="mtmr_"+n)
                        yield MTMR_dof_down("-") #n+"  -  "+str(0.00), id="mtmr_"+n)
 
        
        
    # EVENT HANDLERS AND ACTION LISTENERS
        
    
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
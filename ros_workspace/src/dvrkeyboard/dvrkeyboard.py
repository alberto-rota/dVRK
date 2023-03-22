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

# # ROS
# import rospy
# from sensor_msgs.msg import JointState

# Inheritances
class PSM1(Container): pass
class PSM2(Container): pass
class PSM1_dof(Button): pass
class PSM2_dof(Button): pass

class MTML(Container): pass
class MTMR(Container): pass
class MTMR_dof(Button): pass
class MTML_dof(Button): pass

# APP CLASS
class DVRKeyboard(App):
    
    CSS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)),"dvrkeyboard.css")
    BINDINGS = [
        ("q", "quit_app", "Quit"), # Press Q to quit
        ("p", "publish_suj", "Publish SUJs"), # Press P to publish
        ("r", "reset_suj", "Reset to Default"), # Press R to reset
        ("d", "toggle_dark", "Dark Mode"), # Press R to reset
        ("x", "debug", ""), # Press R to reset
    ]
    DEFAULTS_PATH = "defaults.json"
    # Will contain the names and values upon publishing
    
    current_jnames = {}
    current_jvals = {}

    def read_joints(self, arm: str) -> None:
        
        self.current_jnames = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
        self.current_jvals = {"psm1": [0,0,0,0,0,0],"ecm": [0,0,0,0,0,0],"psm2": [0,0,0,0,0,0],}
        
        jnames = self.query("JointName")
        jvals = self.query("JointValue")
        for a,armname in enumerate(["psm1", "ecm", "psm2"]):
            for j in range(6):
                self.current_jnames[armname][j] = jnames.nodes[a*6+j].value
                self.current_jvals[armname][j] = float(jvals.nodes[a*6+j].value)        
        return {"names": self.current_jnames[arm], "values": self.current_jvals[arm]}
        
    def publish_sujs(self) -> None:
        rospy.init_node('SUJ_publisher_console_node', anonymous=True)
        publishers = {
            "psm1": rospy.Publisher("/dvrk/SUJ/PSM1/state_joint_current", JointState, queue_size=10),
            "ecm": rospy.Publisher("/dvrk/SUJ/ECM/state_joint_current", JointState, queue_size=10),
            "psm2": rospy.Publisher("/dvrk/SUJ/PSM2/state_joint_current", JointState, queue_size=10),
        }
        rate = rospy.Rate(20) 
        for arm in publishers.keys():
            current_joints = self.read_joints(arm)
            joints = JointState()
            joints.name = current_joints["names"]
            joints.position = current_joints["values"]
            publishers[arm].publish(joints)
            rate.sleep()
        
    # App Composition
    def compose(self) -> ComposeResult:
        yield Header()
        yield Footer()
        
        with Horizontal(id="rightleft"):
            with Vertical(id="left"):
                yield Static("PSM1",id="psm1_name")
                yield PSM1(
                    PSM1_dof("YAW"),
                    PSM1_dof("PITCH"),
                    PSM1_dof("INSERTION"),
                    PSM1_dof("OUTER_ROLL"),
                    PSM1_dof("OUTER_PITCH"),
                    PSM1_dof("OUTER_YAW"),
                )
                yield Static("MTML",id="mtml_name")
                yield MTML(
                    MTML_dof("FORCE X"),   
                    MTML_dof("FORCE Y"),   
                    MTML_dof("FORCE X"),   
                )
            with Vertical(id="right"):
                yield Static("PSM2",id="psm2_name")
                yield PSM2(
                    PSM2_dof("YAW"),
                    PSM2_dof("PITCH"),
                    PSM2_dof("INSERTION"),
                    PSM2_dof("OUTER_ROLL"),
                    PSM2_dof("OUTER_PITCH"),
                    PSM2_dof("OUTER_YAW"),
                )
                yield Static("MTMR",id="mtmr_name")
                yield MTMR(
                    MTMR_dof("FORCE X"),   
                    MTMR_dof("FORCE Y"),   
                    MTMR_dof("FORCE X"),   
                )
 
    # EVENT HANDLERS AND ACTION LISTENERS
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        if button_id == "reset_button":
            self.reload_defaults()
        elif button_id == "publish_button":
            self.publish_sujs()
            
    
    # ACTIONS
    def action_quit_app(self) -> None:
        app.exit()
        
    def action_toggle_dark(self) -> None:
        self.dark = not self.dark
    
    def action_publish_suj(self) -> None: 
        self.publish_sujs()

    def action_reset_suj(self) -> None:  
        self.reload_defaults()

    def action_debug(self) -> None:
        print("IN DEBUG MODE")
        pass

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = DVRKeyboard()
    app.run()
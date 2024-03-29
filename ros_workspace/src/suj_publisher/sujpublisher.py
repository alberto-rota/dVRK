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
import json
import os

# Composition
from textual.app import App, ComposeResult
# Widgets
from textual.widgets import Button, Header, Footer, Static, Input, Label
# Containers
from textual.containers import Container, Vertical, Horizontal

# ROS
import rospy
import rostopic
from sensor_msgs.msg import JointState

# Inheritances
class JointCode(Static): pass
class Empty(Static): pass
class Entry(Static): pass
class Entries(Horizontal): pass
class JointName(Input): pass
class JointValue(Input): pass
class Joint(Horizontal): pass

# APP CLASS
class SUJPublisher(App):
    
    CSS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)),"sujpublisher.css")
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
    
    def check_rosmaster(self) -> bool:
        ROS_MASTER_URI = os.environ['ROS_MASTER_URI']
        ROS_IP = os.environ['ROS_IP']
        # Checkif rosmaster is running or not.
        try:
            rostopic.get_topic_class('/rosout')
            master = True
        except: master = False

        if not master:
            print("[red]ROSMaster not found! ")
            print("[red]-----------------------------------------------------------------------------------------")
            print("[red]Check that the ROSMaster is running. If it is, reconfigure your environmet variables")
            print("[red]> ROS_MASTER_URI = ",ROS_MASTER_URI, "[red]---> Must be the IP address of the computer running ROSMaster")
            print("[red]> ROS_IP = ",ROS_IP, "[red]---> Must be your IP address")
            print("[red]-----------------------------------------------------------------------------------------")
            print("[red]Reconfigure this variables with `export ROS_MASTER_URI=http://<MASTER_IP>:11311` and `export ROS_IP=<IP>`\n")
        return master
    
    def load_json(self, jsonpath: str) -> dict:
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)),jsonpath)) as f:
            self.defaults = json.load(f)
# 
    def reload_defaults(self) -> None: 
        self.load_json(self.DEFAULTS_PATH)
        jnames = self.query("JointName")
        jvals = self.query("JointValue")
        for a,armname in enumerate(["psm1", "ecm", "psm2"]):
            for j in range(6):
                jnames.nodes[a*6+j].value = str(self.defaults[armname.upper()]["names"][j])
                jvals.nodes[a*6+j].value = str(self.defaults[armname.upper()]["values"][j])

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
        self.load_json(self.DEFAULTS_PATH)
        yield Header()
        yield Footer()
        
        # Devide the screen in three columns, one for each arm
        with Horizontal(id="threearms"):
            for arm in ["psm1", "ecm", "psm2"]:
                
                # Arms are organized vertically
                with Vertical(id=arm):
                    
                    # Arm name, displayed with its color
                    yield Static(arm.upper(), id=f"{arm}_armname")
                                        
                    # Each arm is a set of 7 joints    
                    for j in range(0,6):
                        with Joint():
                            # Each joint has an index, name and value
                            yield JointCode(f"SUJ {j}")
                            yield JointName(str(self.defaults[arm.upper()]["names"][j]))
                            yield JointValue(str(self.defaults[arm.upper()]["values"][j]))
                            
        # Two Buttons at the bottom, One to reset to default, one to publish
        with Horizontal(id="buttons"):
            yield Button("Reset to Default", variant="warning", id="reset_button")
            yield Button("Publish SUJs", variant="success", id="publish_button")
            # yield Button("Quit", id="quit")
 
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
    app = SUJPublisher()
    if app.check_rosmaster(): app.run()
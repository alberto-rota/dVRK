# Copyright (C) 2023 Alberto Rota
# 
# This file is part of dVRK.
# 
# dVRK is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# dVRK is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with dVRK.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import rostopic
import os
from rich import print

def main(): 
    ROS_MASTER_URI = os.environ['ROS_MASTER_URI']
    ROS_IP = os.environ['ROS_IP']
    try:
        rostopic.get_topic_class('/rosout')
        master = True
    except: master = False
    print()
    if not master:
        print("[red bold]ROSMaster not found! ")
        print("[red]-----------------------------------------------------------------------------------------")
        print("[red]Check that the ROSMaster is running. If it is, reconfigure your environmet variables")
        print("[red]> ROS_MASTER_URI = ",ROS_MASTER_URI, "[red]---> Must be the IP address of the computer running ROSMaster")
        print("[red]> ROS_IP = ",ROS_IP, "[red]---> Must be your IP address")
        print("[red]-----------------------------------------------------------------------------------------")
        print("[red]Reconfigure this variables with `export ROS_MASTER_URI=http://<MASTER_IP>:11311` and `export ROS_IP=<IP>`\n")
    else:
        print("[green bold]ROSMaster found and connected! ")
        print("[green]-----------------------------------------------------------------------------------------")
        print("[green]> ROS_MASTER_URI = ",ROS_MASTER_URI)
        print("[green]> ROS_IP = ",ROS_IP)
        print("[green]-----------------------------------------------------------------------------------------")
    
if __name__ == "__main__":
    main()
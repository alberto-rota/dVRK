# ROS Utils ROSPackage 
*This package was made and is maintained by [Alberto Rota](https://nearlab.polimi.it/medical/alberto-rota/). Contributes, issues and corrections are welcome at [Alberto's email](mailto:alberto1.rota@polimi.it).*

***
You can download this ROSPackage from
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/ros_utils">this DownGit link</a>:   

<p align="left"> 
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/ros_utils">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../../../readme/download_button_white.png">
  <img style="vertical-align:middle" alt="NEARLab" src="../../../readme/download_button_dark.png" width="200" > 
</picture>
</a> </p>

***

The ROS utils ROSPackage contains scripts and launch files useful when working with a ROS framework.
### Installation
1. Copy the folder `ros_utils` in the `src` folder of your ROS workspace.
2. Inside your workspace folder, run `catkin_make` to build the package.

### Launching the nodes
1. Source the workspace with `source devel/setup.bash` (alternatively, you can add this line to your `.bashrc` file, but you will have to extend the path and make it complete and non-relative).
2. Run `rosrun ros_utils check.py` to check the connection to the network and the presence of the rosmaster

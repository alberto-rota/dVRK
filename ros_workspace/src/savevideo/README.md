# savevideo ROSPackage 
*This package was made and is maintained by [Alberto Rota](https://nearlab.polimi.it/medical/alberto-rota/). Contributes, issues and corrections are welcome at [Alberto's email](mailto:alberto1.rota@polimi.it).*

***
You can download this ROSPackage from
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/steve">this DownGit link</a>:   

<p align="left"> 
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/steve">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../../../readme/download_button_white.png">
  <img style="vertical-align:middle" alt="NEARLab" src="../../../readme/download_button_dark.png" width="200" > 
</picture>
</a> </p>

***

The STEVE ROSPackage contains code for:
- Activating gravity compensation on the MTMs
- Activating virtual safety relays on too high wrenches applied at the MTMs. The robot will clip the output forces at 3N and the output torques at 3mN/m, and unpower if the requested forces exceed 5N or 5mN/m.
- Activating virtual safety relays on too high velocities of the surgical tooltips at the MTMs. The robot will unpower at too high velocities.
- Setting the initial teleoperation scale factor to 0.5

### Installation
1. Copy the folder `steve` in the `src` folder of your ROS workspace.
2. Inside your workspace folder, run `catkin_make` to build the package.

### Launching the package
1. Source the workspace with `source devel/setup.bash` (alternatively, you can add this line to your `.bashrc` file, but you will have to extend the path and make it complete and non-relative).
2. Run `roslaunch steve steve.launch` and check for the correct output in the terminal, which should look something like this (lines might appear in a different order):
  
<span style="color:#cccccc"> > Gravity compensation is ACTIVE on Right-MTM </span> <br>
<span style="color:#cccccc"> > Gravity compensation is ACTIVE on Left-MTM </span> <br>
<span style="color:#cccccc"> > Teleoperation scale initialized to 0.5 </span> <br>
<span style="color:#d78700"> > Left-MTM RF set to BASE </span> <br>
<span style="color:#d78700"> > Right-MTM RF set to BASE </span> <br>
<span style="color:#00d25d"> > Wrench Safety buffer initialized </span> <br>
<span style="color:#00d25d"> > Joint Velocity Safety Virtual Relay Initialized </span> <br>
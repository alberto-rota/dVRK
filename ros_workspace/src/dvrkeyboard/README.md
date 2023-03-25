# dVRKeyboard ROSPackage 
*This package was made and is maintained by [Alberto Rota](https://nearlab.polimi.it/medical/alberto-rota/). Contributes, issues and corrections are welcome at [Alberto's email](mailto:alberto1.rota@polimi.it).*

***
You can download this ROSPackage from
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/dvrkeyboard">this DownGit link</a>:   

<p align="left"> 
<a href="https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/NEARLab-MedicalRobotics/dVRK/tree/main/ros_workspace/src/dvrkeyboard">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../../../readme/download_button_white.png">
  <img style="vertical-align:middle" alt="NEARLab" src="../../../readme/download_button_dark.png" width="200" > 
</picture>
</a> </p>

***

The STEVE ROSPackage allows to send joint and wrench messages on a ROS network by pressing keys on the keyboard. It is thought to be integrated with a *dVRK* and network. The package is a TUI (Textual-User-Interface) based on [Textual](https://textual.textualize.io/) thought to simplify the process of testing and troubleshooting research projects on the dVRK from a simple and intuitive graphical inteface.

### Installation
1. Copy the folder `dvrkeyboard` in the `src` folder of your ROS workspace.
2. Inside your workspace folder, run `catkin_make` to build the package.

### Launching the package
1. Source the workspace with `source devel/setup.bash` (alternatively, you can add this line to your `.bashrc` file, but you will have to extend the path and make it complete and non-relative).
2. Run `rosrun dvrkeyboard keyboard_teleop_PSM1.py` if you want to communicate with PSM1
3. Run `rosrun dvrkeyboard keyboard_teleop_PSM2.py` if you want to communicate with PSM2
4. Run `rosrun dvrkeyboard keyboard_wrench_MTMR.py` if you want to communicate with MTMR
5. Run `rosrun dvrkeyboard keyboard_wrench_MTML.py` if you want to communicate with MTML
   
The terminal will idle and wait for your key presses

To exit, keep `Ctrl-C` pressed for a few seconds

# mocap-toolkit

## Installation:
1. Set up catkin workspace
2. Clone this repo into `src/` of workspace
3. `cd mocap_toolkit/`
4. Clone all repos using vcstool: `vcs import . < mocap_toolkit.repos`
5. Install dependencies: `rosdep install -iyr --from-paths src` from the workspace directory.
6. Build with `catkin build mocap_toolkit`, this will build all necessary packages.
7. Source packages `source [WORKSPACe]/devel/setup.[SHELL]`

## Hand eye calibration:
1. Start the mocap system and make sure Motive is broadcasting.
2. Start your camera node
3. Set up the April tag board in the mocap room. We are using the side with the smaller tags. (You need to change `apriltag_ros/apriltag_ros/config/tags.yaml` if using other tag bundles.)
4. `roslaunch mocap_toolkit calibrate.launch` This will launch the apriltag detector, mocap, and the easy_handeye calibration tool.
5. Use the GUI to perform hand eye calibration
6. Save the calibration using the GUI

## Use during experiment:
1. Start the mocap system
2. `roslaunch mocap_toolkit broadcast.launch` This will start the mocap node and broadcast the previously saved hand-eye calibration.

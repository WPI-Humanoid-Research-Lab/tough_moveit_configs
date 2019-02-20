# tough_moveit_configs
Moveit configurations packages for robots supported by [T.O.U.G.H.](https://github.com/WPI-Humanoid-Research-Lab/tough)

* This repository contains configuration packages for **Atlas** and **Vakyrie** robots using MoveIt!


## Atlas
### Move-groups: 
| Move-groups   | Joints        | Chain           |
| ------------- |:-------------:| :--------------:|
| L_PELVIS_PALM_10DOF     | world_pelvis_virtual_joint, back_bkz, back_bky, back_bkx, l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2, l_robotiq_hand_joint, l_robotiq_end_eff_virtual_joint| pelvis -> l_end_eff|
| R_PELVIS_PALM_10DOF     | world_pelvis_virtual_joint back_bkz, back_bky, back_bkx, r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx, r_arm_wry2, r_robotiq_hand_joint, l_robotiq_end_eff_virtual_joint| pelvis -> r_end_eff|
| L_SHOULDER_HAND_7DOF    | l_arm_shz, ..., l_robotiq_end_eff_virtual_joint | l_clav -> l_end_eff|
| R_SHOULDER_HAND_7DOF    | r_arm_shz, ..., l_robotiq_end_eff_virtual_joint | r_clav -> r_end_eff|


___
## Valkyrie
### Move-groups:*
| Move-groups   | Joints        | Chain           |
| ------------- |:-------------:| :--------------:|
| L_PELVIS_PALM_10DOF | torsoYaw, torsoPitch, torsoRoll, leftShoulderPitch, leftShoulderRoll, leftShoulderYaw, leftElbowPitch, leftForearmYaw, leftWristRoll, leftWristPitch, leftMiddleFingerPitch1 | pelvis -> leftMiddleFingerPitch1Link  |
| R_PELVIS_PALM_10DOF | torsoYaw, torsoPitch, torsoRoll, rightShoulderPitch, rightShoulderRoll, rightShoulderYaw, rightElbowPitch, rightForearmYaw, rightWristRoll, rightWristPitch, rightMiddleFingerPitch1 | pelvis -> rightMiddleFingerPitch1Link  |
| L_SHOULDER_HAND_7DOF                | leftShoulderPitch, ..., leftMiddleFingerPitch1 | leftShoulderPitchLink -> leftMiddleFingerPitch1Link |
| R_SHOULDER_HAND_7DOF               | rightShoulderPitch, ..., rightMiddleFingerPitch1 | rightShoulderPitchLink -> rightMiddleFingerPitch1Link |

_*Development in progress_
### Run MoveIt!
For a standalone execution of these packages use following commands in your terminal:
#### Atlas
This command will launch the move_group planners and will also launch RViz for the visualization of the generated trajectory.
``` 
roslaunch atlas_moveit_config atlas_moveit_planner_rviz.launch
```
If you dont want the rviz to get launched, use following command
```
roslaunch atlas_moveit_config atlas_moveit_planner.launch
```
#### Valkyrie
```
roslaunch valkyrie_moveit_config demo.launch 
```
After launching the moveit_planners of the robot (atlas or valkyrie), you can start planning with the help of either T.O.U.G.H. or MoveIt!

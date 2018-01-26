# tough_moveit_configs
Moveit configurations packages for robots supported by [T.O.U.G.H.](https://github.com/WPI-Humanoid-Research-Lab/tough)

* This repository contains configuration packages for **Atlas** and **Vakyrie** robots using MoveIt!


## Atlas
### Move-groups: 
| Move-groups   | Joints        | Chain           |
| ------------- |:-------------:| :--------------:|
| leftMiddleFingerGroup     | back_bkz, back_bky, back_bkx, l_arm_shz, l_arm_shx, l_arm_ely, l_arm_elx, l_arm_wry, l_arm_wrx, l_arm_wry2, l_robotiq_hand_joint| pelvis -> l_palm|
| rightMiddleFingerGroup    | back_bkz, back_bky, back_bkx, r_arm_shz, r_arm_shx, r_arm_ely, r_arm_elx, r_arm_wry, r_arm_wrx, r_arm_wry2, r_robotiq_hand_joint| pelvis -> r_palm|
|  leftArm      |     -          | l_clav -> l_hand|
| rightArm      |     -          | r_clav -> r_hand|
| rightPalm     |     -          | pelvis -> r_hand|
| leftPalm      |     -          | pelvis -> l_hand|
___
## Valkyrie
### Move-groups:
| Move-groups   | Joints        | Chain           |
| ------------- |:-------------:| :--------------:|
| rightMiddleFingerGroup | torsoYaw, torsoPitch, torsoRoll, rightShoulderPitch, rightShoulderRoll, rightShoulderYaw, rightElbowPitch, rightForearmYaw, rightWristRoll, rightWristPitch, rightMiddleFingerPitch1 | pelvis -> rightMiddleFingerPitch1Link  |
| leftMiddleFingerGroup  | torsoYaw, torsoPitch, torsoRoll, leftShoulderPitch, leftShoulderRoll, leftShoulderYaw, leftElbowPitch, leftForearmYaw, leftWristRoll, leftWristPitch, leftMiddleFingerPitch1 | pelvis -> leftMiddleFingerPitch1Link  |
| leftArm                | leftShoulderPitch, leftShoulderRoll, leftShoulderYaw, leftElbowPitch, leftForearmYaw, leftWristRoll, leftWristPitch | leftShoulderPitchLink -> leftPalm |
| rightArm               | rightShoulderPitch, rightShoulderRoll, rightShoulderYaw, rightElbowPitch, rightForearmYaw, rightWristRoll, rightWristPitch  | rightShoulderPitchLink -> rightPalm |
| rightPalm              | -  | pelvis -> rightPalm |
| leftPalm               | -  | pelvis -> leftPalm  |

# *Shape and alignment control of an elastic beam*

The idea of this project is to make available the implementation of the above mentioned framework, so that it can be used, modified and improved. 

The reference benchmark is represented by the equipment in LS2N, throughout the guide the convention adopted is that the left and right is opposite with respect to the viewer's viewpoint.

## Preliminary operations

- Switch right and left robots on by turning on the corresponding switches of the lower and upper KUKA PCs in the case close to the workstation;

- during bootstrap it may be required to press F1 on the keyboard that can be connected to each box;

- at the end of bootstrap the robot must be sent to their **home position*, it not already there;

- the robots must be brought to the **ft6 position**, to start working in automatic mode.

## Home and FT6 position

- Select **Manual mode**, by turning mode selection key to the *T with 3 arrows* icon;

- grab the panel with the left hand and press the white button also known as **dead man**;

- while keeping the dead man pressed, keep the green button pressed until the *home* position is reached

- repeat for *ft6*, in this case release and press again the green button.

## Control manager and planner

Here we launch some nodes in order to transmit the received commands to the robot (actually to the FRI), together with RVIZ visualize the state of the robot and the planning stack to
translate cartesian commands into joint ones:
- roslaunch deformation_control model.launch
- roslaunch deformation_control planning.launch
- rqt to open the control manager for each robot
In order to start the two control managers, in rqt repeat twice Robot Tools, Control manager,
at this point in one window select the topic left, while in the other right.

## Grasping

As already discussed in multiple parts of this work the grasping part require the intervention
of the operator within the setup phase, hereafter we show the instruction to move the grippers:
- cd Documents/EGN100/egn100s;
- ./testEGN100;
- enter value in mm;
- insert the beam into the grippers;
- roslaunch schunk sdh_only.launch;


## Calibration phase
The output of this part is a text file containing the 4 X M elements of the sensitivity jacobian
matrix:
roslaunch tuw_aruco_marker marker_live.launch;
roslaunch deformation_control marker_detector.launch;
roslaunch deformation_control calibration.launch;
roslaunch deformation_control jacobian_computation.launch;
roslaunch deformation_control joint_position_publisher.launch.

## Positioning and shaping implementation
Here we launch all the nodes required to implement the pipeline presented:
- roslaunch tuw_aruco_marker marker_live.launch;
- roslaunch deformation_control marker_detector.launch;
- roslaunch deformation_control defomation_computation.launch;
- roslaunch deformation_control ee_position_ctrl.launch;
- roslaunch deformation_control joint_position_publisher.launch;
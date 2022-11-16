#!/bin/zsh

export CONTROLLER_NAME=inverse_dynamics_test_controller
export CONTROLLER_NAME_CAPS=InverseDynamicsTestController
export TEMPLATE=joint_pd_test_pin_controller
export TEMPLATE_CAPS=JointPDTestPinController

export NEW_CONTROLLER_FILENAME=franka_example_controllers/src/$CONTROLLER_NAME.cpp
export TEMPLATE_FILENAME=franka_example_controllers/src/$TEMPLATE.cpp

export NEW_CONTROLLER_HEADERFILE=franka_example_controllers/include/franka_example_controllers/$CONTROLLER_NAME.h
export TEMPLATE_HEADERFILE=franka_example_controllers/include/franka_example_controllers/$TEMPLATE.h

export NEW_CONTROLLER_LAUNCHFILE=franka_example_controllers/launch/$CONTROLLER_NAME.launch
export TEMPLATE_LAUNCHFILE=franka_example_controllers/launch/$TEMPLATE.launch

touch $NEW_CONTROLLER_FILENAME
echo "Created CPP file $NEW_CONTROLLER_FILENAME"

cat $TEMPLATE_FILENAME > $NEW_CONTROLLER_FILENAME
echo "Copied content of $TEMPLATE to $NEW_CONTROLLER_FILENAME, remember to change header and function names"

touch $NEW_CONTROLLER_HEADERFILE
echo "Created header file $NEW_CONTROLLER_HEADERFILE"

cat $TEMPLATE_HEADERFILE > $NEW_CONTROLLER_HEADERFILE
echo "Copied content of $TEMPLATE_HEADERFILE to $NEW_CONTROLLER_HEADERFILE, remember to change class names"

touch $NEW_CONTROLLER_LAUNCHFILE
echo "Created header file $NEW_CONTROLLER_HEADERFILE"

cat $TEMPLATE_LAUNCHFILE > $NEW_CONTROLLER_LAUNCHFILE
echo "Copied content of $TEMPLATE_LAUNCHFILE to $NEW_CONTROLLER_LAUNCHFILE, remember to change controller name"

echo "Remember to change:"
echo " - franka_example_controllers/config/franka_example_controller.yaml"
echo " - franka_example_controllers/CMakeLists.txt"
echo " - franka_example_controllers/franka_example_controllers_plugin.xml"
echo " - franka_gazebo/config/sim_controllers.yaml"
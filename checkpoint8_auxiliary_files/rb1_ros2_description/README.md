Starting the Simulation
To launch the RB1 robot simulation:

Use the following command to start the simulation environment:


ros2 launch rb1_ros2_description rb1_ros2_lift_xacro.launch.py


This will initialize the robot model and controllers in Gazebo.

Verify that the necessary topics are active by running:


ros2 topic list


You should see topics such as /lift_controller/commands, /joint_states, /odom, and others.

Controller Activation
The launch file should automatically activate the necessary controllers. 

Move the lifting unit up:
ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data: [10]" -1


Move the lifting unit down:
ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data: [0]" -1


These commands will elevate or lower the robot's lifting unit, respectively.


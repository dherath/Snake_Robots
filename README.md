## simulation of snake robots 

The open source platform Gazebo was used to build the simulations for snake robots. This project was done as part of my research work in fullfilment for my Bachelors degree in Computational Physics.

**1) Serial Snake Robot**

The snake robot consist of 21 links serially connected via revolute joints.
The movement plugin defines planar lateral  undulation. Note that the absolute path of the plugin (libmovementPlugin.so) within the serial_snake_robot folder must be enetered within the < plugin > tag inorder for the plugin to operate on the robot.

**2) Parallel Snake Robot**

The snake robot consist of 21 parallely linked via a combinaiton of ball joints and prismatic joints.
Like the above case the absolute path for the plugin within the parallel_snake_robot folder (libmovementPlugin.so)
must be entered within the model.sdf file < plugin > tag for the parallel snake.

#### papers

1. Comparison of Serial and Parallel Snake Robots for Lateral Undulation Motion using Gazebo.[[link](https://www.researchgate.net/publication/311716282_Comparison_of_Serial_and_Parallel_Snake_Robots_for_Lateral_Undulation_Motion_using_Gazebo)]
2. Simulation of Symmetric and Asymmetric movement gaits for Lateral Undulation in Serial Snake Robots. [[link](https://www.researchgate.net/publication/317015239_Simulation_of_Symmetric_and_Asymmetric_movement_gaits_for_Lateral_Undulation_in_Serial_Snake_Robots)]

#### Undergraduate Thesis : [[link](https://www.researchgate.net/publication/316471922_Simulation_of_a_Snake_Robot)] 

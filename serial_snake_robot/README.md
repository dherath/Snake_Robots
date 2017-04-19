# simulation of snake robots 

Serial Snake Robot

The snake robot consist of 21 links serially connected via revolute joints.
The movement plugin defines planar lateral  undulation. Note that the absolute path of the plugin (libmovementPlugin.so) within the serial_snake_robot folder must be enetered within the <plugin> tag inorder for the plugin to operate on the robot.

Parallel Snake Robot

The snake robot consist of 21 parallely linked via a combinaiton of ball joints and prismatci joints.
Like the above case the absolute path for the plugin within the parallel_snake_robot folder (libmovementPlugin.so)
must be entered within the model.sdf file <plugin> tag for the parallel snake.

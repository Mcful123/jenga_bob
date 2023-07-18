# jenga_bob

## Setup

You need to have lab2andDriver installed in the same catkin workspace.
Build and source the workspace with `catkin_make` and `source ./devel/setup.bash`.

## Running the simulator

In terminal 1, source using `source ./devel/setup.bash`. Then run  `roslaunch ur3_driver ur3_gazebo.launch` to launch the simulation. 

## Spawning the tower

In another terminal, source again using `source ./devel/setup.bash`. Then run `rosrun jenga_bob jenga_spawn.py` to spawn the jenga tower in the environment. Running 'jenga_spawn.py' at any point will delete any blocks currently in the environment and create another tower. 

Currently, the tower is a 3x5x1 arrangement of cubes. This will be updated later to spawn a traditional jenga tower: 18 levels with 3 blocks on each level, total of 54 blocks.

## Moving the arm

Once the blocks have been spawned, run `rosrun jenga_bob jenga_exec.py` to execute the motion of the robot arm, which is currently just starter code from the lab procedures. This will be updated at a later point to move blocks based on the mode of jenga that the robot is playing. 

## Displaying sensor data

Open a new terminal and source again using `source ./devel/setup.bash`. Running `rosrun rqt_image_view rqt_image_view image:=/cv_camera_node/image_raw` will open an image view of the camera sensor that is included in the environment.

This camera will eventually be used to detect the collapse of the jenga tower so the robot knows to stop pulling and stacking blocks.

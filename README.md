## Mapping

This is Assignment 4 of the course DD2410 Introduction to Robotics.

### Environment Setup
To run this repository, ubuntu 16.04 with ROS-kinetic is needed.

Download this repository, and put the folder into the ROS workspace, catkin_make and source the devel/setup.bash.

In this repository, I only modify
`mapping_assignment_metapackage/mapping_assignment/scripts/mapping.py`

## Run

**Terminal 1**: ```roscore```

**Terminal 2**: ```roslaunch mapping_assignment play.launch```

    This launches RVIZ for you so that you can see how the robot moves and how the map is being updated by your code.

**Terminal 3**: ```rosbag play --clock BAGFILE```

where :

    --clock:                Publish the clock time. This is to make sure that the time from the BAGFILE is published.
                            Otherwise, the robot in RVIZ will not reset if you restart the bag.

    BAGFILE:                Path to a rosbag, which are located  here:  
                            mapping_assignment_metapackage/mapping_assignment/bags/ 					

**Terminal 4**: ```rosrun mapping_assignment main.py```


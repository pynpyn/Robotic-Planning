# Robotic-Planning

## MP2
Following instructions assume a reasonably up to date version of Ubuntu. Start in the root directory 
of this project and, in a terminal, do the following:

1. `sudo apt-get install libgeos-dev`
2. `sudo pip install shapely`
3. `roslaunch test_youbot.launch`
4. open a new terminal in the project root
5. `python mp2/mp2.py`

To change the goal position, open mp2.py and set GOAL_POS to anything you want.

## MP3 Part 1
Same directions as mp2 above except you may need to install more libraries
(numpy, scipy). Also, obviously, do `python mp3/mp3p1.py`. 

You can tune the step size in `rrt.py`. At the top of `mp3p1.py` there are some
lines that compute the RRT tree. You can uncomment the plot_RRT line to see the
full graph of connected RRTs and shortest path in the graph, but you have to
close it before the cmd_vel messages get published. Also, the second argument
to buildRRT is the number of nodes per tree, which you can tune. You can change
the GOAL_POS variable if you feel fancy, or the EPSILON variable if you want to
make the robot faster/slower. 

Quick note: My merge function is nonstandard, read it carefully to see how it
works. The method in the book didn't work for me, I suspect because the chances
of two trees finding the same sample is low because the step sizing is so
small. I ignored it and extended T1, and tested to see if T2 could be extended
to include qnew ignoring step size. 




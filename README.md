# Robotic-Planning
1. control_turtle
    This C++ program will control our turtle to track a goal position. 
    After user published a topic of goal position and orientation, 
    this program will subscribe to that topic and also subscribe to 
    turtle's current position and orientation. Then, PID control will 
    track the goal position and drive the angle and distance errors to a 
    sufficiently small amount. 
    
Usage:
1. Run roscore
2. Run turtlesim_node
3. Run control_turtle
4. publish a goal position topic 


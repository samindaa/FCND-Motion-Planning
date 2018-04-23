## Project: 3D Motion Planning
### Saminda Abeyruwan
![Quad Image](./misc/enroute.png)

---
### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

* __planning_utils.py__:
    * _create_grid_: creates a dense 2D grid such that the no-fly zones are marked with one. 
'data' contains the matrix read from 'colliders.csv' where posX maps to north and posY maps to east coordinates.
(0, 0) represents minimum north and east coordinates, while the resolution of the grid is set to 1 meters. 
We have developed this mapping in one of the assignments.
    * _a_star_: find A* path from the grid, start, and goal tuples. The implementation only
provides L^2-norm as the heuristic, which is implemented in the 'heuristic' function. We have implemented 
this form of A* in one of the assignments.  
    * _valid_actions_: provides at most four move action of one meter towards north, south, east and west. Only valid 
actions within the grid are allowed.

* __motion_planning.py__
    * The initial script resembles the _backyard_flyer.py_, we have implemented in the 
first project. Instead of moving around square, in this script we define a start and goal
tuples, and used the A* to find a path. 
    * The 'plan_path' has been called after the drone is armed, which:
        * reads the colliders.csv, 
        * creates the grid, with safety distance and drone altitude five meters,
        * set up the start tuple, which is just a transformation of the 
        grid offset to center of the map, and goal tuple, which is 10 meters from 
        north and east. 
        * calls A* to gets the path and setup the 'waypoints' from the path. 
        The waypoints are also transformed to the map coordiates using north and east offsets.
    * The shortest path from start to goal is in the diagonal. But the allowed actions does not have 
    the diagonal action build-in. Therefore, the A* needs to zig-zag to reach the goal. E.g.,:
        * start tuple (316, 445), and goal tuple (326, 455)
        * The first action selected by A* is to move north. Even though there is another potential action east
        with the same cost (with the heuristic) added to the queue, Python PriotyQueue selects the state that moves 
        the drone to north. 
        * From this new state, only east is the movement that the heuristic selects as the closest to the goal.
        * Therefore, A* selects actions that zig-zag the diagonal to read the goal (Figure 1).
![Demo Path](./misc/demo_motion_planning.png)          

    * The waypoint transition happens within a radius of 1 m. Once the drone follows all the waypoints are explored, the drone starts landing once the 
    velocity of the drone is less that 1 m/s. The drone disarm, if the home and global
    height difference is less than 0.1 m and the drone is very close to ground with less 
    that 0.01 m. These are the deadbands used by the demo implementation. 
             

### Implementing Your Path Planning Algorithm

TODO(saminda)

### Execute the flight

TODO(saminda)
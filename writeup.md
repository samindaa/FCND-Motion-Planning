## Project: 3D Motion Planning
### Saminda Abeyruwan
![Quad Image](./misc/enroute.png)

---
### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

* planning_utils.py:
    * create_grid: creates a dense 2D grid such that the no-fly zones are marked with one. 
'data' contains the matrix read from 'colliders.csv' where posX maps to north and posY maps to east coordinates.
(0, 0) represents minimum north and east coordinates, while the resolution of the grid is set to 1 meters. 
We have developed this mapping in one of the assignments.
    * a_star: find $A^{*}$ path from the grid, start, and goal tuples. The implementation only
provides $L^2$-norm as the heuristic, which is implemented in the 'heuristic' function. We have implemented 
this form of $A^{*}$ 
    * valid_actions: provides at most four move action of one meter towards north, south, east and west. Only valid 
actions  

TODO(saminda)

### Implementing Your Path Planning Algorithm

TODO(saminda)

### Execute the flight

TODO(saminda)
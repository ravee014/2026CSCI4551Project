# route_planner (Person 2)

Finds a walking route on the UMN campus from where the wearer is now to a goal, using A* on the campus path graph, and publishes it so RViz (and the guidance node) can use it.

It does **not** move the wearer. It only decides which footpaths to take.

## How it works
1. Loads the path graph from `campus_sim/data/` (8,571 points and the connections between them)
   - `route_graph_nodes.csv`: `id, x, y` (metres, same frame as Gazebo)
   - `route_graph_edges.csv`: `u, v` (two connected points; cost = straight-line distance)
2. Gets the wearer's position from `/wearer/odom`
3. Snaps the wearer and the goal to their nearest graph points
4. Runs A* (heuristic = straight-line distance to the goal)
5. Publishes the route on `/route` once a second

## Code walkthrough (`route_planner/route_planner_node.py`)
Everything is in one class, `RoutePlanner`:

| Function | What it does |
|---|---|
| `__init__` | Reads the `goal_x` / `goal_y` parameters (default 300, 100). Loads the CSVs into two dicts: `self.nodes = {id: (x, y)}` and `self.edges = {id: [neighbor ids]}`. Each edge is added both ways because paths are walkable in both directions. Then it sets up the `/route` publisher, the `/wearer/odom` and `/goal_pose` subscribers, and a 1-second timer. |
| `odom_cb` | Saves the wearer's (x, y) every time odometry arrives. On the **first** message it calls `plan()`, because that's the first moment we know where the wearer is. |
| `goal_cb` | Called when you click "2D Goal Pose" in RViz. Saves the clicked point as the new goal and calls `plan()`. |
| `nearest(p)` | Checks every graph point and returns the closest one to `p` (simple `min()` over ~8.5k points; it's instant). |
| `a_star(start, goal)` | The A* search (details below). Returns a list of node ids from start to goal. |
| `plan()` | Snaps the wearer and goal with `nearest()`, runs `a_star()`, and builds a `nav_msgs/Path`: wearer position → route points → exact goal. Then it publishes it. |
| `publish()` | Re-sends the last route with a fresh timestamp. The timer calls it every second so nodes that start late (RViz, guidance) still get the route. |
| `main()` | Starts ROS, creates the node and spins (keeps it running). |

### How `a_star` works
- `queue`: a priority queue (`heapq`) of `(estimated total cost, node)`. It always expands the most promising node first.
- `cost[n]`: the shortest distance found so far from the start to `n` (called *g*).
- `h(n)`: the straight-line distance from `n` to the goal (the heuristic). It never overestimates, so A* finds the true shortest route.
- `parent[n]`: which node we came from. Used at the end to walk backwards from the goal and rebuild the route.
- For each neighbor `m`: if going through `n` is cheaper than what we had, update `cost[m]` and `parent[m]` and push `(cost + h, m)` onto the queue.
- It stops when the goal is popped. On campus a route usually takes ~1 ms.

## Topics
| Topic | Type | In/Out | What |
|---|---|---|---|
| `/wearer/odom` | `nav_msgs/Odometry` | in | start position |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in | new goal from RViz "2D Goal Pose" (replans) |
| `/route` | `nav_msgs/Path` | out | the route, frame `map` |

## Run
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch campus_sim sim.launch.py          # terminal 1
ros2 run route_planner route_planner          # terminal 2
```
Default goal is (300, 100). To pick another one:
```bash
ros2 run route_planner route_planner --ros-args -p goal_x:=-400.0 -p goal_y:=200.0
```
Expected output:
```
Loaded 8571 nodes
Route: 38 nodes to goal (300.0, 100.0)
```

## See it in RViz
1. Fixed Frame: `map`
2. Add → By topic → `/route` → Path (Line Style: Billboards, Line Width: 2)
3. Views → Type: TopDownOrtho, then scroll out (the route is ~600 m long)
4. Click **2D Goal Pose** anywhere on the map to plan a new route

## Notes
- The graph is all connected, so A* always finds a route.
- If the sim is restarted, restart this node too.
- Files: all the code is in `route_planner/route_planner_node.py`. `__init__.py` and `resource/route_planner` are empty on purpose, because ROS needs them to find the package.

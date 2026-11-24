import csv, heapq, math, os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('goal_x', 300.0)
        self.declare_parameter('goal_y', 100.0)
        self.goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)

        # Load the graph: nodes = {id: (x, y)}, edges = {id: [neighbor ids]}
        data = os.path.join(get_package_share_directory('campus_sim'), 'data')
        self.nodes = {int(r['id']): (float(r['x']), float(r['y']))
                      for r in csv.DictReader(open(os.path.join(data, 'route_graph_nodes.csv')))}
        self.edges = {n: [] for n in self.nodes}
        for r in csv.DictReader(open(os.path.join(data, 'route_graph_edges.csv'))):
            u, v = int(r['u']), int(r['v'])
            self.edges[u].append(v)
            self.edges[v].append(u)
        self.get_logger().info(f'Loaded {len(self.nodes)} nodes')

        self.pos = None    # wearer (x, y)
        self.path = None   # last planned Path
        self.pub = self.create_publisher(Path, '/route', 10)
        self.create_subscription(Odometry, '/wearer/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)  # RViz "2D Goal Pose"
        self.create_timer(1.0, self.publish)  # republish every second so late nodes get it

    def odom_cb(self, msg):
        first = self.pos is None
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if first:
            self.plan()

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan()

    def nearest(self, p):
        return min(self.nodes, key=lambda n: math.dist(self.nodes[n], p))

    def a_star(self, start, goal):
        h = lambda n: math.dist(self.nodes[n], self.nodes[goal])
        queue = [(h(start), start)]
        cost = {start: 0.0}
        parent = {start: None}
        while queue:
            _, n = heapq.heappop(queue)
            if n == goal:
                route = []
                while n is not None:
                    route.append(n)
                    n = parent[n]
                return route[::-1]
            for m in self.edges[n]:
                c = cost[n] + math.dist(self.nodes[n], self.nodes[m])
                if c < cost.get(m, math.inf):
                    cost[m] = c
                    parent[m] = n
                    heapq.heappush(queue, (c + h(m), m))
        return []

    def plan(self):
        if self.pos is None:
            return
        route = self.a_star(self.nearest(self.pos), self.nearest(self.goal))
        self.path = Path()
        self.path.header.frame_id = 'map'
        for x, y in [self.pos] + [self.nodes[n] for n in route] + [self.goal]:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y = x, y
            self.path.poses.append(p)
        self.get_logger().info(f'Route: {len(route)} nodes to goal {self.goal}')
        self.publish()

    def publish(self):
        if self.path:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.path)


def main():
    rclpy.init()
    rclpy.spin(RoutePlanner())


if __name__ == '__main__':
    main()

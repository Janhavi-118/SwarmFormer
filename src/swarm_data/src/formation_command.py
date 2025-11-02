#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from swarm_msgs.msg import FormationInfo  
from std_msgs.msg import String  

import math

class FormationCommander(Node):
    def __init__(self):
        super().__init__('formation_commander')

        # Declare parameters
        self.declare_parameter('formation_type', 'line')      # e.g., "line", "circle", "diamond", "triangle"
        self.declare_parameter('formation_spacing', 1.5)      # Distance between robots (m)
        self.declare_parameter('formation_goal', [0.0, 0.0])  # Global formation center [x, y]
        self.declare_parameter('num_robots', 10)              # Number of robots in swarm

        # Get parameters
        self.formation_type = self.get_parameter('formation_type').value
        self.formation_spacing = self.get_parameter('formation_spacing').value
        self.formation_goal = self.get_parameter('formation_goal').value
        self.num_robots = self.get_parameter('num_robots').value

        # Publishers for formation goals: one per robot
        self.goal_pubs = []
        for i in range(self.num_robots):
            topic_name = f'/formation/goal_tb3_{i}'
            pub = self.create_publisher(Point, topic_name, 10)
            self.goal_pubs.append(pub)

        # Publisher for typed FormationInfo message
        self.formation_info_pub = self.create_publisher(FormationInfo, '/formation/info', 10)

        self.param_pub = self.create_publisher(String, '/formation/params', 10)

        # Timer to periodically publish formation info and goals
        self.timer = self.create_timer(1.0, self.publish_formation)

        self.get_logger().info(
            f"FormationCommander initialized: {self.num_robots} robots, "
            f"type='{self.formation_type}', spacing={self.formation_spacing}, "
            f"goal={self.formation_goal}"
        )

    def compute_formation_offsets(self, formation_type, spacing, n):
        offsets = []
        if formation_type == 'line':
            center = (n - 1) / 2.0
            for i in range(n):
                x = (i - center) * spacing
                y = 0.0
                offsets.append((x, y))

        elif formation_type == 'circle':
            if n == 1:
                offsets = [(0.0, 0.0)]
            else:
                radius = spacing / (2 * math.sin(math.pi / n))
                for i in range(n):
                    theta = 2 * math.pi * i / n
                    x = radius * math.cos(theta)
                    y = radius * math.sin(theta)
                    offsets.append((x, y))

        elif formation_type == 'triangle':
            # Triangular lattice approx (bottom-up rows)
            row = 0
            count = 0
            while count < n:
                items_in_row = row + 1
                y = -row * spacing * math.sqrt(3) / 2
                start_x = -row * spacing / 2
                for i in range(items_in_row):
                    if count >= n:
                        break
                    x = start_x + i * spacing
                    offsets.append((x, y))
                    count += 1
                row += 1

        elif formation_type == 'diamond':
            side_len = math.ceil(math.sqrt(n))
            count = 0
            for row in range(side_len):
                for col in range(side_len):
                    if count >= n:
                        break
                    x = (row - col) * spacing / math.sqrt(2)
                    y = (row + col) * spacing / math.sqrt(2)
                    offsets.append((x, y))
                    count += 1

        else:
            # Default: all zeros (robots stacked at formation goal)
            offsets = [(0.0, 0.0)] * n

        return offsets

    def publish_formation(self):
        # Publish typed formation info
        formation_msg = FormationInfo()
        formation_msg.formation_type = self.formation_type
        formation_msg.formation_spacing = self.formation_spacing
        formation_msg.formation_goal = list(self.formation_goal)  # Must be [x, y]

        self.formation_info_pub.publish(formation_msg)

        params_json = String()
        params_json.data = (
            f'{{"type":"{self.formation_type}",'
            f'"spacing":{self.formation_spacing},'
            f'"goal":{self.formation_goal}}}'
        )
        self.param_pub.publish(params_json)

        # Compute and publish per-robot formation goals
        offsets = self.compute_formation_offsets(self.formation_type,
                                                 self.formation_spacing,
                                                 self.num_robots)
        for i, (offset_x, offset_y) in enumerate(offsets):
            goal_point = Point()
            goal_point.x = self.formation_goal[0] + offset_x
            goal_point.y = self.formation_goal[1] + offset_y
            goal_point.z = 0.0
            self.goal_pubs[i].publish(goal_point)

            self.get_logger().debug(
                f"Published formation goal to tb3_{i}: "
                f"({goal_point.x:.2f}, {goal_point.y:.2f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = FormationCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

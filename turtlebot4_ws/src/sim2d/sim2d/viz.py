import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import pygame
import math

# --- Constants ---
WINDOW_SIZE = 500
ROBOT_RADIUS = 10
UPDATE_HZ = 30

class Simple2DVisualizer(Node):
    def __init__(self):
        super().__init__('ttb4_visualizer')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot_state',
            self.odom_callback,
            10)

        self.x, self.y, self.theta = WINDOW_SIZE // 2, WINDOW_SIZE // 2, 0.0

        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption("2D TurtleBot Visualization")
        self.font = pygame.font.SysFont('Arial', 16)

        self.timer = self.create_timer(1.0 / UPDATE_HZ, self.update)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2 * math.atan2(qz, qw)

    def update(self):
        self.screen.fill((255, 255, 255))
        pygame.draw.circle(self.screen, (0, 0, 255), (int(self.x), int(self.y)), ROBOT_RADIUS)

        # Heading line
        end_x = int(self.x + ROBOT_RADIUS * 2 * math.cos(self.theta))
        end_y = int(self.y + ROBOT_RADIUS * 2 * math.sin(self.theta))
        pygame.draw.line(self.screen, (255, 0, 0), (int(self.x), int(self.y)), (end_x, end_y), 2)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    viz = Simple2DVisualizer()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
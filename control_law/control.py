import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transformations import euler_from_quaternion
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_yaw = 0.0

    def odom_callback(self, msg):
        # Actualiza la posición y orientación actuales del robot
        orientation_q = msg.pose.pose.orientation
        euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        _, _, self.current_yaw = euler  # Actualiza current_yaw con el valor de yaw (z)

        self.get_logger().info(f'Posición actual: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, yaw={self.current_yaw:.2f}')
    
    def move_to_point(self, xe, ye, k1, k2):
        # Calcular d y alpha
        d = math.sqrt(xe**2 + ye**2)
        alpha = math.atan2(ye, xe) - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Verificar si el robot está cerca del punto objetivo
        if d < 0.1:  # Considera un umbral pequeño para detenerse
            self.stop_robot()
            return True

        # Calcular v y w usando las leyes de control dadas
        v = k1 * d * math.cos(alpha)
        w = k2 * alpha + k1 * math.sin(alpha) * math.cos(alpha)

        # Publicar velocidades al robot
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.publisher_.publish(twist)

        return False

    def stop_robot(self):
        # Publicar velocidades cero para detener el robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    k1, k2 = 0.8, 0.5  # Parámetros de control

    try:
        while True:
            xe, ye = map(float, input("Ingrese las coordenadas del punto objetivo (x, y): ").split(","))
            while not robot_controller.move_to_point(xe, ye, k1, k2):
                rclpy.spin_once(robot_controller, timeout_sec=0.1)
            if input("¿Desea ingresar otro punto? (s/n): ").lower() != 's':
                break
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Pose2D

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.start_path = False #bandera para ingresar coordenadas
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Punto objetivo
        self.target = Pose2D()
        self.target.x = 0.0
        self.target.y = 0.0

        # Parámetros de control
        self.k1 = 0.5
        self.k2 = 1

        # Estado actual
        self.current = Pose2D()
        
        self.current.x = 0.0
        self.current.y = 0.0
        self.current_yaw = 0.0

        self.timer = self.create_timer(0.1, self.update_movement)


    def odom_callback(self, msg):
        # Actualiza la posición y orientación actuales del robot
        self.current.x = msg.pose.pose.position.x
        self.current.y = msg.pose.pose.position.y

        self.get_logger().info(f'Posicion actual: x={self.current.x: .2f}, y={self.current.y: .2f}')

        # Obtén la orientación del robot en quaterniones
        orientation_q = msg.pose.pose.orientation
        # Convierte quaterniones a Euler
        euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        _, _, self.current_yaw = euler

    def update_movement(self):

        #dummy para poner las coordenadas
        if not self.start_path:

            point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
            self.target.x = float(point_x)
            self.target.y = float(point_y)
            self.start_path = True
            
            
        # Calcula la distancia y el ángulo hacia el objetivo
        dx = self.target.x - self.current.x
        dy = self.target.y - self.current.y
        distance = math.sqrt(dx**2 + dy**2)

        #Introduce un umbral para detener al robot cerca del objetivo
        self.stop_threshold = 0.2
        self.slow_down_threshold = 1.0

        # Calcula el ángulo relativo al objetivo
        alpha = math.atan2((self.target.y-self.current.y),(self.target.x-self.current.x))-2*self.current_yaw
        
        distance_tolerance = 0.15
        
        #Verifica la distancia al objetivo para ajustar la velocidad
        if distance <= distance_tolerance:
            # Detiene al robot si está muy cerca del objetivo
            v = 0.0
            w = 0.0
            
            vel_msg = Twist()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            self.publisher_.publish(vel_msg)
            
            self.start_path = False
            answer = input("Desea ingresar otro punto? y/n:  ")

            if answer == "y":
                pass
            else:
                #parar el nodo
                #salir
                #arreglar
                rclpy.shutdown()
                exit(-1)
            
            
        else:
            # Velocidad normal si está lejos del objetivo (Ley de Control)
            v = self.k1 * distance * math.cos(alpha)
            w = self.k2 * alpha + self.k1 * math.sin(alpha) * math.cos(alpha)

            #-- limitar la velocidad --
            if v > 1:
                v = 1.0

            if v < -1:
                v = -1.0

            if w > 1:
                w = 1.0

            if w < -1:
                w = -1.0
            
        # Publica las velocidades al robot
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        self.publisher_.publish(vel_msg)
        self.get_logger().info(f'Moviendo el robot: vel linear: {v:.2f}, vel angular: {w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
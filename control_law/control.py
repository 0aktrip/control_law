import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
#from transformations import euler_from_quaternion
import math
from math import pow, atan2, sqrt, asin, cos, sin, atan

import os,sys

class RobotController(Node):
    
    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info("Se inicializa el nodo")
        
        # GENERAR PUBLISHERS Y SUBSCRIBERS
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        #self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        #Ganancias de control
        #k1 velocidad lineal
        #k2 velocidad angular
        self.k1 = 0.8
        self.k2 = 0.5

        #Posicion robot todo momento
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0

        self.theta = 0.0

        #Posicion objetivo
        self.goal_pose = Pose2D()
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        
        self.start_path = False
        
        self.timer = self.create_timer(0.5, self.move_to_point)
        
    #Callback del lidar
    def laserscan_callback(self,msg):
        #leer datos del lidar
        #la logica no la debes hacer aqui, pasa los datos a otro metodo
        #el lidar regresa un array de 360 puntos
        #derecha frente 90-180
        #izquierda frente 180-270
        
        #msg.ranges[0:179]
        
        return None
    
    def odom_callback(self, msg_odom):

        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)

        orientation = msg_odom.pose.pose.orientation
        
        # tetha quaternion
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        
        self.theta = math.atan2(t3,t4)

    # error hacia el objetivo
    def euclidean_distance(self, goal_pose):
        
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
    
    def ley_control(self):

        # Error a
        x_err = self.goal_pose.x - self.pose.x
        y_err = self.goal_pose.y - self.pose.y
        
        a = sqrt(x_err**2 + y_err**2)
        
        if(self.theta > math.pi):
            self.theta = self.theta - 2 * math.pi
            
        if(self.theta < -math.pi):
            self.theta = self.theta + 2 * math.pi
            
        # Error alpha
        alpha = atan2(y_err,x_err) - self.theta
        
        # Ley de control
        v = self.k1 * a * cos(alpha)
        
        if v > 1:
            v = 1.0
                
        if v < -1:
            v = -1.0
            
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)
        
        if w > 1.0:
            w = 1.0

        if w < -1.0:
            w = -1.0

        #publica cada 1seg
        self.get_logger().info(f'goal pose: ({self.goal_pose.x} , {self.goal_pose.y}) - robo pose: ({round(self.pose.x,2)},{round(self.pose.y,2)}) - theta: {round(self.theta,2)} - {round(self.theta*180/math.pi,2)}', throttle_duration_sec=1)
        
        return v,w

    # Metodo para control, aqui debes de monitoriar si hay obstaculos
    # para cambiar de el control a bug
    # y regresar si ya rodio el obstaculo
    def control(self):
        
        v = 0.0
        w = 0.0

        #if obstaculo
        #  v,w = funcion logica para rodear obstaculo bug2
        #else
        #  v, w = self.ley_control()
        
        v, w = self.ley_control()
            
        return v, w

    def move_to_point(self):
        
        #mover hacia el objetivo
        goal_pose = self.goal_pose
        
        if not self.start_path:

            #parar robot
            self.stop_robot()
            
            #Datos
            point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
            
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)
            
            self.start_path = True

        
        distance_tolerance = 0.15
        
        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            
            v,w = self.control()
            
            self.mover(v,w)
                                    
        else:

            self.stop_robot()
            self.start_path = False

            while True:
                answer = input("Quieres ingresar otro punto? y/n:   ")
                
                if answer.lower() not in ('y', 'n'):
                    print("Error fancy")
                else:
                    break
                
            if answer == "y":
                pass
            else:
                sys.exit()

    # FUNCION PARA MOVER EL ROBOT
    def mover(self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
    
    # FUNCION PARA DETENER ROBOT
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("ROBOT DETENIDO")
        
def main(args=None):

    #limpiar pantalla
    os.system('clear')
    
    rclpy.init()
    
    robot_controller = RobotController()
    
    #FIXME - Salir del programa ctrl + c
    try:
        rclpy.spin(robot_controller)
    except:
        sys.exit()
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

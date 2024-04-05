import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)

    def listener_callback(self, msg):
        twist_msg = Twist()
        angle_width = 30
        dist_min = 0.2
        dist_max = 1.0
        dist_opti = 0.6
        nb_points = len(msg.ranges)
        nb_valid_points = 0
        somme_dist = 0
        somme_ang = 0
        vitesse_linear_max = 1.0
        vitesse_angular_max = 0.3

        for i in range(nb_points):
          if 0 <= i < angle_width/2 or (nb_points - angle_width/2) < i:
            #Le point est dans le cone
            if dist_min < msg.ranges[i] < dist_max:
              #Le point est aussi à la bonne distance
              if i > 180:
                ang = i - 360
              else:
                 ang = i
              dist = msg.ranges[i]
              nb_valid_points +=1
              somme_dist += dist
              somme_ang += ang

        #Ceci est notre meilleure estimation d'où est la personne
        if nb_valid_points != 0:
          moy_dist = somme_dist / nb_valid_points
          moy_ang = somme_ang / nb_valid_points
          self.get_logger().info(f"moy_dist = {moy_dist}, moy_ang = {moy_ang}")
        else:
           #IDEE? ajouter la fonction où il faut suivre la dernière postion valide avant que le point sorte du cone
           moy_dist = dist_opti
           moy_ang = 0
           self.get_logger().info("Pas de point détecté")
        #Ceci sont les vitesses linéaire et angulaire à appliquer au robot pour qu'il se déplace à notre meilleure estimation de la personne
        twist_msg.linear.x = (moy_dist - dist_opti) * vitesse_linear_max
        twist_msg.angular.z = (moy_ang - 0) * vitesse_angular_max
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    follow_me = FollowMe()

    rclpy.spin(follow_me)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_me.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
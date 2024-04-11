import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#initialisation de la classe FollowMe en tant que node
class FollowMe(Node):
    def __init__(self):
        super().__init__('follow_me')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) #On configure le node FollowMe en tant que publisher sur le topic cmd_vel de type Twist
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10) #On configure le node FollowMe en tant que subscriber sur le topic scan de type LaserScan
        self.init_follow_me = 0 #On initialise la variable init_follow_me à 0. Cette dernière a pour but de permettre au robot de regarder le nombre de points qui composent la personne à suivre lors du prmier scan uniquement
        self.taille_personne = 0 #On initialise la variable taille_personne à 0. Cette dernière enregistre le nombre de points qui composent la personne
    
    #On initialise la fonction qui sera lancée en boucle
    def listener_callback(self, msg):
        twist_msg = Twist()
        angle_width = 50 #Taille du cône de détection
        dist_min = 0.2 #Distance minimale pour qu'un point soit à bonne distance
        dist_max = 0.5 #Distance maximale pour qu'un point soit à bonne distance
        dist_opti = 0.3 #Distance optimale pour qu'un point soit à bonne distance
        nb_points = len(msg.ranges) #Nombre de points qui rentrent dans la zone de bonne distance
        nb_valid_points = 0
        somme_dist = 0
        somme_ang = []
        somme_ang_val = 0
        nb_somme_ang_final = 0
        vitesse_linear_max = 0.5
        vitesse_angular_max = 0.05


        for i in range(nb_points):
          if 0 <= i < angle_width/2 or (nb_points - angle_width/2) < i:
            #Le point est dans le cone
            if dist_min < msg.ranges[i] < dist_max:
              #Le point est aussi à la bonne distance
              if self.init_follow_me == 0: #Si c'est la première fois que l'on exécute un scan, on dit que la personne fait un point de plus
                self.taille_personne += 1
              if i > 180:
                ang = i - 360
              else:
                 ang = i
              dist = msg.ranges[i]
              nb_valid_points +=1
              somme_dist += dist
              somme_ang.append(ang)
              nb_somme_ang = len(somme_ang)
              milieu_somme_ang = nb_somme_ang // 2
              valeurs_du_milieu = somme_ang[milieu_somme_ang - (self.taille_personne // 2) : milieu_somme_ang + (self.taille_personne // 2)] #Pour réduire le risque d'erreur de tracking du robot, on lui dit que la moyenne des angles qu'il doit prendre en compte pour suivre sa cible est le centre de tout ce qu'il voit +/- la moitié de la personne
              nb_somme_ang_final = len(valeurs_du_milieu)
              self.get_logger().info(f"taille_personne= {self.taille_personne}, valeurs_du_milieu= {len(valeurs_du_milieu)}")
        
        if self.taille_personne != 0: #Une fois le premier scan finit, on passe la variable init_follow_me à 1 pour ne plus recompter le nombre de points qui composent une personne
          self.init_follow_me = 1

        for i in range(nb_somme_ang_final):
          somme_ang_val += valeurs_du_milieu[i]

        #Ceci est notre meilleure estimation d'où est la personne
        if nb_valid_points != 0:
          moy_dist = somme_dist / nb_valid_points
          moy_ang = somme_ang_val / nb_somme_ang_final
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
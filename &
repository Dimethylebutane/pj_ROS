import numpy as np
#import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

def clamp(m, a, M):
    return min(max(m, a), M)

class couloir_class(Node):
    def __init__(self):
        super().__init__('Testgreg')

        # Souscription à un topic de trigger externe (ex: vision)
        self.triggerSub = self.create_subscription(
            String,
            '/vision_trigger',
            self.trigger_cb,
            10
        )

        #paramètre
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('vmin', "0.3")
        self.declare_parameter('DEBUG', '0')

        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')
        cmdveltopic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.vmin = float(self.get_parameter('vmin').get_parameter_value().string_value)

        self.Kl = 0.5 #raideur ressort
        self.La = 0.3 #longueur ressort

        self.frot = 1.5 #frotement rotation

        #Dynamic state
        self.I = 1.95
        self.w = 0.0 #vitesse rotation z

        #subscribe to lidar
        self.lidarSub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_cb,
            10)
        self.lidarSub  # prevent unused variable warning

        #commande envoyee au robot (vitesse + rotation)
        self.botPub = self.create_publisher(
                Twist,
                cmdveltopic,
                10)
        self.botMsg = Twist()
        self.botPub

    def moving_average(self, r, n=10):
        ret = np.cumsum(r, dtype=np.float32)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    def lidar_cb(self, msg):
        inc = msg.angle_increment #rad, combien de radian entre chaque mesure

        r = msg.ranges #distance brut par le lidar
        N = len(r) #lidar = 360 valeurs

        #convert to np array
        r = np.abs(np.array(r, dtype=np.float32))
        r = np.append(r[-N//4:][::-1], r[:N//4]) #-pi/2 à pi/2
        r[r == np.inf] = 3 #on enlève les inf pour les calcules
        r[r == np.nan] = 2 #on enlève les nan (jic)
        N = N//2 #On garde uniquement la moiteir des valeurs, les laser qui mesure devant le robot
        n = 4
        r = self.moving_average(r, n=n) #moyenne glissante
        N = N - n + 1 #la moyenne à supprimer quelque valeurs

        #angles en radian des faisceaux
        angles = np.linspace(-np.pi/2 + inc*(n-1)/2, np.pi/2 - inc*(n-1)/2, N, True) #en rad l'angle de chaque rayon
        
        #couple de rotation
        Mz = (r-self.La) * np.cos(angles/1.08) * self.Kl * self.La
        #                         correction pour eviter de trop froler les mur
        #Mz[r>=10] = 0 #supprime les inf
        Mz = Mz * np.sin(angles) #si obstacle gauche, theta < 0 => sin < 0 => Mz < 0 => tourne à droite

        #Physics
        Mz = np.sum(Mz) #somme des moments = moment de rotation

        if self.DEBUG:
            print("Mz:", Mz)

        wfrot = self.w * abs(self.w) * self.frot # f.w²
        self.w += (Mz - wfrot)/self.I            # Iw. = Mz
        self.w = min(max(-1.5, self.w), 1.5)     #clamp -1.5; 1.5, eviter de construire trop d'inertie

        if self.DEBUG:
            print("v:", self.vmin, "w:", self.w)

        #envoi de la comande
        self.botMsg = Twist()
        self.botMsg.linear.x = self.vmin
        self.botMsg.angular.z = self.w 

        self.botPub.publish(self.botMsg)
        
    def trigger_cb(self, msg):
        if msg.data.strip().lower() == "stop":
            self.get_logger().warn("Message 'stop' reçu : arrêt du nœud. PASSAGE EN MODE SUIVIE DE LIGNE")
            rclpy.shutdown()

def main(args=None):
    print("[I] Starting couloir challenge IA solver of the dead")
    rclpy.init(args=args)

    minimal_subscriber = couloir_class()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import numpy as np
#import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

def clamp(m, a, M):
    return min(max(m, a), M)

class couloir_class(Node):
    def Ll(self, angles):
        return 0.4 * np.cos(angles)**7 #longueur ressort
    def La(self, angles):
        return 0.3 * np.cos(angles/4)**2 + 0.1 #longueur ressort

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

        self.DEBUG = (self.get_parameter('output_topic').get_parameter_value().string_value == '1')
        cmdveltopic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.vmin = float(self.get_parameter('vmin').get_parameter_value().string_value)

        #angle entre -pi/2 et pi/2
        self.theta = np.linspace(-np.pi/2, np.pi/2, 180, True)

        self.Ka = 0.2 #raideur ressort angulaire
        self.Kl = 0.5 #raideur ressort linéaire
        if self.DEBUG:
            print(self.Ka)

        #Dynamic state
        self.m = 5.0
        self.v = 0.0 #vitesse x
        self.I = 0.9
        self.w = 0.0 #vitesse rotation z

        #subscribe to lidar
        self.lidarSub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_cb,
            10)
        self.lidarSub  # prevent unused variable warning

        self.imuSub = self.create_subscription(
            Imu,
            'imu',
            self.imu_cb,
            10)
        self.prevTime = 0
        self.imuPos = np.zeros(3) #x, y, a
        self.imuV = np.zeros(2)
        self.imuW = 0.0
        self.imuMat = np.array([[1, 0, 0],[0, 1, 0]])

        self.cibSub = self.create_subscription(
            Vector3,
            'Cible_loc',
            self.cible_cb,
            10)
        self.cibSub  # prevent unused variable warning
        self.cib_theta = 0    #angle avec la cible
        self.bool_cib = False #la cible a déjà été vu


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

    def cible_cb(self, msg):
        self.cib_theta = msg.y*msg.z #get theta if confiance else 0
        self.bool_cib = msg.z
        pass

    def lidar_cb(self, msg):
        inc = msg.angle_increment #rad

        r = msg.ranges #distance brut par le lidar
        N = len(r) #lidar = 360 valeurs
        
        #convert to np array
        r = np.abs(np.array(r, dtype=np.float32))
        r = np.append(r[-N//4:][::-1], r[:N//4]) #-pi/2 à pi/2
        r[r == np.inf] = 2 #on enlève les inf pour les calcules
        r[r == np.nan] = 2 #on enlève les nan 
        N = N//2 #On garde uniquement la moiteir des valeurs, les laser qui mesure devant le robot
        n = 4
        r = self.moving_average(r, n=n) #moyenne glissante
        N = N - n + 1 #la moyenne à supprimer quelque valeurs

        #angles en radian des faisceaux
        angles = np.linspace(-np.pi/2 + inc*(n-1)/2, np.pi/2 - inc*(n-1)/2, N, True) #en rad l'angle de chaque rayon
        
        #force de freinage → inutilisé voir vmin
        Fx = (r-self.Ll(angles)) * np.cos(angles) * self.Kl
        Fx[Fx > 0] = 0 #les ressorts linéaire nous repousse uniquement. On supprime les ressorts qui nous attire

        #couple de rotation
        Mz = (r-self.La(angles)) * np.cos(angles/1.08) * self.Kl * self.La(angles)
        #                               correction pour eviter de trop froler les mur
        #Mz[r>=10] = 0 #supprime les inf
        Mz = Mz * np.sin(angles) #si obstacle gauche, theta < 0 => sin < 0 => Mz < 0 => tourne à droite

        confiance = np.sum(abs(Mz))

        #Physics
        Fx = np.sum(Fx) #somme des force = résultante de freinage →inutilisé, voir vmin
        Mz = np.sum(Mz) #somme des moments = moment de rotation
        if abs(self.cib_theta) > 0.03: #2 deg
            print("CIB REDUCE MZ")
            Mz = Mz/4 - self.cib_theta / 1.5
            self.cib_theta /= 5 #decay

        if self.DEBUG:
            print("Fx:", Fx, "Mz:", Mz)
        
        self.v = max(self.m*(1 + Fx), self.vmin)    #clamp vmin → on max la vitesse vu que ça passe le robot est lent

        wfrot = self.w * abs(self.w) * 1.25 * (1 + 0.12*self.bool_cib)
        self.w += (Mz - wfrot)/self.I
        self.w = min(max(-1.5, self.w), 1.5)        #clamp -2; 2, eviter de construire trop d'inertie

        if self.DEBUG:
            print("v:", self.v, "w:", self.w)

        #envoi de la comande
        self.botMsg = Twist()
        self.botMsg.linear.x = self.v
        self.botMsg.angular.z = self.w 

        self.botMsg.linear.y = clamp(0.0, (20-confiance) / 16.0, 1.0)
        
        if self.DEBUG:
            print("confiance:", self.botMsg.linear.y)

        self.botPub.publish(self.botMsg)
        
    def imu_cb(self, msg):#Euler null, à changer ?
        Dt = msg.header.stamp
        Dt = Dt.nanosec/(1e9)
        Dt, self.prevTime = Dt-self.prevTime, Dt
        if Dt <0 : #handle second decalage, evite de monter trop haut en seconde pr garder précision en ns
            Dt += 1.0
        
        dangl = msg.angular_velocity.z * Dt
        rotation = np.array([[np.cos(dangl), -np.sin(dangl)],[np.sin(dangl), np.cos(dangl)]])
        
        self.V = rotation@self.imuV #rotation en 1er je pense ?
        self.imuV += Dt * np.array([msg.linear_acceleration.x, msg.linear_acceleration.y]) #speed
        self.imuPos += np.array([self.imuV[0]*Dt, Dt*self.imuV[1], dangl])

        aW = self.imuPos[-1]

        print("POS :", self.imuPos)
        print("quat:", msg.orientation)
        aq = np.arccos(msg.orientation.w)
        av = np.arctan2(self.imuV[1], self.imuV[0])
        am = (aW + av + aq) / 3 #change orientation based on multiple sources

        self.imuPos[-1] = am 
        self.imuMat = np.array([[np.cos(am), -np.sin(am), self.imuPos[0]], [np.sin(am), np.cos(am), self.imuPos[1]]])
        self.imuV = np.array([np.cos(am), np.sin(am)]) * np.linalg.norm(self.imuV)

    def trigger_cb(self, msg):
        if msg.data.strip().lower() == "stop":
            self.get_logger().warn("Message 'stop' reçu : arrêt du nœud.")
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

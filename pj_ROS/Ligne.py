import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2


class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        
        # quand la balle est detecter, permet d'eteindre le noeud actuelle
        self.triggerSub = self.create_subscription(
            String,
            '/vision_trigger',
            self.trigger_cb,
            10
        )

        # Souscription à l'image compressée
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10
        )
        self.subscription  # évite un warning
        
        # Paramètres
        self.declare_parameter('DEBUG', '0')
        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')

        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('vmin', "0.3")
        self.declare_parameter('k_p', 0.003)
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value
        cmdveltopic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.vmin = float(self.get_parameter('vmin').get_parameter_value().string_value)

        # Publisher Twist pour cmd_vel
        self.botPub = self.create_publisher(Twist, cmdveltopic, 10)
        self.botMsg = Twist()

        #stopper le premier challenge
        self.triggerPub = self.create_publisher(String, '/vision_trigger', 10)

    def trigger_cb(self, msg):
        if msg.data.strip().lower() == "balle":
            self.get_logger().warn("Message 'balle' reçu : arrêt du nœud. PASSAGE EN MODE DESTRUCTION DE BALLE")
            rclpy.shutdown()


    def listener_callback(self, msg):
        # Conversion du message compressé en image OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Échec du décodage de l'image")
            return

        if self.DEBUG:
            cv2.imshow("Compressed Image", image)

        # Définir une région d'intérêt (ROI) horizontale

        # Convertir en HSV et créer un masque pour la couleur verte
        def getMoment(roi):
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 100, 100])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Calcul du centre de gravité
            return cv2.moments(mask)

        roilim = [[200, 240], [160, 200], [120, 160], [80, 120], [40, 80]]
        roi = image[200:240, :]
        M = 0
        for m, Ma in roilim:
            roi = image[m:Ma, :]
            M = getMoment(roi)
            if M["m00"] > 0:
                break

        if M["m00"] > 0:
            # Publication du signal d'arrêt pour un autre nœud
            self.triggerPub.publish(String(data="stop"))
            cx = int(M["m10"] / M["m00"])
              # Calcul de l’erreur de centrage
            width = roi.shape[1]
            center = width // 2
            error = cx - center

            # Commande robot : avance + corrige direction
            self.botMsg = Twist()
            self.botMsg.linear.x = self.vmin
            self.botMsg.angular.z = -error * self.k_p  # coeff à ajuster

            self.botPub.publish(self.botMsg)
            if self.DEBUG:
                self.get_logger().info(f"Ligne verte à x={cx}, erreur={error}")
            # Affichage sur la ROI
            cv2.circle(roi, (cx, 20), 5, (0, 255, 0), -1)
        elif self.DEBUG:
            # Pas de ligne détectée → ne publie rien
            self.get_logger().info("Aucune ligne détectée, aucune commande publiée.")
            
        # Affichage
        if self.DEBUG:
            cv2.imshow("ROI", roi)
            #cv2.imshow("Masque", mask)
            cv2.waitKey(1)

       


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2 as cv
from .smallest_enclosing_circle import make_circle #plus petit cercle qui entoure une liste de point, source = voir fichier

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        # Souscription à l'image compressée
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.imageCam_cb,
            10
        )
        self.subscription  # évite un warning

        self.cibleSub = self.create_subscription(
            Vector3,
            '/Cible_loc',
            self.Cible_cb,
            10
        )

        # Paramètres
        self.declare_parameter('output_topic', '/cmd_vel')
        cmdveltopic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.declare_parameter('DEBUG', '0')
        self.DEBUG = (self.get_parameter('DEBUG').get_parameter_value().string_value == '1')

        # Publisher Twist pour cmd_vel
        self.botPub = self.create_publisher(Twist, cmdveltopic, 10)
        self.botMsg = Twist()

        #stop epreuve 2
        self.triggerPub = self.create_publisher(String, '/vision_trigger', 10)

        #Cible
        self.cib_dst = 0
        self.cib_ang = 0
        self.cib_detec = 0
 

    def Cible_cb(self, msg):
        self.cib_dst = msg.x
        self.cib_ang = msg.y
        self.cib_detec = msg.z
        if self.cib_detec > 0.5:
            self.triggerPub.publish(String(data="balle")) #stop suivi de ligne
            

    def imageCam_cb(self, msg):
        # Conversion du message compressé en image OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Échec du décodage de l'image")
            return

        #if self.DEBUG:
        #    cv.imshow("Compressed Image", image)

        self.detec_balle(image.copy())
                    
        if self.DEBUG:
            # Affichage
            cv.waitKey(1)

    def detec_balle(self, img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_yellow = np.array([30, 180, 10])
        upper_yellow = np.array([40, 255, 255])
        maskb = cv.inRange(hsv, lower_yellow, upper_yellow)

        D = 11
        Y, X = np.ogrid[:D, :D]
        dist_from_center = np.sqrt((X-D//2)**2 + (Y-D//2)**2)
        kernel = np.array((dist_from_center <= D//2)*1, dtype=np.uint8)

        maskb = cv.morphologyEx(maskb, cv.MORPH_CLOSE, kernel) #fermeture pour éviter les trous
        cs, _ = cv.findContours(maskb, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE) #contour externe de la zone détectée
        if len(cs) == 0:
            return #no balle detectée

        contour = max(cs, key=lambda c: len(c))

        #draw contour
        maskb = cv.drawContours(maskb, contour, -1, (255, 0, 255), 3)

        contour = contour.reshape(max(contour.shape), 2)
        self.detec = len(contour) > 20
        print("detec?", self.detec, contour.shape)

        if self.detec:
            cx, cy, _ = make_circle(contour) #x,y only, ignore radius

            if self.DEBUG:
                print("balle trouvé, coord image:", cx, cy, "px")
       
            if self.DEBUG:
                cy = int(cy + 0.5)
                cx = int(cx + 0.5)
                maskb = cv.add(img, cv.cvtColor(maskb, cv.COLOR_GRAY2BGR))
                cv.circle(maskb, (cx, cy), 5, (0, 255, 0), -1)

                cv.imshow("mask", maskb)
                cv.waitKey(1)
        
        

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
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()

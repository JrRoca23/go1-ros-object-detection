#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped

class BottleDetector:
    def __init__(self):
        rospy.init_node("bottle_detector")
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.depth_image = None

        self.bottle_pub = rospy.Publisher("/bottle_position", PointStamped, queue_size=10)
        self.debug_img_pub = rospy.Publisher("/bottle_detector/image", Image, queue_size=1)

        # Rango de tamaño esperado (pixeles)
        self.min_width = 15
        self.max_width = 75
        self.min_height = 40
        self.max_height = 220

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("Error al convertir imagen de profundidad: %s", e)

    def image_callback(self, msg):
        if self.depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Error al convertir imagen RGB: %s", e)
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Rango de azul ajustado
        lower = np.array([95, 50, 50])
        upper = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_candidate = None
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if self.min_width <= w <= self.max_width and self.min_height <= h <= self.max_height:
                best_candidate = cnt
                break  # Solo tomamos la primera válida

        if best_candidate is not None:
            x, y, w, h = cv2.boundingRect(best_candidate)
            aspect_ratio = float(h) / float(w + 1e-5)

            M = cv2.moments(best_candidate)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                distance = self.get_valid_depth(cx, cy)

                if distance is None:
                    rospy.logwarn("Profundidad inválida en (u,v)=(%d,%d)", cx, cy)
                else:
                    pt = PointStamped()
                    pt.header.stamp = rospy.Time.now()
                    pt.header.frame_id = "camera_link"
                    pt.point.x = float(cx)
                    pt.point.y = float(cy)
                    pt.point.z = float(distance)
                    self.bottle_pub.publish(pt)

                    rospy.loginfo(f"Botella detectada en (u,v)=({cx},{cy}) distancia={distance:.2f}m")
                    rospy.loginfo(f"Ancho: {w} px | Alto: {h} px | Aspect ratio: {aspect_ratio:.2f}")

                    # Anotaciones
                    cv2.drawContours(cv_image, [best_candidate], -1, (0, 255, 0), 2)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 6, (0, 255, 0), -1)
                    cv2.putText(cv_image, f"{distance:.2f} m", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publicar imagen con anotaciones
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_img_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr("Error al convertir imagen debug: %s", e)

    def get_valid_depth(self, x, y, window_size=5):
        h, w = self.depth_image.shape
        for r in range(1, window_size + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        d = self.depth_image[ny, nx] / 1000.0
                        if d > 0 and not np.isnan(d):
                            return d
        return None

def main():
    try:
        BottleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

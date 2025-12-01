#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

class BottleApproachNode:
    def __init__(self):
        rospy.init_node("bottle_approach_node")

        # Parámetros configurables
        self.min_distance = rospy.get_param("~min_distance", 0.75)   # distancia para detenerse
        self.max_distance = rospy.get_param("~max_distance", 3.0)   # distancia para comenzar
        self.linear_speed = rospy.get_param("~linear_speed", 0.25)  # velocidad de avance
        self.angular_speed = rospy.get_param("~angular_speed", 0.15) # velocidad de giro

        self.image_width = rospy.get_param("~image_width", 640)     # ancho de la imagen
        self.center_tolerance = rospy.get_param("~center_tolerance", 80)  # px a izquierda/derecha del centro

        # Publicador y suscriptor
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bottle_sub = rospy.Subscriber("/bottle_position", PointStamped, self.bottle_callback)

        rospy.loginfo("Nodo bottle_approach_node mejorado iniciado.")
        rospy.spin()

    def bottle_callback(self, msg):
        distance = msg.point.z
        x = msg.point.x  # coordenada horizontal en píxeles

        center_x = self.image_width / 2
        left_bound = center_x - self.center_tolerance
        right_bound = center_x + self.center_tolerance

        cmd = Twist()

        if distance <= self.min_distance:
            rospy.loginfo("📍 Botella muy cerca → Detenido.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif distance > self.max_distance:
            rospy.loginfo_throttle(5, "🔍 Botella demasiado lejos → Esperando.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            cmd.linear.x = self.linear_speed

            if x < left_bound:
                cmd.angular.z = self.angular_speed
                rospy.loginfo(f"↩ Botella a la izquierda → Avanzando y girando izquierda.")
            elif x > right_bound:
                cmd.angular.z = -self.angular_speed
                rospy.loginfo(f"↪ Botella a la derecha → Avanzando y girando derecha.")
            else:
                cmd.angular.z = 0.0
                rospy.loginfo("⬆ Botella centrada → Avanzando recto.")

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    try:
        BottleApproachNode()
    except rospy.ROSInterruptException:
        pass


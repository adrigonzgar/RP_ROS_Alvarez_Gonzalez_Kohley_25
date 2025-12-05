#!/usr/bin/env python3

#############################################
#                 RESULT_NODE               #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from project_game.msg import user_msg
from project_game.srv import GetUserScore # Importamos el servicio
from std_msgs.msg import Int64

class ResultNode:
    def __init__(self):
        rospy.init_node('result_node', anonymous=True)
        
        self.username = "Unknown"
        
        # Suscriptor para saber quién juega
        rospy.Subscriber("user_information", user_msg, self.callback_user)
        
        # Suscriptor para saber CUÁNDO acaba el juego
        # (Usamos el topic antiguo solo como "señal" de que ha terminado)
        rospy.Subscriber("result_information", Int64, self.callback_game_over)
        
        rospy.loginfo("RESULT_NODE started. Waiting for game results...")
        rospy.spin()

    def callback_user(self, msg):
        self.username = msg.username
        rospy.loginfo(f"Player identified: {self.username}")

    def callback_game_over(self, msg):
        """
        Cuando recibimos la señal de Game Over, 
        LLAMAMOS AL SERVICIO para pedir la puntuación oficial.
        """
        rospy.loginfo("Game Over signal received. Requesting score via SERVICE...")
        
        # 1. Esperar a que el servicio esté disponible
        rospy.wait_for_service('user_score')
        
        try:
            # 2. Crear la conexión con el servicio
            get_score_service = rospy.ServiceProxy('user_score', GetUserScore)
            
            # 3. Hacer la petición (Request) enviando el nombre de usuario
            response = get_score_service(self.username)
            
            # 4. Mostrar la respuesta (Response)
            print("\n" + "="*30)
            print(f"   GAME OVER REPORT for {self.username}")
            print("="*30)
            print(f"   FINAL SCORE (from Service): {response.score}")
            print("="*30 + "\n")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        ResultNode()
    except rospy.ROSInterruptException:
        pass
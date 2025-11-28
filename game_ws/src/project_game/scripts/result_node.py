#!/usr/bin/env python3

#############################################
#                 RESULT_NODE               #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from project_game.msg import user_msg
from std_msgs.msg import Int64

class ResultNode:
    def __init__(self):
        rospy.init_node('result_node', anonymous=True)
        
        self.username = "Unknown"
        self.score = 0
        
        # --- SUBSCRIBERS ---
        # [cite_start]1. Recibir info del usuario (para saber quién jugó) [cite: 143]
        rospy.Subscriber("user_information", user_msg, self.callback_user)
        
        # [cite_start]2. Recibir puntuación final desde GAME_NODE [cite: 144]
        rospy.Subscriber("result_information", Int64, self.callback_result)
        
        rospy.loginfo("RESULT_NODE started. Waiting for game results...")
        rospy.spin()

    def callback_user(self, msg):
        self.username = msg.username
        rospy.loginfo(f"Player identified: {self.username}")

    def callback_result(self, msg):
        """
        Se activa cuando recibe la puntuación final.
        [cite_start]Muestra el mensaje de GAME OVER. [cite: 145]
        """
        self.score = msg.data
        rospy.loginfo("Final score received.")
        
        # Mensaje final en pantalla
        print("\n" + "="*30)
        print("       GAME OVER")
        print("="*30)
        print(f" PLAYER: {self.username}")
        print(f" FINAL SCORE: {self.score}")
        print("="*30 + "\n")

if __name__ == '__main__':
    try:
        ResultNode()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

#############################################
#               INFO_USER NODE              #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from project_game.msg import user_msg  # Import our custom message

class InfoUserNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('info_user_node', anonymous=True)
        
        # Create the Publisher
        # Topic: user_information, Message Type: user_msg
        self.publisher = rospy.Publisher('user_information', user_msg, queue_size=10)
        
        rospy.loginfo("INFO_USER node started. Waiting for data...")
        
        # Execute main logic
        self.run()

    def run(self):
        # Wait a bit to ensure connections are established
        rospy.sleep(1)
        
        # Request data via terminal (User Input)
        print("\n--- WELCOME TO THE ROS GAME ---")
        try:
            name_input = input("Enter your real name: ")
            username_input = input("Enter your username: ")
            age_input = int(input("Enter your age: "))
            
            # Create the message object and fill it
            msg = user_msg()
            msg.name = name_input
            msg.username = username_input
            msg.age = age_input
            
            # Publish the message
            rospy.loginfo(f"Sending data for: {username_input}...")
            self.publisher.publish(msg)
                     

            # Notify system that user data is ready
            rospy.set_param("/user_ready", True)

            rospy.loginfo("Data sent successfully to GAME_NODE.")
            
            # If you needed continuous operation, you would use rospy.spin() -> noted
            
        except ValueError:
            rospy.logerr("Error: Age must be an integer.")

if __name__ == '__main__':
    try:
        InfoUserNode()
    except rospy.ROSInterruptException:
        pass

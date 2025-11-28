#!/usr/bin/env python3

#############################################
#               CONTROL_NODE                #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from std_msgs.msg import String
import sys, select, tty, termios

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        
        # Publisher: Topic 'keyboard_control' - Type String
        self.publisher = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        rospy.loginfo("CONTROL_NODE started. Use ARROW KEYS to move. Press 'q' to quit.")
        
        self.run()

    def get_key(self):
        # Function to read a single key from the terminal without pressing Enter
        # This is standard Linux terminal manipulation
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': # Escape sequence for arrows
                key += sys.stdin.read(2)
        else:
            key = ''
        
        term_settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term_settings)
        return key

    def run(self):
        # Save terminal settings to restore them later
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            while not rospy.is_shutdown():
                # Read key
                key = self.get_key()
                
                command = None
                
                # Map arrow keys to Strings
                if key == '\x1b[A':
                    command = "UP"
                elif key == '\x1b[B':
                    command = "DOWN"
                elif key == '\x1b[C':
                    command = "RIGHT"
                elif key == '\x1b[D':
                    command = "LEFT"
                elif key == 'q':
                    print("\nExiting...")
                    break

                # Publish if it is a valid command
                if command:
                    rospy.loginfo(f"Sending command: {command}")
                    self.publisher.publish(command)

        except Exception as e:
            print(e)

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        ControlNode()
    except rospy.ROSInterruptException:
        pass
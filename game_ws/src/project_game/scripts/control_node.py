#!/usr/bin/env python3

#############################################
#               CONTROL_NODE                #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from std_msgs.msg import String
from project_game.srv import SetGameDifficulty  # <--- IMPORTANTE: Importar el servicio
import sys, select, tty, termios
import os

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        
        # Publisher para el movimiento (flechas)
        self.publisher = rospy.Publisher('keyboard_control', String, queue_size=10)
        
        # Cliente para el Servicio de Dificultad
        # Esperamos a que el servicio esté disponible (opcional, pero recomendado)
        # rospy.wait_for_service('difficulty') 
        self.difficulty_client = rospy.ServiceProxy('difficulty', SetGameDifficulty)
        
        rospy.loginfo("CONTROL_NODE started.")
        rospy.loginfo("Use ARROW KEYS to move (publishes to topic).")
        rospy.loginfo("Press E (Easy), M (Medium), H (Hard) to change difficulty (calls Service).")
        rospy.loginfo("Press 'q' to quit.")
        
        self.run()

    def get_key(self):
        # Función para leer teclas sin pulsar Enter (Raw mode)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': # Secuencia de escape (flechas)
                key += sys.stdin.read(2)
            # BORRAMOS EL ELIF QUE CAMBIABA \r A 'ENTER'
            return key
        return None

    def call_difficulty_service(self, level):
        """ Función auxiliar para llamar al servicio """
        try:
            # Esperamos brevemente por si el servicio no está listo aún
            rospy.wait_for_service('difficulty', timeout=1.0)
            
            # Llamada al servicio
            response = self.difficulty_client(level)
            
            if response.success:
                rospy.loginfo(f"SUCCESS: Difficulty changed to {level}. Msg: {response.message}")
            else:
                rospy.logwarn(f"FAILED: Could not change difficulty. Msg: {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException:
            rospy.logerr("Service 'difficulty' not available.")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                command = None
                
                # --- TECLAS DE MOVIMIENTO (TOPIC) ---
                if key == '\x1b[A':
                    command = "UP"
                elif key == '\x1b[B':
                    command = "DOWN"
                elif key == '\x1b[C':
                    command = "RIGHT"
                elif key == '\x1b[D':
                    command = "LEFT"
                elif key == '\r' or key == '\n':
                    command = "ENTER"
                
                # --- TECLAS DE DIFICULTAD (SERVICIO) ---
                # CORRECTO: Llaman al servicio y establecen command = None para NO publicar
                elif key == 'e' or key == 'E':
                    rospy.loginfo("User pressed E -> Requesting EASY mode...")
                    self.call_difficulty_service("easy")
                    command = None 

                elif key == 'm' or key == 'M':
                    rospy.loginfo("User pressed M -> Requesting MEDIUM mode...")
                    self.call_difficulty_service("medium")
                    command = None

                elif key == 'h' or key == 'H':
                    rospy.loginfo("User pressed H -> Requesting HARD mode...")
                    self.call_difficulty_service("hard")
                    command = None

                # --- AÑADIDO: TECLAS DE COLOR (TOPIC) ---
                # CORRECTO: Asignan un command para que SÍ se publique al final de la función
                elif key == '1':
                    command = "1"
                elif key == '2':
                    command = "2"
                elif key == '3':
                    command = "3"
                # ----------------------------------------
                
                # --- SALIR ---
                elif key == 'q':
                    print("\nExiting and shutting down all nodes...")
                    # Kill all ROS nodes
                    os.system("rosnode kill -a 2>/dev/null")
                    # Shutdown this node
                    rospy.signal_shutdown("User requested exit")
                    break

                # Publicar SOLO si se asignó un 'command' (es decir, Topic de Movimiento o Color)
                if command:
                    rospy.loginfo(f"Publishing command to Topic: {command}") # Log para ver qué se envía
                    self.publisher.publish(command)

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        ControlNode()
    except rospy.ROSInterruptException:
        pass


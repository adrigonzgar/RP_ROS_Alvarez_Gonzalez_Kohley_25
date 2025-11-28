#!/usr/bin/env python3

#############################################
#                 GAME_NODE                 #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
from project_game.msg import user_msg
from project_game.msg import game_state
from std_msgs.msg import String, Int64

class GameNode:
    def __init__(self):
        rospy.init_node('game_node', anonymous=True)

        # --- GAME VARIABLES ---
        self.player_name = ""
        self.player_username = ""
        self.player_age = 0

        # Start position (Level 1, left side)
        self.player_x = 100
        self.player_y = 400
        self.score = 0

        # States: "WELCOME", "RUNNING", "GAME_OVER"
        self.current_state = "WELCOME"

        # --- PHYSICS VARIABLES ---
        self.vel_y = 0          
        self.gravity = 2.0      # Gravity
        self.jump_force = -15   # Jump power
        self.speed = 8          # Move speed
        
        # Player Hitbox (Approximating your pixel art)
        self.player_w = 30
        self.player_h = 30

        self.is_on_ground = False
        self.is_on_ladder = False

        # --- MAP CONFIGURATION (COPIED FROM PYGAME_NODE) ---
        self.block_width = 40
        self.block_height = 25

        # 0 = Empty, 1 = Blue Platform, 2 = Yellow Ladder
        self.level_map = [
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 0
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 1
            [0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0], # 2 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 3 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 4 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 5 
            [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0], # 6 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 7 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 8
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 9
            [0,0,1,1,1,1,1,1,2,0,0,0,0,1,1,1,1,1,0,0], # 10 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 11 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 12
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 13
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], # 14
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 15 
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 16
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 17
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], # 18 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 19 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 20
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 21
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # 22
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]  # 23
        ]

        # --- PUBLISHERS ---
        self.result_publisher = rospy.Publisher('result_information', Int64, queue_size=10)
        self.state_publisher = rospy.Publisher('game_state', game_state, queue_size=10)

        # --- SUBSCRIBERS ---
        rospy.Subscriber("user_information", user_msg, self.callback_user_info)
        rospy.Subscriber("keyboard_control", String, self.callback_keyboard)

        # --- PHYSICS LOOP (60Hz for smoothness) ---
        rospy.Timer(rospy.Duration(0.016), self.update_physics)

        rospy.loginfo("GAME_NODE started...")
        self.publish_game_state()
        rospy.spin()

    # -----------------------------------------------------------
    #                   HELPER: GET TILE TYPE
    # -----------------------------------------------------------
    def get_tile_at(self, x, y):
        # Convert pixel coordinate to grid index
        col = int(x / self.block_width)
        row = int(y / self.block_height)

        # Check bounds
        if row < 0 or row >= len(self.level_map): return 0
        if col < 0 or col >= len(self.level_map[0]): return 0

        return self.level_map[row][col]

    # -----------------------------------------------------------
    #                   CALLBACKS
    # -----------------------------------------------------------

    def callback_user_info(self, msg):
        self.player_name = msg.name
        self.player_username = msg.username
        self.player_age = msg.age
        self.Welcome()

    def callback_keyboard(self, msg):
        command = msg.data

        if self.current_state == "WELCOME":
            self.Game()
            return

        if self.current_state == "GAME_OVER":
            self.Welcome()
            return

        if self.current_state == "RUNNING":
            self.process_command(command)
            # End condition (e.g., reaching top platform)
            # Checking if player is high enough (approx row 2)
            if self.player_y < 100:
                self.score = 100
                self.Final()

    # -----------------------------------------------------------
    #                   GAME LOGIC
    # -----------------------------------------------------------

    def process_command(self, command):
        # Horizontal Movement
        if command == "LEFT":
            self.player_x -= self.speed
        elif command == "RIGHT":
            self.player_x += self.speed

        # Vertical Movement
        if self.is_on_ladder:
            if command == "UP":
                self.player_y -= self.speed
            elif command == "DOWN":
                self.player_y += self.speed
        else:
            # Jumping (Only if on ground)
            if command == "UP" and self.is_on_ground:
                self.vel_y = self.jump_force
                self.is_on_ground = False 

    # -----------------------------------------------------------
    #                   PHYSICS LOOP
    # -----------------------------------------------------------

    def update_physics(self, event):
        if self.current_state != "RUNNING":
            return

        # 1. Determine Player's Critical Points
        center_x = self.player_x + (self.player_w / 2)
        center_y = self.player_y + (self.player_h / 2)
        feet_y = self.player_y + self.player_h

        # 2. Check current tile for Ladder (Type 2)
        # We check the center of the player
        current_tile = self.get_tile_at(center_x, center_y)
        
        # Also check slightly below feet to help transition onto ladder
        feet_tile = self.get_tile_at(center_x, feet_y)

        if current_tile == 2 or feet_tile == 2:
            self.is_on_ladder = True
        else:
            self.is_on_ladder = False

        # 3. Apply Gravity
        if self.is_on_ladder:
            self.vel_y = 0 # No gravity on ladder
        else:
            self.vel_y += self.gravity
            self.player_y += self.vel_y

        # 4. Ground Collision (Platform Type 1)
        # We only check for ground if we are falling (vel_y > 0) and NOT on a ladder
        self.is_on_ground = False
        
        if not self.is_on_ladder and self.vel_y >= 0:
            # Check the tile exactly at our feet
            # We look 1 pixel deep into the floor to detect collision
            tile_below_feet = self.get_tile_at(center_x, self.player_y + self.player_h + 1)
            
            if tile_below_feet == 1:
                # We hit a platform!
                # Calculate the exact Y position of that platform's top
                row = int((self.player_y + self.player_h + 1) / self.block_height)
                platform_top_y = row * self.block_height
                
                # Snap player to top of platform
                self.player_y = platform_top_y - self.player_h
                self.vel_y = 0
                self.is_on_ground = True
            
            # Ground Level fallback (in case map definition fails at very bottom)
            if self.player_y >= 550:
                self.player_y = 550
                self.vel_y = 0
                self.is_on_ground = True

        # 5. Publish State
        self.publish_game_state()

    # -----------------------------------------------------------
    #                   PHASES
    # -----------------------------------------------------------

    def Welcome(self):
        self.current_state = "WELCOME"
        # Start at bottom left
        self.player_x = 80
        self.player_y = 500
        self.score = 0
        self.vel_y = 0
        rospy.loginfo("--- WELCOME ---")
        self.publish_game_state()

    def Game(self):
        self.current_state = "RUNNING"
        self.score = 0
        rospy.loginfo("--- GAME START ---")

    def Final(self):
        self.current_state = "GAME_OVER"
        rospy.loginfo(f"--- GAME OVER: {self.score} ---")
        
        msg = Int64()
        msg.data = self.score
        self.result_publisher.publish(msg)
        self.publish_game_state()

    def publish_game_state(self):
        msg = game_state()
        msg.player_x = int(self.player_x)
        msg.player_y = int(self.player_y)
        msg.score = self.score
        msg.state = self.current_state
        self.state_publisher.publish(msg)

if __name__ == '__main__':
    try:
        GameNode()
    except rospy.ROSInterruptException:
        pass
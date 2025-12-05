#!/usr/bin/env python3

#############################################
#                 GAME_NODE                 #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
import random 
from project_game.msg import user_msg
from project_game.msg import game_state
from std_msgs.msg import String, Int64

# --- IMPORTAMOS LOS SERVICIOS (AMBOS) ---
from project_game.srv import GetUserScore, GetUserScoreResponse
from project_game.srv import SetGameDifficulty, SetGameDifficultyResponse

# --- CLASES AUXILIARES ---
class Barrel:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.speed_x = random.choice([-3, 3])
        self.speed_y = 0
        self.gravity = 0.5
        self.is_falling = True 

class Coin:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 15
        self.height = 15
        self.active = True

class GameNode:
    def __init__(self):
        rospy.init_node('game_node', anonymous=True)

        # --- GAME VARIABLES ---
        self.player_name = ""
        self.player_username = "Unknown"
        self.player_age = 0

        self.player_x = 100
        self.player_y = 400
        self.score = 0
        self.lives = 3
        self.current_state = "WELCOME"

        # --- PHYSICS VARIABLES ---
        self.vel_y = 0          
        self.gravity = 2.0      
        self.jump_force = -15   
        self.speed = 8          
        
        self.player_w = 30
        self.player_h = 30
        self.is_on_ground = False
        self.is_on_ladder = False

        # --- BARRILES & DIFICULTAD ---
        self.barrels = []
        self.last_barrel_time = 0
        self.barrel_spawn_rate = 2.0 # Dificultad por defecto (MEDIUM)

        # --- MONEDAS ---
        self.coins = [
            Coin(200, 500), Coin(600, 500),
            Coin(150, 420), Coin(700, 420),
            Coin(400, 320),
            Coin(100, 220), Coin(750, 220),
            Coin(200, 120), Coin(600, 120)
        ]

        # --- MAP CONFIGURATION ---
        self.block_width = 40
        self.block_height = 25
        self.level_map = [
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], 
            [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0], 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], 
            [0,0,1,1,1,1,1,1,2,0,0,0,0,1,1,1,1,1,0,0], 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], 
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], 
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], 
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], 
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], 
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]  
        ]

        # --- PUBLISHERS ---
        self.result_publisher = rospy.Publisher('result_information', Int64, queue_size=10)
        self.state_publisher = rospy.Publisher('game_state', game_state, queue_size=10)
        self.barrel_publisher = rospy.Publisher('barrels_data', String, queue_size=10)
        self.coins_publisher = rospy.Publisher('coins_data', String, queue_size=10)

        # --- SERVICES (SERVIDORES) ---
        rospy.Service('user_score', GetUserScore, self.handle_get_score)
        rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)

        # --- SUBSCRIBERS ---
        rospy.Subscriber("user_information", user_msg, self.callback_user_info)
        rospy.Subscriber("keyboard_control", String, self.callback_keyboard)

        # --- LOOP ---
        rospy.Timer(rospy.Duration(0.016), self.update_physics)
        rospy.loginfo("GAME_NODE started...")
        self.publish_game_state()
        rospy.spin()

    # -----------------------------------------------------------
    #                   SERVICE CALLBACKS
    # -----------------------------------------------------------
    def handle_get_score(self, req):
        """Devuelve la puntuación si el nombre coincide"""
        rospy.loginfo(f"Service Request: Score for {req.username}")
        if req.username == self.player_username:
            return GetUserScoreResponse(self.score)
        else:
            return GetUserScoreResponse(0)

    def handle_set_difficulty(self, req):
        """Cambia la dificultad SOLO si estamos en la fase WELCOME"""
        rospy.loginfo(f"Service Request: Set Difficulty to {req.change_difficulty}")
        
        # Regla del PDF: Solo en fase 1
        if self.current_state != "WELCOME":
            return SetGameDifficultyResponse(False, "Error: You can only change difficulty in the Start Screen!")

        diff = req.change_difficulty.lower()
        
        if diff == "easy":
            self.barrel_spawn_rate = 4.0 # Muy lento
            return SetGameDifficultyResponse(True, "Difficulty set to EASY")
        elif diff == "medium":
            self.barrel_spawn_rate = 2.0 # Normal
            return SetGameDifficultyResponse(True, "Difficulty set to MEDIUM")
        elif diff == "hard":
            self.barrel_spawn_rate = 1.0 # Muy rápido
            return SetGameDifficultyResponse(True, "Difficulty set to HARD")
        else:
            return SetGameDifficultyResponse(False, "Invalid difficulty. Use: easy, medium, hard")

    # -----------------------------------------------------------
    #                   HELPER: GET TILE TYPE
    # -----------------------------------------------------------
    def get_tile_at(self, x, y):
        col = int(x / self.block_width)
        row = int(y / self.block_height)
        if row < 0 or row >= len(self.level_map): return 0
        col = max(0, min(col, len(self.level_map[0]) - 1))
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
            if self.player_y < 100:
                self.score += 500
                self.Final()

    # -----------------------------------------------------------
    #                   GAME LOGIC
    # -----------------------------------------------------------
    def process_command(self, command):
        if command == "LEFT": self.player_x -= self.speed
        elif command == "RIGHT": self.player_x += self.speed

        screen_limit = 800 
        if self.player_x > screen_limit:
            self.player_x = 0
            safety = 0
            while self.get_tile_at(self.player_x, self.player_y + self.player_h + 2) == 0 and safety < 20:
                self.player_x += 20
                safety += 1
        elif self.player_x < 0:
            self.player_x = screen_limit - self.player_w
            safety = 0
            while self.get_tile_at(self.player_x, self.player_y + self.player_h + 2) == 0 and safety < 20:
                self.player_x -= 20
                safety += 1

        if self.is_on_ladder:
            if command == "UP": self.player_y -= self.speed
            elif command == "DOWN": self.player_y += self.speed
        else:
            if command == "UP" and self.is_on_ground:
                self.vel_y = self.jump_force
                self.is_on_ground = False 

    def update_physics(self, event):
        if self.current_state != "RUNNING": return
        self.update_player_physics()
        self.update_barrels()
        self.update_coins()
        self.publish_game_state()

    def update_player_physics(self):
        center_x = self.player_x + (self.player_w / 2)
        center_y = self.player_y + (self.player_h / 2)
        feet_y = self.player_y + self.player_h
        current_tile = self.get_tile_at(center_x, center_y)
        feet_tile = self.get_tile_at(center_x, feet_y)
        self.is_on_ladder = (current_tile == 2 or feet_tile == 2)

        if self.is_on_ladder: self.vel_y = 0 
        else:
            self.vel_y += self.gravity
            self.player_y += self.vel_y

        self.is_on_ground = False
        if not self.is_on_ladder and self.vel_y >= 0:
            tile_below = self.get_tile_at(center_x, self.player_y + self.player_h + 1)
            if tile_below == 1:
                row = int((self.player_y + self.player_h + 1) / self.block_height)
                self.player_y = (row * self.block_height) - self.player_h
                self.vel_y = 0
                self.is_on_ground = True
            if self.player_y >= 550:
                self.player_y = 550
                self.vel_y = 0
                self.is_on_ground = True

    def update_barrels(self):
        now = rospy.get_time()
        if now - self.last_barrel_time > self.barrel_spawn_rate:
            new_barrel = Barrel(380, 60)
            self.barrels.append(new_barrel)
            self.last_barrel_time = now

        barrels_data_str = ""
        active_barrels = []
        for b in self.barrels:
            b.speed_y += b.gravity
            b.x += b.speed_x
            b.y += b.speed_y
            
            center_bx = b.x + 10
            feet_by = b.y + 20
            tile_below = self.get_tile_at(center_bx, feet_by)
            
            if tile_below == 1: 
                row = int(feet_by / self.block_height)
                b.y = (row * self.block_height) - 20
                b.speed_y = 0
                if b.is_falling:
                    b.is_falling = False 
                    b.speed_x = random.choice([-3, 3])
            else:
                b.is_falling = True

            if abs(self.player_x - b.x) < 25 and abs(self.player_y - b.y) < 25:
                self.lives -= 1
                rospy.logwarn(f"¡GOLPE! Vidas: {self.lives}")
                self.player_x = 100
                self.player_y = 400
                if self.lives <= 0: self.Final()
                continue 

            if b.x > -20 and b.x < 820 and b.y < 650:
                active_barrels.append(b)
                barrels_data_str += f"{int(b.x)},{int(b.y)};"

        self.barrels = active_barrels
        self.barrel_publisher.publish(barrels_data_str)

    def update_coins(self):
        coins_data_str = ""
        for c in self.coins:
            if c.active:
                dx = abs(self.player_x - c.x)
                dy = abs(self.player_y - c.y)
                if dx < 30 and dy < 30:
                    self.score += 50 
                    c.active = False 
                    rospy.loginfo(f"Coin collected! Score: {self.score}")
                else:
                    coins_data_str += f"{c.x},{c.y};"
        self.coins_publisher.publish(coins_data_str)

    # -----------------------------------------------------------
    #                   PHASES
    # -----------------------------------------------------------
    def Welcome(self):
        self.current_state = "WELCOME"
        self.player_x = 80
        self.player_y = 500
        self.score = 0
        self.vel_y = 0
        self.lives = 3
        self.barrels = []
        for c in self.coins: c.active = True
        rospy.loginfo("--- WELCOME ---")
        self.publish_game_state()

    def Game(self):
        self.current_state = "RUNNING"
        self.score = 0
        self.lives = 3
        self.barrels = []
        for c in self.coins: c.active = True
        self.last_barrel_time = rospy.get_time()
        rospy.loginfo("--- GAME START ---")

    def Final(self):
        self.current_state = "GAME_OVER"
        rospy.loginfo(f"--- GAME OVER: Score {self.score} ---")
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
        msg.lives = self.lives 
        self.state_publisher.publish(msg)

if __name__ == '__main__':
    try:
        GameNode()
    except rospy.ROSInterruptException:
        pass
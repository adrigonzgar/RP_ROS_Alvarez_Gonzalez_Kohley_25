#!/usr/bin/env python3

#############################################
#                 GAME_NODE                 #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
import random 
import time
from project_game.msg import user_msg
from project_game.msg import game_state
from std_msgs.msg import String, Int64

# --- IMPORT SERVICES ---
from project_game.srv import GetUserScore, GetUserScoreResponse
from project_game.srv import SetGameDifficulty, SetGameDifficultyResponse

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

# --- CLASE HEART ---
class Heart:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 20
        self.height = 20
        self.creation_time = rospy.get_time()

class GameNode:
    def __init__(self):
        rospy.init_node('game_node', anonymous=True)

        # --- GAME VARIABLES ---
        self.player_name = ""
        self.player_username = "Unknown"
        self.player_age = 0

        # Mario Start Position
        self.player_x = 50
        self.player_y = 520 
        
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

        # --- BARRELS ---
        self.barrels = []
        self.last_barrel_time = 0
        self.barrel_spawn_rate = 2.0 

        # --- COINS ---
        self.coins = [
            Coin(100, 520), Coin(700, 520),
            Coin(150, 420), Coin(650, 420),
            Coin(250, 320), Coin(550, 320),
            Coin(400, 220),
            Coin(380, 120), Coin(420, 120)
        ]

        # --- HEARTS ---
        self.hearts = []
        self.last_heart_spawn_time = rospy.get_time()

        # --- MAP CONFIGURATION ---
        self.block_width = 40
        self.block_height = 25
        self.level_map = [
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,2,2,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,2,2,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0],
            [0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0],
            [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
            [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
            [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1] 
        ]

        # --- PUBLISHERS ---
        self.result_publisher = rospy.Publisher('result_information', Int64, queue_size=10)
        self.state_publisher = rospy.Publisher('game_state', game_state, queue_size=10)
        
        # Topics de objetos (Usamos String estándar, sin tocar .msg)
        self.barrel_publisher = rospy.Publisher('barrels_data', String, queue_size=10)
        self.coins_publisher = rospy.Publisher('coins_data', String, queue_size=10)
        self.hearts_publisher = rospy.Publisher('hearts_data', String, queue_size=10) # <--- NUEVO TOPIC

        # --- SERVICES ---
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

    # --- SERVICE CALLBACKS ---
    def handle_get_score(self, req):
        if req.username == self.player_username:
            return GetUserScoreResponse(self.score)
        else:
            return GetUserScoreResponse(0)

    def handle_set_difficulty(self, req):
        if self.current_state != "WELCOME":
            return SetGameDifficultyResponse(False, "Error: You can only change difficulty in the Start Screen!")

        diff = req.change_difficulty.lower()
        if diff == "easy":
            self.barrel_spawn_rate = 4.0 
            return SetGameDifficultyResponse(True, "Difficulty set to EASY")
        elif diff == "medium":
            self.barrel_spawn_rate = 2.0 
            return SetGameDifficultyResponse(True, "Difficulty set to MEDIUM")
        elif diff == "hard":
            self.barrel_spawn_rate = 1.0 
            return SetGameDifficultyResponse(True, "Difficulty set to HARD")
        else:
            return SetGameDifficultyResponse(False, "Invalid difficulty. Use: easy, medium, hard")

    # --- HELPER: GET TILE TYPE ---
    def get_tile_at(self, x, y):
        col = int(x / self.block_width)
        row = int(y / self.block_height)
        if row < 0 or row >= len(self.level_map): return 0
        col = max(0, min(col, len(self.level_map[0]) - 1))
        return self.level_map[row][col]

    # --- CALLBACKS ---
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
            
            # --- WIN CONDITION ---
            dist_to_crown = abs(self.player_x - 380) + abs(self.player_y - 50)
            if dist_to_crown < 50:
                self.score += 1000
                self.Final()

    # --- GAME LOGIC ---
    def process_command(self, command):
        if command == "LEFT": self.player_x -= self.speed
        elif command == "RIGHT": self.player_x += self.speed

        screen_limit = 800 
        if self.player_x > screen_limit: self.player_x = 0
        elif self.player_x < 0: self.player_x = screen_limit - self.player_w

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
        self.update_hearts() # <--- Llamada a función de corazones
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
            
            if self.player_y >= 520:
                self.player_y = 520
                self.vel_y = 0
                self.is_on_ground = True

    def update_barrels(self):
        now = rospy.get_time()
        if now - self.last_barrel_time > self.barrel_spawn_rate:
            new_barrel = Barrel(400, 150)
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
                rospy.logwarn(f"HIT! Lives: {self.lives}")
                self.player_x = 50 
                self.player_y = 520
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
    
    # --- LOGICA CORAZONES (Ahora sincronizada) ---
    def update_hearts(self):
        now = rospy.get_time()
        
        # 1. Spawn logic (cada 8-15 segundos)
        if now - self.last_heart_spawn_time > random.randint(8, 15):
            accessible_y = [520, 420, 320, 220, 120] 
            hy = random.choice(accessible_y)
            hx = random.randint(50, 750)
            
            self.hearts.append(Heart(hx, hy))
            self.last_heart_spawn_time = now
            rospy.loginfo(f"Heart Spawned at {hx},{hy}")

        hearts_data_str = ""
        for h in self.hearts[:]:
            # Borrar si caduca
            if now - h.creation_time > 10.0:
                self.hearts.remove(h)
                continue
            
            # 2. Colisión (La lógica la lleva este nodo)
            dx = abs(self.player_x - h.x)
            dy = abs(self.player_y - h.y)
            
            if dx < 30 and dy < 30:
                if self.lives < 3:
                    self.lives += 1
                    rospy.loginfo(f"VIDA RECUPERADA! Total: {self.lives}")
                    self.hearts.remove(h)
                else:
                    # Si vidas a tope, puntos
                    self.score += 50
                    self.hearts.remove(h)
            else:
                # Si no colisiona, lo añadimos al string para el visualizador
                hearts_data_str += f"{h.x},{h.y};"
        
        # Publicamos la posición REAL de los corazones
        self.hearts_publisher.publish(hearts_data_str)

    # --- PHASES ---
    def Welcome(self):
        self.current_state = "WELCOME"
        self.player_x = 50
        self.player_y = 520
        self.score = 0
        self.vel_y = 0
        self.lives = 3
        self.barrels = []
        self.hearts = [] # Reset hearts
        for c in self.coins: c.active = True
        rospy.loginfo("--- WELCOME ---")
        self.publish_game_state()

    def Game(self):
        self.current_state = "RUNNING"
        self.score = 0
        self.lives = 3
        self.barrels = []
        self.hearts = [] # Reset hearts
        for c in self.coins: c.active = True
        self.last_barrel_time = rospy.get_time()
        self.last_heart_spawn_time = rospy.get_time()
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
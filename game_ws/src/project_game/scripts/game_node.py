#!/usr/bin/env python3

#############################################
#                 GAME_NODE                 #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
import random 
import copy 
from project_game.msg import user_msg
from project_game.msg import game_state
from std_msgs.msg import String, Int64

# --- IMPORT SERVICES ---
from project_game.srv import GetUserScore, GetUserScoreResponse
from project_game.srv import SetGameDifficulty, SetGameDifficultyResponse

# --- CLASSES ---
class Barrel:
    def __init__(self, x, y, speed_range, gravity_val):
        self.x = x
        self.y = y
        self.speed_x = random.choice(speed_range)
        self.speed_y = 0
        self.gravity = gravity_val
        self.is_falling = True 

class Coin:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 15
        self.height = 15
        self.active = True

class Heart:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 20
        self.height = 20
        self.creation_time = rospy.get_time()

class MovingPlatform:
    def __init__(self, x, y, width, height, range_x):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.start_x = x
        self.range_x = range_x
        self.speed = 2
        self.direction = 1 
    
    def update(self):
        self.x += self.speed * self.direction
        if self.x > self.start_x + self.range_x:
            self.direction = -1
        elif self.x < self.start_x - self.range_x:
            self.direction = 1

class GameNode:
    def __init__(self):
        rospy.init_node('game_node', anonymous=True)

        # --- INITIALIZE ROS PARAMETERS ---
        # 1. user_name (String)
        rospy.set_param('user_name', 'Unknown')
        
        # 2. change_player_color (Int64) - Default 1 (Red)
        if not rospy.has_param('change_player_color'):
            rospy.set_param('change_player_color', 1)
            
        # 3. screen_param (String) - Default phase1
        rospy.set_param('screen_param', 'phase1')

        # --- GAME VARIABLES ---
        self.player_name = ""
        self.player_username = "Unknown"
        self.player_age = 0

        self.player_x = 50
        self.player_y = 520 
        
        self.score = 0
        self.lives = 3
        self.current_state = "WELCOME"
        
        self.player_speed = 10

        # --- PHYSICS VARIABLES ---
        self.vel_y = 0          
        self.gravity = 1.8      
        self.jump_force = -17
        self.speed = 10  
        
        self.player_w = 30
        self.player_h = 30
        self.is_on_ground = True
        self.is_on_ladder = False

        # --- DIFFICULTY VARIABLES ---
        self.current_difficulty_level = "medium"
        self.difficulty_selected = False  # Track if user explicitly selected a difficulty
        self.barrel_spawn_rate = 2.0 
        self.barrel_gravity_val = 0.5
        self.barrel_speed_range = [-3, 3]
        self.start_lives = 3 

        # --- GAME OBJECTS ---
        self.barrels = []
        self.last_barrel_time = 0
        self.moving_platforms = [] 
        
        self.coins = [
            Coin(100, 520), Coin(700, 520),
            Coin(150, 420), Coin(650, 420),
            Coin(250, 320), Coin(550, 320),
            Coin(400, 220),
            Coin(380, 120), Coin(420, 120)
        ]

        self.hearts = []
        self.last_heart_spawn_time = rospy.get_time()

        # --- MAP CONFIGURATION ---
        self.block_width = 40
        self.block_height = 25
        
        self.original_level_map = [
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
        
        self.setup_level(self.current_difficulty_level)

        # --- PUBLISHERS ---
        self.result_publisher = rospy.Publisher('result_information', Int64, queue_size=10)
        self.state_publisher = rospy.Publisher('game_state', game_state, queue_size=10)
        # Give publisher time to establish connections
        rospy.sleep(0.1)
        
        self.barrel_publisher = rospy.Publisher('barrels_data', String, queue_size=10)
        self.coins_publisher = rospy.Publisher('coins_data', String, queue_size=10)
        self.hearts_publisher = rospy.Publisher('hearts_data', String, queue_size=10) 
        self.platforms_publisher = rospy.Publisher('moving_platforms_data', String, queue_size=10)

        # --- SERVICES ---
        self.service_score = rospy.Service('user_score', GetUserScore, self.handle_get_user_score)
        self.service_difficulty = rospy.Service('difficulty', SetGameDifficulty, self.handle_set_difficulty)

        # --- SUBSCRIBERS ---
        rospy.Subscriber("user_information", user_msg, self.user_callback)
        rospy.Subscriber("keyboard_control", String, self.callback_keyboard)

        # --- LOOP ---
        rospy.Timer(rospy.Duration(0.016), self.update_physics)
        rospy.loginfo("GAME_NODE started...")
        self.publish_game_state()
        rospy.spin()

    def handle_get_score(self, req):
        if req.username == self.player_username:
            return GetUserScoreResponse(self.score)
        else:
            return GetUserScoreResponse(0)

    # --- SETUP LEVEL ---
    def setup_level(self, diff):
        self.level_map = copy.deepcopy(self.original_level_map)
        self.moving_platforms = []
        
        if diff == "easy":
            self.barrel_spawn_rate = 3.5      
            self.barrel_gravity_val = 0.3     
            self.barrel_speed_range = [-2, 2] 
            self.crown = {'x': 390, 'y': 45, 'width': 30, 'height': 30}
            
        elif diff == "medium":
            self.barrel_spawn_rate = 2.0      
            self.barrel_gravity_val = 0.5     
            self.barrel_speed_range = [-3, 3] 
            self.moving_platforms.append(MovingPlatform(80, 14 * 25, 240, 25, 40))  
            self.moving_platforms.append(MovingPlatform(480, 14 * 25, 240, 25, 40)) 
            self.crown = {'x': 390, 'y': 45, 'width': 30, 'height': 30}
            
        elif diff == "hard":
            self.barrel_spawn_rate = 0.8      
            self.barrel_gravity_val = 0.9     
            self.barrel_speed_range = [-5, 5] 
            self.crown = {'x': 390, 'y': 45, 'width': 30, 'height': 30}
            self.moving_platforms.append(MovingPlatform(80, 14 * 25, 240, 25, 40))
            self.moving_platforms.append(MovingPlatform(480, 14 * 25, 240, 25, 40))
            self.moving_platforms.append(MovingPlatform(40, 18 * 25, 320, 25, 30))
            self.moving_platforms.append(MovingPlatform(440, 18 * 25, 320, 25, 30))

    def get_tile_at(self, x, y):
        col = int(x / self.block_width)
        row = int(y / self.block_height)
        if row < 0 or row >= len(self.level_map): return 0
        col = max(0, min(col, len(self.level_map[0]) - 1))
        return self.level_map[row][col]

    def user_callback(self, msg):
        if self.current_state == "RUNNING":
            return
        # --------------------------

        rospy.loginfo(f"!!! USER INFO RECEIVED: {msg.username} !!!")
        self.username = msg.username
        self.player_name = msg.name # Guardamos nombre real

        # 1. Guardar nombre de usuario en el parámetro para Pygame
        rospy.set_param('user_name', self.player_name)
        
        # 2. Ir a la pantalla de bienvenida (no iniciar el juego automáticamente)
        self.Welcome()
        
        
    # --- AÑADIR ESTOS MÉTODOS ---
    
    def handle_get_user_score(self, req):
        """ Devuelve la puntuación actual al ResultNode """
        rospy.loginfo(f"Service Request: Score requested for {req.username}")
        # Retorna la respuesta definida en el .srv (int64 score)
        return GetUserScoreResponse(self.score)

    def handle_set_difficulty(self, req):
        """ Cambia la dificultad SOLO si estamos en la fase de bienvenida """
        rospy.loginfo(f"Service Request: Change difficulty to {req.change_difficulty}")
        
        # Lógica: Solo permitimos cambiar si estamos en "WELCOME" (Fase 1)
        if self.current_state == "WELCOME":
            if req.change_difficulty in ["easy", "medium", "hard"]:
                self.current_difficulty_level = req.change_difficulty
                # Llamamos a setup_level para reajustar plataformas/barriles
                self.setup_level(self.current_difficulty_level)
                self.difficulty_selected = True
                
                # Republish game_state immediately so pygame_node updates the highlight
                self.publish_game_state()
                rospy.sleep(0.01)
                
                return SetGameDifficultyResponse(True, f"Difficulty set to {req.change_difficulty}")
            else:
                return SetGameDifficultyResponse(False, "Invalid difficulty name")
        else:
            return SetGameDifficultyResponse(False, "Cannot change difficulty: Not in Phase 1")
        
    # ----------------------------
    
    def try_jump(self):
        # Solo saltamos si estamos tocando el suelo
        if self.is_on_ground:
            self.player_vel_y = self.jump_strength
            self.is_on_ground = False
            # Opcional: log para depurar
            rospy.loginfo("JUMPING!")

    def callback_keyboard(self, msg):
        cmd = msg.data
        
        # 1. JUGANDO (RUNNING)
        if self.current_state == "RUNNING":
            self.process_command(cmd)
            self.publish_game_state()
            
        # 2. PANTALLA DE TÍTULO (WELCOME)
        elif self.current_state == "WELCOME": 
            # Note: Difficulty changes (E/M/H) are handled via ROS service, not through keyboard topic
            # They are handled in handle_set_difficulty() which is called by control_node service client
            
            # Cambios de color
            if cmd == "1": 
                rospy.set_param("change_player_color", 1)
                self.publish_game_state()
            elif cmd == "2": 
                rospy.set_param("change_player_color", 2)
                self.publish_game_state()
            elif cmd == "3": 
                rospy.set_param("change_player_color", 3)
                self.publish_game_state()
            
            
            elif cmd == "ENTER":
                # Comprobamos si ya tenemos un nombre guardado de antes
                if self.player_name != "":
                    if self.difficulty_selected:
                        rospy.loginfo("Quick Restart initiated with Enter!")
                        self.Game() # <--- ¡EMPEZAR PARTIDA!
                    else:
                        rospy.logwarn("Please select a difficulty (E/M/H) before starting the game!")
                else:
                    rospy.logwarn("No player data found. Run info_user node first.")
            # ---------------------------------

        # 3. PANTALLA FINAL (GAME OVER / VICTORY)
        elif self.current_state in ["GAME_OVER", "VICTORY"]:
            # Volver al menú principal
            if cmd == "ENTER":
                rospy.loginfo("Returning to Main Menu...")
                self.Welcome()

        # 4. SALIDA GLOBAL
        if cmd == "q": 
            rospy.signal_shutdown("Quit requested by user")
        
        
        
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
        self.check_victory() # ¡IMPORTANTE: Verificar victoria antes de mover!
        self.update_player_physics()
        self.update_barrels()
        self.update_coins()
        self.update_hearts() 
        self.update_platforms() 
        self.publish_game_state()

    def update_platforms(self):
        platforms_str = ""
        for p in self.moving_platforms:
            p.update()
            platforms_str += f"{int(p.x)},{int(p.y)},{p.width},{p.height};"
        self.platforms_publisher.publish(platforms_str)

    def update_player_physics(self):
        center_x = self.player_x + (self.player_w / 2)
        center_y = self.player_y + (self.player_h / 2)
        feet_y = self.player_y + self.player_h
        current_tile = self.get_tile_at(center_x, center_y)
        feet_tile = self.get_tile_at(center_x, feet_y)
        self.is_on_ladder = (current_tile == 2 or feet_tile == 2)

        if self.is_on_ladder: self.vel_y = 0 
        else:
            # Aplicar gravedad
            self.vel_y += self.gravity
            self.player_y += self.vel_y

            # --- SALTO RELATIVO: limitar altura según plataforma superior ---
            if self.vel_y < 0:  # Solo cuando el jugador SUBE
                upper_limit = self.get_upper_limit(self.player_y)

                if upper_limit is not None:
                    # margen para no quedarse pegado al bloque
                    max_jump_y = upper_limit + 10  

                    if self.player_y < max_jump_y:
                        self.player_y = max_jump_y
                        self.vel_y = 0


        self.is_on_ground = False
        if self.vel_y >= 0: 
            for p in self.moving_platforms:
                if center_x > p.x and center_x < p.x + p.width:
                    if feet_y >= p.y and feet_y <= p.y + 20: 
                        self.player_y = p.y - self.player_h
                        self.vel_y = 0
                        self.is_on_ground = True
                        self.player_x += p.speed * p.direction 

        if not self.is_on_ground and not self.is_on_ladder and self.vel_y >= 0:
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
            new_barrel = Barrel(400, 150, self.barrel_speed_range, self.barrel_gravity_val)
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
            landed = False
            
            if tile_below == 1: 
                row = int(feet_by / self.block_height)
                b.y = (row * self.block_height) - 20
                b.speed_y = 0
                landed = True

            if not landed:
                for p in self.moving_platforms:
                     if center_bx > p.x and center_bx < p.x + p.width:
                        if feet_by >= p.y and feet_by <= p.y + 20:
                            b.y = p.y - 20
                            b.speed_y = 0
                            landed = True
                            b.x += p.speed * p.direction 

            if landed:
                if b.is_falling:
                    b.is_falling = False 
                    b.speed_x = random.choice(self.barrel_speed_range)
            else:
                b.is_falling = True

            if abs(self.player_x - b.x) < 20 and abs(self.player_y - b.y) < 20:
                self.lives -= 1
                rospy.logwarn(f"HIT! Lives: {self.lives}")
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
                else:
                    coins_data_str += f"{c.x},{c.y};"
        self.coins_publisher.publish(coins_data_str)
    
    def update_hearts(self):
        now = rospy.get_time()
        if now - self.last_heart_spawn_time > random.randint(8, 15):
            accessible_y = [520, 420, 320, 220, 120] 
            hy = random.choice(accessible_y)
            hx = random.randint(50, 750)
            self.hearts.append(Heart(hx, hy))
            self.last_heart_spawn_time = now

        hearts_data_str = ""
        for h in self.hearts[:]:
            if now - h.creation_time > 10.0:
                self.hearts.remove(h)
                continue
            dx = abs(self.player_x - h.x)
            dy = abs(self.player_y - h.y)
            if dx < 30 and dy < 30:
                if self.lives < 3:
                    self.lives += 1
                    self.hearts.remove(h)
                else:
                    self.score += 50
                    self.hearts.remove(h)
            else:
                hearts_data_str += f"{h.x},{h.y};"
        self.hearts_publisher.publish(hearts_data_str)

    def Welcome(self):
        self.current_state = "WELCOME"
        # --- UPDATE PARAM: screen_param ---
        rospy.set_param('screen_param', 'phase1')
        
        self.player_x = 50
        self.player_y = 520
        self.score = 0
        self.vel_y = 0
        self.lives = self.start_lives 
        self.barrels = []
        self.hearts = []
        self.difficulty_selected = False  # Reset difficulty selection when returning to welcome
        
        for c in self.coins: c.active = True
        rospy.loginfo("--- WELCOME ---")
        self.publish_game_state()

    def Game(self):
        self.current_state = "RUNNING"
        # --- UPDATE PARAM: screen_param ---
        rospy.set_param('screen_param', 'phase2')
        
        self.score = 0
        self.lives = self.start_lives
        self.barrels = []
        self.hearts = [] 
        self.setup_level(self.current_difficulty_level)
        for c in self.coins: c.active = True
        self.last_barrel_time = rospy.get_time()
        self.last_heart_spawn_time = rospy.get_time()
        rospy.loginfo(f"--- GAME START ---")

    def Victory(self):
        self.current_state = "VICTORY"
        # --- UPDATE PARAM: screen_param ---
        rospy.set_param('screen_param', 'phase3')
        
        rospy.loginfo(f"--- VICTORY! Score {self.score} ---")
        msg = Int64()
        msg.data = self.score
        self.result_publisher.publish(msg)
        self.publish_game_state()
    
    def check_victory(self):
        # 1. Definir coordenadas
        if self.current_difficulty_level == "easy":
            # --- CAMBIO: Arriba y centrado ---
            target_x = 390  # Centro horizontal (aprox)
            target_y = 45   # Arriba del todo
            # ------------------------------------------------------------
        elif self.current_difficulty_level == "medium":
            target_x = 390
            target_y = 45
        else: # hard
            target_x = 390
            target_y = 45

        # 2. Calcular distancias
        dx = abs(self.player_x - target_x)
        dy = abs(self.player_y - target_y)

        # 3. CONDICIÓN DE VICTORIA (Mantenemos margen amplio)
        if dx < 45 and dy < 45:
            rospy.loginfo("!!! VICTORIA DETECTADA !!!")
            self.Victory()
        
        # 4. CHIVATO (Sigue siendo útil)
        else:
            if dx < 150: 
                rospy.loginfo_throttle(0.5, 
                    f"DEBUG CORONA -> Jugador: ({int(self.player_x)}, {int(self.player_y)}) | Corona: ({target_x}, {target_y}) | Dist: {int(dx)}, {int(dy)}"
                )
        
    def Final(self):
        self.current_state = "GAME_OVER"
        # --- UPDATE PARAM: screen_param ---
        rospy.set_param('screen_param', 'phase3')
        
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
        msg.difficulty = str(self.current_difficulty_level)  # Ensure it's a string
        msg.color = rospy.get_param('change_player_color', 1)
        rospy.loginfo(f"Publishing game_state: difficulty={msg.difficulty}, state={msg.state}")
        self.state_publisher.publish(msg)

    def get_upper_limit(self, y):
        """ Devuelve la altura máxima a la que el jugador puede subir según la plataforma superior """
        current_row = int(y / self.block_height)

        for row in range(current_row - 1, -1, -1):
            # ¿Hay al menos un bloque sólido (1) en esta fila?
            if 1 in self.level_map[row]:
                return row * self.block_height

        return None  # No hay techo → salto libre


if __name__ == '__main__':
    try:
        GameNode()
    except rospy.ROSInterruptException:
        pass
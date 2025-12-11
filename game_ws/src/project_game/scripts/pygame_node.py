#!/usr/bin/env python3

#############################################
#                PYGAME_NODE                #
#############################################
# Andrea Álvarez Campos    100496951
# Adrián Gonzalez García   100523018
# Lucas Kohley Aguilar     100497018

import rospy
import pygame
import os 
import copy
import math
from project_game.msg import game_state
from std_msgs.msg import String
from project_game.srv import SetGameDifficulty

# --- VICTORY SCREEN CLASS ---
class VictoryScreen:
    def __init__(self, screen_width, screen_height):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.animation_frame = 0
        self.firework_particles = []
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.WHITE = (255, 255, 255)
        
    def create_firework(self):
        x = pygame.math.Vector2(
            self.screen_width // 2 + (200 * math.cos(self.animation_frame * 0.1)),
            self.screen_height // 2 + (200 * math.sin(self.animation_frame * 0.1))
        )
        for _ in range(20):
            angle = (360 / 20) * _ + self.animation_frame
            speed = 3
            velocity = pygame.math.Vector2(
                speed * math.cos(math.radians(angle)),
                speed * math.sin(math.radians(angle))
            )
            color = [self.RED, self.YELLOW, self.GREEN, self.BLUE, (255, 0, 255)][_ % 5]
            self.firework_particles.append({
                'pos': pygame.math.Vector2(x),
                'vel': velocity,
                'color': color,
                'life': 60
            })
    
    def update_fireworks(self):
        for particle in self.firework_particles[:]:
            particle['pos'] += particle['vel']
            particle['vel'].y += 0.1
            particle['life'] -= 1
            if particle['life'] <= 0:
                self.firework_particles.remove(particle)
    
    def draw_fireworks(self, screen):
        for particle in self.firework_particles:
            alpha = int(255 * (particle['life'] / 60))
            size = max(1, int(4 * (particle['life'] / 60)))
            pygame.draw.circle(screen, particle['color'], 
                             (int(particle['pos'].x), int(particle['pos'].y)), size)
    
    def draw(self, screen, score):
        self.animation_frame += 1
        if self.animation_frame % 30 == 0:
            self.create_firework()
        self.update_fireworks()
        
        for y in range(self.screen_height):
            color_value = int(20 + (y / self.screen_height) * 40)
            pygame.draw.line(screen, (0, 0, color_value), (0, y), (self.screen_width, y))
        
        self.draw_fireworks(screen)
        
        font_huge = pygame.font.SysFont("Arial Black", 96)
        font_large = pygame.font.SysFont("Arial Black", 64)
        font_medium = pygame.font.SysFont("Arial", 48)
        font_small = pygame.font.SysFont("Arial", 36)
        
        bounce = int(10 * math.sin(self.animation_frame * 0.1))
        
        victory_text = font_huge.render("VICTORY!", True, self.YELLOW)
        victory_rect = victory_text.get_rect(center=(self.screen_width // 2, 100 + bounce))
        screen.blit(victory_text, victory_rect)
        
        subtitle_text = font_large.render("You reached the Crown!", True, self.WHITE)
        subtitle_rect = subtitle_text.get_rect(center=(self.screen_width // 2, 200))
        screen.blit(subtitle_text, subtitle_rect)
        
        score_text = font_medium.render(f"FINAL SCORE: {score}", True, self.GREEN)
        score_rect = score_text.get_rect(center=(self.screen_width // 2, 350))
        screen.blit(score_text, score_rect)
        
        instructions = font_small.render("Press ENTER to return to menu", True, self.WHITE)
        instructions_rect = instructions.get_rect(center=(self.screen_width // 2, self.screen_height - 60))
        screen.blit(instructions, instructions_rect)

class PygameNode:
    def __init__(self):
        rospy.init_node("pygame_node")
        rospy.loginfo("PYGAME_NODE started.")

        pygame.init()
        self.screen_width = 800
        self.screen_height = 600
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Donkey Kong ROS")

        self.clock = pygame.time.Clock()
        self.victory_screen = VictoryScreen(self.screen_width, self.screen_height)

        # --- Variables ---
        self.state = "WELCOME"
        self.player_x = 100
        self.player_y = 300
        self.score = 0
        self.lives = 3
        self.selected_difficulty = None
        self.username = "Player"

        self.barrels_data = []
        self.coins_data = [] 
        self.hearts_data = [] 
        self.platforms_data = [] 
        
        # --- COLORS ---
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0) 
        self.BLUE_PLATFORM = (0, 0, 255)
        self.GREEN_PLATFORM = (0, 200, 0)
        self.YELLOW_LADDER = (255, 255, 0)
        self.PINK = (255, 105, 180)
        
        # Sprites Defaults
        self.M_SHIRT = (255, 0, 0)      
        self.M_OVERALLS = (0, 0, 255)   
        self.M_SKIN = (255, 200, 150)   
        self.M_SHOES = (139, 69, 19)    
        self.DK_BROWN = (139, 69, 19)
        self.DK_PEACH = (222, 184, 135)
        self.DK_EYES = (0, 0, 0)
        self.BARREL_COLOR = (160, 82, 45) 
        self.COIN_COLOR = (255, 215, 0) 
        self.CROWN_COLOR = (255, 223, 0)

        self.background_image = self.load_resources()
        self.font_welcome = pygame.font.SysFont("Arial Black", 70, bold=True)
        self.font_gameover = pygame.font.SysFont("Arial Black", 80, bold=True)
        self.font_info = pygame.font.SysFont("Arial", 30)

        self.block_width = 40
        self.block_height = 25

        # --- MAP CONFIGURATION ---
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
        self.level_map = copy.deepcopy(self.original_level_map)

        # Subscribers
        rospy.Subscriber("game_state", game_state, self.callback_game_state)
        rospy.Subscriber("barrels_data", String, self.callback_barrels)
        rospy.Subscriber("coins_data", String, self.callback_coins)
        rospy.Subscriber("hearts_data", String, self.callback_hearts)
        rospy.Subscriber("moving_platforms_data", String, self.callback_platforms)
        
        # Publisher
        self.start_pub = rospy.Publisher("keyboard_control", String, queue_size=1)

        self.run()

    def load_resources(self):
        image_name = "Pantalla_Carga_DK.jpg"
        script_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(script_dir, image_name)
        if os.path.exists(image_path):
            try:
                img = pygame.image.load(image_path)
                return pygame.transform.scale(img, (self.screen_width, self.screen_height))
            except: return None
        return None

    def callback_game_state(self, msg):
        self.state = msg.state
        self.player_x = msg.player_x
        self.player_y = msg.player_y
        self.score = msg.score
        if hasattr(msg, 'lives'): self.lives = msg.lives
        if hasattr(msg, 'difficulty') and msg.difficulty:
            # Si la dificultad ha cambiado (o es la primera vez), actualizamos el dibujo del mapa
            if self.selected_difficulty != msg.difficulty:
                self.selected_difficulty = msg.difficulty
                
                # Reiniciamos al mapa original
                self.level_map = copy.deepcopy(self.original_level_map)
                
                # APLICAMOS LA OPCIÓN NUCLEAR (Igual que en game_node)
                if self.selected_difficulty == "medium":
                    # Borrar toda la fila 14
                    for c in range(20): self.level_map[14][c] = 0
                    
                elif self.selected_difficulty == "hard":
                    # Borrar filas 14 y 18
                    for c in range(20): self.level_map[14][c] = 0
                    for c in range(20): self.level_map[18][c] = 0

    def callback_barrels(self, msg):
        raw_data = msg.data
        new_barrels = []
        if raw_data:
            for item in raw_data.split(';'):
                if ',' in item:
                    try:
                        x, y = map(int, item.split(','))
                        new_barrels.append((x, y))
                    except: pass
        self.barrels_data = new_barrels

    def callback_coins(self, msg):
        raw_data = msg.data
        new_coins = []
        if raw_data:
            for item in raw_data.split(';'):
                if ',' in item:
                    try:
                        x, y = map(int, item.split(','))
                        new_coins.append((x, y))
                    except: pass
        self.coins_data = new_coins

    def callback_hearts(self, msg):
        raw_data = msg.data
        new_hearts = []
        if raw_data:
            for item in raw_data.split(';'):
                if ',' in item:
                    try:
                        x, y = map(int, item.split(','))
                        new_hearts.append((x, y))
                    except: pass
        self.hearts_data = new_hearts

    def callback_platforms(self, msg):
        raw_data = msg.data
        new_platforms = []
        if raw_data:
            for item in raw_data.split(';'):
                if ',' in item:
                    try:
                        x, y, w, h = map(int, item.split(','))
                        new_platforms.append((x, y, w, h))
                    except: pass
        self.platforms_data = new_platforms

    # --- SET DIFFICULTY (AND SYNC MAP) ---
    def set_difficulty(self, level):
        try:
            rospy.wait_for_service('difficulty', timeout=2)
            set_diff = rospy.ServiceProxy('difficulty', SetGameDifficulty)
            resp = set_diff(level)
            
            # --- SYNC VISUAL MAP WITH GAME_NODE ---
            self.level_map = copy.deepcopy(self.original_level_map)
            
            if level == "medium":
                for c in range(2, 8): self.level_map[14][c] = 0
                for c in range(12, 18): self.level_map[14][c] = 0
            
            elif level == "hard":
                for c in range(2, 8): self.level_map[14][c] = 0
                for c in range(12, 18): self.level_map[14][c] = 0
                for c in range(1, 9): self.level_map[18][c] = 0
                for c in range(11, 19): self.level_map[18][c] = 0
                
            rospy.loginfo(resp.message)
        except Exception as e:
            rospy.logerr(f"ERROR setting difficulty: {e}")

    # --- DRAWING ---
    def draw_map(self):
        for row_index, row in enumerate(self.level_map):
            for col_index, tile in enumerate(row):
                x = col_index * self.block_width
                y = row_index * self.block_height
                if tile == 1:
                    pygame.draw.rect(self.screen, self.BLUE_PLATFORM, (x, y, self.block_width, self.block_height))
                    for i in range(x, x + self.block_width, 10):
                         pygame.draw.line(self.screen, self.BLACK, (i, y), (i, y + self.block_height), 2)
                    pygame.draw.line(self.screen, self.BLACK, (x, y), (x + self.block_width, y), 2)
                    pygame.draw.line(self.screen, self.BLACK, (x, y + self.block_height - 2), (x + self.block_width, y + self.block_height - 2), 2)
                elif tile == 2:
                    pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 10, y, 5, self.block_height))
                    pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 25, y, 5, self.block_height))
                    for i in range(0, self.block_height, 5):
                        pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 10, y + i, 20, 2))
    
    def draw_moving_platforms(self):
        for (x, y, w, h) in self.platforms_data:
            pygame.draw.rect(self.screen, self.GREEN_PLATFORM, (x, y, w, h))
            for i in range(x, x + w, 20):
                 pygame.draw.line(self.screen, self.BLACK, (i, y), (i, y + h), 2)
            pygame.draw.rect(self.screen, self.BLACK, (x, y, w, h), 2)

    def draw_player_original(self):
        x = self.player_x
        y = self.player_y
        
        # --- COLOR LOGIC FROM PARAM ---
        color_choice = rospy.get_param('change_player_color', 1)
        
        if color_choice == 1: # Red (Mario)
            self.M_SHIRT = (255, 0, 0)
            self.M_OVERALLS = (0, 0, 255)
        elif color_choice == 2: # Purple (Waluigi)
            self.M_SHIRT = (128, 0, 128)
            self.M_OVERALLS = (50, 50, 50)
        elif color_choice == 3: # Blue (Luigi/Blue)
            self.M_SHIRT = (0, 0, 255)
            self.M_OVERALLS = (0, 150, 0)
        
        pygame.draw.rect(self.screen, self.M_SKIN, (x + 4, y, 16, 10))
        pygame.draw.rect(self.screen, self.M_SHIRT, (x, y, 24, 4))
        pygame.draw.rect(self.screen, self.M_SHIRT, (x + 4, y - 2, 16, 4)) 
        pygame.draw.rect(self.screen, self.M_SHIRT, (x + 2, y + 10, 20, 10))
        pygame.draw.rect(self.screen, self.M_OVERALLS, (x + 4, y + 16, 16, 10))
        pygame.draw.rect(self.screen, self.M_OVERALLS, (x + 2, y + 26, 6, 4)) 
        pygame.draw.rect(self.screen, self.M_OVERALLS, (x + 16, y + 26, 6, 4)) 
        pygame.draw.rect(self.screen, self.M_SHOES, (x, y + 30, 8, 4))
        pygame.draw.rect(self.screen, self.M_SHOES, (x + 16, y + 30, 8, 4))
        pygame.draw.rect(self.screen, self.M_SKIN, (x - 2, y + 14, 4, 4))
        pygame.draw.rect(self.screen, self.M_SKIN, (x + 22, y + 14, 4, 4))

    def draw_donkey_kong(self):
        dk_x = 370  
        dk_y = 90    
        pygame.draw.rect(self.screen, self.DK_BROWN, (dk_x, dk_y + 20, 60, 40))
        pygame.draw.rect(self.screen, self.DK_BROWN, (dk_x + 10, dk_y, 40, 30))
        pygame.draw.rect(self.screen, self.DK_PEACH, (dk_x + 15, dk_y + 5, 30, 20))
        pygame.draw.rect(self.screen, self.DK_EYES, (dk_x + 20, dk_y + 10, 4, 4))
        pygame.draw.rect(self.screen, self.DK_EYES, (dk_x + 30, dk_y + 10, 4, 4))
        pygame.draw.rect(self.screen, self.DK_BROWN, (dk_x - 10, dk_y + 25, 15, 25))
        pygame.draw.rect(self.screen, self.DK_BROWN, (dk_x + 55, dk_y + 25, 15, 25))

    def draw_crown(self):
        cx = 390
        cy = 45
        pygame.draw.rect(self.screen, self.CROWN_COLOR, (cx, cy, 30, 20))
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx, cy), (cx - 5, cy - 10), (cx + 10, cy)])
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx + 10, cy), (cx + 15, cy - 15), (cx + 20, cy)])
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx + 20, cy), (cx + 35, cy - 10), (cx + 30, cy)])
        pygame.draw.circle(self.screen, self.RED, (cx + 15, cy + 10), 5)

    def draw_barrels(self):
        for (bx, by) in self.barrels_data:
            center_x, center_y = bx + 10, by + 10
            radius = 10
            pygame.draw.circle(self.screen, self.BARREL_COLOR, (center_x, center_y), radius)
            pygame.draw.line(self.screen, self.BLACK, (bx, center_y - 5), (bx + 20, center_y - 5), 2)
            pygame.draw.line(self.screen, self.BLACK, (bx, center_y + 5), (bx + 20, center_y + 5), 2)
            pygame.draw.line(self.screen, self.BLACK, (center_x, center_y - 5), (center_x, center_y + 5), 2)

    def draw_coins(self):
        for (cx, cy) in self.coins_data:
            pygame.draw.circle(self.screen, self.COIN_COLOR, (cx + 15, cy + 15), 8)

    def draw_hearts(self):
        for (hx, hy) in self.hearts_data:
            pygame.draw.circle(self.screen, self.PINK, (hx + 5, hy + 5), 5)
            pygame.draw.circle(self.screen, self.PINK, (hx + 15, hy + 5), 5)
            pygame.draw.polygon(self.screen, self.PINK, [(hx, hy + 6), (hx + 20, hy + 6), (hx + 10, hy + 20)])

    def draw_ui(self):
        font = pygame.font.SysFont("Arial", 24)
        score_text = font.render(f"Score: {self.score}", True, self.WHITE)
        lives_text = font.render(f"Lives: {self.lives}", True, self.WHITE)
        self.screen.blit(score_text, (10, 10))
        self.screen.blit(lives_text, (700, 10))

    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Window closed")
                # Keyboard input disabled - all input now comes from control_node via ROS topic

            self.screen.fill(self.BLACK)

            if self.state == "WELCOME":
                if self.background_image: self.screen.blit(self.background_image, (0, 0))
                welcome_text = self.font_welcome.render("WELCOME", True, self.YELLOW)
                welcome_border = self.font_welcome.render("WELCOME", True, self.BLACK)
                welcome_rect = welcome_text.get_rect(center=(self.screen_width/2, 80))
                self.screen.blit(welcome_border, (welcome_rect.x + 3, welcome_rect.y + 3))
                self.screen.blit(welcome_text, welcome_rect)
                
                # DIFFICULTY TEXT
                easy_color = self.YELLOW if self.selected_difficulty == "easy" else self.WHITE
                medium_color = self.YELLOW if self.selected_difficulty == "medium" else self.WHITE
                hard_color = self.YELLOW if self.selected_difficulty == "hard" else self.WHITE
                easy_text = self.font_info.render("E - EASY", True, easy_color)
                medium_text = self.font_info.render("M - MEDIUM", True, medium_color)
                hard_text = self.font_info.render("H - HARD", True, hard_color)
                
                # COLOR TEXT (New Visuals)
                current_color = rospy.get_param('change_player_color', 1)
                c1_color = self.YELLOW if current_color == 1 else self.WHITE
                c2_color = self.YELLOW if current_color == 2 else self.WHITE
                c3_color = self.YELLOW if current_color == 3 else self.WHITE
                
                col1_text = self.font_info.render("1 - RED", True, c1_color)
                col2_text = self.font_info.render("2 - PURPLE", True, c2_color)
                col3_text = self.font_info.render("3 - BLUE", True, c3_color)

                # Positioning
                # Difficulty
                self.screen.blit(easy_text, (self.screen_width/2 - 200, 200))
                self.screen.blit(medium_text, (self.screen_width/2 - 200, 240))
                self.screen.blit(hard_text, (self.screen_width/2 - 200, 280))
                
                # Colors
                self.screen.blit(col1_text, (self.screen_width/2 + 50, 200))
                self.screen.blit(col2_text, (self.screen_width/2 + 50, 240))
                self.screen.blit(col3_text, (self.screen_width/2 + 50, 280))
                
                if self.selected_difficulty is not None:
                    enter_text = self.font_info.render("Press ENTER to start", True, self.WHITE)
                    enter_rect = enter_text.get_rect(center=(self.screen_width/2, 450))
                    self.screen.blit(enter_text, enter_rect)
                info_text = self.font_info.render("Waiting for User Info from ROS...", True, self.WHITE)
                info_rect = info_text.get_rect(center=(self.screen_width/2, self.screen_height - 50))
                self.screen.blit(info_text, info_rect)

            elif self.state == "RUNNING":
                self.draw_map()
                self.draw_moving_platforms()
                self.draw_crown()
                self.draw_donkey_kong()
                self.draw_coins()
                self.draw_barrels()
                self.draw_hearts()
                self.draw_player_original()
                self.draw_ui()

            elif self.state == "VICTORY":
                self.victory_screen.draw(self.screen, self.score)

            elif self.state == "GAME_OVER":
                self.screen.fill((50, 0, 0))
                text = self.font_gameover.render("GAME OVER", True, self.WHITE)
                score_msg = self.font_info.render(f"Final Score: {self.score}", True, self.WHITE)
                self.screen.blit(text, (200, 200))
                self.screen.blit(score_msg, (280, 300))
                restart_msg = self.font_info.render("Press ENTER to Restart", True, self.YELLOW)
                self.screen.blit(restart_msg, (220, 400))

            pygame.display.flip()
            rate.sleep()

if __name__ == '__main__':
    try:
        PygameNode()
    except rospy.ROSInterruptException:
        pass	


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
from project_game.msg import game_state
from std_msgs.msg import String

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

        # --- Variables ---
        self.state = "WELCOME"
        self.player_x = 100
        self.player_y = 300
        self.score = 0
        self.lives = 3
        self.barrels_data = []
        self.coins_data = [] 
        
        # --- COLORES ---
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 255, 0) 
        self.BLUE_PLATFORM = (0, 0, 255)
        self.YELLOW_LADDER = (255, 255, 0)
        
        # Sprites Colors
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

        # Subscribers
        rospy.Subscriber("game_state", game_state, self.callback_game_state)
        rospy.Subscriber("barrels_data", String, self.callback_barrels)
        rospy.Subscriber("coins_data", String, self.callback_coins)
        
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
        if hasattr(msg, 'lives'):
            self.lives = msg.lives

    def callback_barrels(self, msg):
        raw_data = msg.data
        new_barrels = []
        if raw_data:
            items = raw_data.split(';')
            for item in items:
                if ',' in item:
                    coords = item.split(',')
                    try:
                        new_barrels.append((int(coords[0]), int(coords[1])))
                    except: pass
        self.barrels_data = new_barrels

    def callback_coins(self, msg):
        raw_data = msg.data
        new_coins = []
        if raw_data:
            items = raw_data.split(';')
            for item in items:
                if ',' in item:
                    coords = item.split(',')
                    try:
                        new_coins.append((int(coords[0]), int(coords[1])))
                    except: pass
        self.coins_data = new_coins

    # ---------------------------------------------------
    #                DRAWING
    # ---------------------------------------------------

    def draw_map(self):
        for row_index, row in enumerate(self.level_map):
            for col_index, tile in enumerate(row):
                x = col_index * self.block_width
                y = row_index * self.block_height
                if tile == 1: 
                    pygame.draw.rect(self.screen, self.BLUE_PLATFORM, (x, y, self.block_width, self.block_height))
                elif tile == 2: 
                    pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 10, y, 5, self.block_height))
                    pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 25, y, 5, self.block_height))
                    for i in range(0, self.block_height, 5):
                        pygame.draw.rect(self.screen, self.YELLOW_LADDER, (x + 10, y + i, 20, 2))

    def draw_player_original(self):
        x = self.player_x
        y = self.player_y
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
        dk_y = 90    # ADJUSTED: Moves DK up to sit on the platform
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
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx, cy), (cx-5, cy-10), (cx+10, cy)])
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx+10, cy), (cx+15, cy-15), (cx+20, cy)])
        pygame.draw.polygon(self.screen, self.CROWN_COLOR, [(cx+20, cy), (cx+35, cy-10), (cx+30, cy)])
        pygame.draw.circle(self.screen, self.RED, (cx+15, cy+10), 5)

    def draw_barrels(self):
        for (bx, by) in self.barrels_data:
            pygame.draw.circle(self.screen, self.BARREL_COLOR, (bx + 10, by + 10), 10)
            pygame.draw.circle(self.screen, (100, 50, 20), (bx + 10, by + 10), 6)

    def draw_coins(self):
        for (cx, cy) in self.coins_data:
            pygame.draw.circle(self.screen, self.COIN_COLOR, (cx + 15, cy + 15), 8)
            pygame.draw.circle(self.screen, self.YELLOW, (cx + 15, cy + 15), 4)

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

            self.screen.fill(self.BLACK)

            if self.state == "WELCOME":
                if self.background_image:
                    self.screen.blit(self.background_image, (0, 0))
                welcome_text = self.font_welcome.render("WELCOME", True, self.YELLOW)
                welcome_rect = welcome_text.get_rect(center=(self.screen_width/2, 80))
                welcome_border = self.font_welcome.render("WELCOME", True, self.BLACK)
                self.screen.blit(welcome_border, (welcome_rect.x + 2, welcome_rect.y + 2))
                self.screen.blit(welcome_text, welcome_rect)
                msg_text = "Waiting for User Info from ROS..."
                info_text = self.font_info.render(msg_text, True, self.WHITE)
                info_rect = info_text.get_rect(center=(self.screen_width/2, self.screen_height - 50))
                self.screen.blit(info_text, info_rect)
            
            elif self.state == "RUNNING":
                self.draw_map()
                self.draw_crown()
                self.draw_donkey_kong()
                self.draw_coins() 
                self.draw_barrels()
                self.draw_player_original() 
                self.draw_ui()

            elif self.state == "GAME_OVER":
                self.screen.fill((50, 0, 0))
                font = pygame.font.SysFont("Arial", 60)
                text = font.render("GAME OVER", True, self.WHITE)
                score_msg = font.render(f"Final Score: {self.score}", True, self.WHITE)
                self.screen.blit(text, (250, 200))
                self.screen.blit(score_msg, (250, 300))
                restart_msg = self.font_info.render("Press any key to Restart", True, self.YELLOW)
                restart_rect = restart_msg.get_rect(center=(self.screen_width/2, self.screen_height/2 + 100))
                self.screen.blit(restart_msg, restart_rect)

            pygame.display.flip()
            rate.sleep()

if __name__ == '__main__':
    try:
        PygameNode()
    except rospy.ROSInterruptException:
        pass
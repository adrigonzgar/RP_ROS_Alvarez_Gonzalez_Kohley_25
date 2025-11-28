#!/usr/bin/env python3

#############################################
#                PYGAME_NODE                #
#############################################

import rospy
import pygame
from project_game.msg import game_state

class PygameNode:
    def __init__(self):
        rospy.init_node("pygame_node")
        rospy.loginfo("PYGAME_NODE started.")

        # --- Init pygame ---
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("ROS Game - Minimal Edition")

        self.clock = pygame.time.Clock()

        # --- Game state variables ---
        self.state = "WELCOME"
        self.player_x = 100
        self.player_y = 300
        self.score = 0
        
        # --- LEVEL CONFIGURATION ---
        self.block_width = 40
        self.block_height = 25
        
        # Colors
        self.BLUE_PLATFORM = (0, 0, 255)  
        self.YELLOW_LADDER = (255, 255, 0) # <--- New Color for Ladders

        # Define the level map (24 rows x 20 columns)
        # 0 = Empty, 1 = Blue Platform, 2 = Yellow Ladder
        self.level_map = [
            # Level 6 (Empty top area)
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 0
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 1
            
            # Level 5 (Princess Platform)
            [0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0], # 2 
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 3 (Ladder)
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 4 (Ladder)
            [0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0], # 5 (Ladder)

            # Level 4 (DK Platform)
            [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0], # 6 
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 7 (Ladders down)
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 8
            [0,0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0], # 9

            # Level 3 (Two separated platforms)
            [0,0,1,1,1,1,1,1,2,0,0,0,0,1,1,1,1,1,0,0], # 10 
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 11 (Ladders down)
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 12
            [0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # 13

            # Level 2 
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], # 14
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 15 (Ladders down)
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 16
            [0,0,0,0,0,0,0,2,0,0,0,0,2,0,0,0,0,0,0,0], # 17

            # Level 1
            [0,0,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0], # 18 
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 19 (Ladders down to ground)
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 20
            [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0], # 21

            # Ground Level
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # 22
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]  # 23
        ]
        
        # Define a simple Mario-like figure
        self.mario_pixels = [
            (10, 0, (255, 0, 0)), (15, 0, (255, 0, 0)), 
            (5, 5, (255, 0, 0)), (20, 5, (255, 0, 0)), 
            (10, 5, (255, 255, 255)), (15, 5, (255, 255, 255)), 
            (5, 10, (0, 0, 255)), (20, 10, (0, 0, 255)), 
            (10, 10, (255, 255, 255)), (15, 10, (255, 255, 255)), 
            (10, 15, (0, 0, 255)), (15, 15, (0, 0, 255)), 
            (10, 20, (139, 69, 19)), (15, 20, (139, 69, 19)), 
        ]
        self.pixel_size = 6

        # --- Subscribe to game state ---
        rospy.Subscriber("game_state", game_state, self.callback_state)

        self.run()

    # Receives state updates from game_node
    def callback_state(self, msg):
        self.state = msg.state
        self.player_x = msg.player_x
        self.player_y = msg.player_y
        self.score = msg.score

    # Draw WELCOME screen
    def draw_welcome(self):
        self.screen.fill((0, 0, 0))
        self.draw_text("WELCOME!", 60, 400, 150)
        self.draw_text("Press any key to start", 32, 400, 300)
        pygame.display.flip()

    # Draw RUNNING screen
    def draw_running(self):
        self.screen.fill((0, 0, 0))
        
        # Iterate over the map to draw platforms (1) and ladders (2)
        for row_index, row in enumerate(self.level_map):
            for col_index, tile in enumerate(row):
                
                # Calculate exact position
                x_pos = col_index * self.block_width
                y_pos = row_index * self.block_height

                if tile == 1:
                    # --- DRAW BLUE PLATFORM ---
                    pygame.draw.rect(
                        self.screen, 
                        self.BLUE_PLATFORM, 
                        (x_pos, y_pos, self.block_width, self.block_height)
                    )
                    # Border
                    pygame.draw.rect(
                         self.screen, (0,0,0), 
                         (x_pos, y_pos, self.block_width, self.block_height), 1
                    )
                
                elif tile == 2:
                    # --- DRAW YELLOW LADDER ---
                    # 1. Left Vertical Rail
                    pygame.draw.line(
                        self.screen, self.YELLOW_LADDER, 
                        (x_pos + 10, y_pos), (x_pos + 10, y_pos + self.block_height), 3
                    )
                    # 2. Right Vertical Rail
                    pygame.draw.line(
                        self.screen, self.YELLOW_LADDER, 
                        (x_pos + 30, y_pos), (x_pos + 30, y_pos + self.block_height), 3
                    )
                    # 3. Horizontal Rungs (Steps) - Draw 4 steps per block
                    for step_y in range(y_pos + 5, y_pos + self.block_height, 6):
                        pygame.draw.line(
                            self.screen, self.YELLOW_LADDER,
                            (x_pos + 10, step_y), (x_pos + 30, step_y), 3
                        )
        
        # Draw the custom Mario figure
        for px_offset_x, px_offset_y, color in self.mario_pixels:
            pygame.draw.rect(
                self.screen, 
                color, 
                (self.player_x + px_offset_x, 
                 self.player_y + px_offset_y, 
                 self.pixel_size, 
                 self.pixel_size)
            )

        # Score
        self.draw_text(f"Score: {self.score}", 30, 70, 30)

        pygame.display.flip()

    # Draw GAME OVER
    def draw_game_over(self):
        self.screen.fill((50, 0, 0))
        self.draw_text("GAME OVER", 60, 400, 150)
        self.draw_text(f"Final Score: {self.score}", 40, 400, 250)
        self.draw_text("Press any key to restart", 30, 400, 350)
        pygame.display.flip()

    # Helper function to draw text centered
    def draw_text(self, text, size, x, y):
        font = pygame.font.SysFont("Arial", size)
        surface = font.render(text, True, (255, 255, 255))
        rect = surface.get_rect(center=(x, y))
        self.screen.blit(surface, rect)

    # Main loop
    def run(self):
        rate = rospy.Rate(60)  # 60 fps

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Pygame window closed")

            # Draw according to state
            if self.state == "WELCOME":
                self.draw_welcome()
            elif self.state == "RUNNING":
                self.draw_running()
            elif self.state == "GAME_OVER":
                self.draw_game_over()

            self.clock.tick(60)
            rate.sleep()


if __name__ == "__main__":
    try:
        PygameNode()
    except rospy.ROSInterruptException:
        pass
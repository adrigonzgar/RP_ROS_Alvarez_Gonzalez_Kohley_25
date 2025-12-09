# RP_ROS_Alvarez_Gonzalez_Kohley_25

ROS Noetic project that integrates a 2D game controlled through ROS nodes, with graphics rendered using Pygame and modular communication between nodes. The goal of the project is to demonstrate real-time interaction between ROS, keyboard input, distributed game logic, and graphical visualization.

---

## System Overview

This project consists of five ROS nodes working together to implement a fully interactive 2D game. Each component is isolated into its own node to ensure modularity and clean communication through ROS topics, services, and parameters.

---

## Node Descriptions

### info_user.py
Collects user information from the terminal:
- Real name  
- Username  
- Age  

Publishes the data on the topic: user_information (project_game/user_msg)

Also sets a ROS parameter indicating that user data is ready.

---

### control_node.py
Handles global keyboard input using pynput.  
Publishes commands to the topic: keyboard_control (std_msgs/String)

Supported commands:
- LEFT, RIGHT, UP, DOWN  
- START (Enter key)  
- RESET  
- DIFF_EASY, DIFF_MEDIUM, DIFF_HARD  
- COLOR_1, COLOR_2, COLOR_3  

This node performs all keyboard reading so that the pygame window can focus exclusively on rendering.

---

### game_node.py
Main controller of the game logic. It handles:
- Player physics (movement, gravity, ladders)  
- Collisions with platforms, barrels, coins, and hearts  
- Score and life system  
- Difficulty configuration  
- Player color through ROS parameters  
- State transitions: WELCOME, RUNNING, VICTORY, GAME_OVER  

Publishes:
- game_state  
- result_information  
- barrels_data  
- coins_data  
- hearts_data  
- moving_platforms_data  

Provides services:
- GetUserScore  
- SetGameDifficulty  

Receives commands from keyboard_control.

---

### pygame_node.py
Handles rendering of the entire game interface.

Draws:
- Welcome screen with difficulty and color highlighting  
- Player sprite  
- Platforms, ladders  
- Barrels, coins, hearts  
- Donkey Kong and crown  
- Score and lives  
- Victory animation  
- Game Over screen  

This node does not handle any keyboard input.

---

### result_node.py
Receives the final score and prints a summary.  
Optionally could store results in JSON.

---

## ROS Messages & Services

user_msg.msg contains:
- string name  
- string username  
- int64 age  

game_state.msg contains:
- int64 player_x  
- int64 player_y  
- int64 score  
- string state  
- int64 lives  
- string difficulty  
- int64 color  

GetUserScore.srv:
- Request: username  
- Response: score  

SetGameDifficulty.srv:
- Request: change_difficulty  
- Response: success, message  

---

## Project Structure

game_ws/  
 ├── src/  
 │    └── project_game/  
 │         ├── msg/  
 │         ├── srv/  
 │         ├── scripts/  
 │         ├── launch/  
 │         ├── CMakeLists.txt  
 │         └── package.xml  
 ├── compilar  
 ├── install_deps.sh  
 └── requirements.txt  

---

## Installation & Dependencies

To install all dependencies:
./install_deps.sh

To build the workspace:
./compilar

The script:
- Runs catkin_make  
- Sources devel/setup.bash  

---

## Running the Game

Run:
cd game_ws  
source devel/setup.bash  
roslaunch project_game game.launch  

This automatically launches:
- info_user  
- control_node  
- game_node  
- pygame_node  
- result_node  

---

## Controls

Arrow Keys → Move  
Enter → Start game  
R → Reset  
E / M / H → Select difficulty  
1 / 2 / 3 → Select player color  

---

## Suggested Improvements

- Add audio using pygame.mixer  
- Improve animations (walking, jumping)  
- Add more levels or procedural maps  
- Implement new enemy types  
- Persistent high-score leaderboard (JSON/YAML)  
- Power-ups or special items  
- Enhanced sprite artwork  

---

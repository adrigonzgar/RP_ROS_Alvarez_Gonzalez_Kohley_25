# RP_ROS_Alvarez_Gonzalez_Kohley_25
Nuevo repositorio para la entrega de ROS debido a problemas con el anterior


Proyecto desarrollado en ROS Noetic donde se integra un juego 2D controlado mediante nodos ROS, con visualización en Pygame y comunicación modular entre nodos.  
El objetivo del proyecto es demostrar interacción en tiempo real entre ROS, entradas de teclado, lógica de juego distribuida y visualización gráfica.

---

## 🧩 **Arquitectura del Sistema**

El sistema está compuesto por **6 nodos ROS** que se comunican entre sí mediante *topics*:
      ┌───────────────────┐
      │   INFO_USER_NODE  │
      └─────────┬─────────┘
                │ publishes user_msg
                ▼
      ┌───────────────────┐
      │     GAME_NODE     │ <────────── keyboard_control  ─────────┐
      └───────┬────┬─────┘                                        │
              │    │                                               │
         result   game_state                                        │
              │    │                                               │
              ▼    ▼                                               │
    ┌──────────┐   ┌─────────────┐                                │
    │RESULT_NODE│   │PYGAME_NODE  │ <──────────── CONTROL_NODE ────┘
    └──────────┘   └─────────────┘


## 🧩 **Descripción de Nodos**

### ✔ `info_user.py`
Solicita al usuario su nombre, edad y username, y publica los datos por el topic: user_information (project_game/user_msg)

### ✔ `control_node.py`
Lee el teclado del usuario y publica comandos como:

- `"UP"`
- `"DOWN"`
- `"LEFT"`
- `"RIGHT"`

en el topic: keyboard_control (std_msgs/String)


### ✔ `game_node.py`
Es el núcleo lógico del sistema. Contiene los estados:

- `WELCOME`
- `RUNNING`
- `GAME_OVER`

Publica el estado en tiempo real: game_state (project_game/game_state) y envía puntuación final a:

result_information (std_msgs/Int64)

### ✔ `pygame_node.py`
Ventana gráfica creada con **Pygame**.  
Recibe el estado del juego y renderiza:

- pantalla de bienvenida  
- jugador en movimiento  
- marcador  
- pantalla de game over  

---

### ✔ `result_node.py`
Recibe el score final y muestra el resultado al usuario, o lo guarda si se amplía con JSON.

---

## 🎮 **Mensajes ROS**

### `user_msg.msg`
```txt
string name
string username
int64 age

### `game_state.msg`
int64 player_x
int64 player_y
int64 score
string state




🚀 Cómo Compilar

Dentro del workspace:
./compilar

Este script:

ejecuta catkin_make

hace source devel/setup.bash

lanza roscore

🚀 Cómo Ejecutar los Nodos

Cada nodo se lanza desde una terminal distinta con:

./run_nodes info
./run_nodes control
./run_nodes game
./run_nodes pygame
./run_nodes result

ESTRUCTURA DEL PROYECTO

game_ws/
 ├── src/
 │    └── project_game/
 │         ├── msg/
 │         │    ├── user_msg.msg
 │         │    └── game_state.msg
 │         ├── scripts/
 │         │    ├── info_user.py
 │         │    ├── control_node.py
 │         │    ├── game_node.py
 │         │    └── pygame_node.py
 │         ├── CMakeLists.txt
 │         └── package.xml
 ├── compilar
 └── run_nodes

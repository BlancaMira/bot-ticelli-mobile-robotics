# Bot-ticelli

Bot-ticelli és un projecte desenvolupat amb ROS i TurtleBot3 que permet detectar una figura mitjançant visió per ordinador i reproduir-la físicament sobre el terra mitjançant la trajectòria del robot.

## Objectius del Projecte

L'arquitectura està dissenyada per resoldre de manera modular els següents reptes:

1. **Exploració i Mapejat:** Construcció de mapes d'ocupació 2D en entorns desconeguts (SLAM) o localització en mapes preexistents (AMCL).
2. **Anàlisi Espacial:** Cerca probabilística i validació de zones lliures d'obstacles i regions desconegudes suficients per allotjar el dibuix.
3. **Visió Computacional:** Segmentació robusta mitjançant el mètode d'Otsu i filtres morfològics, i extracció/simplificació de contorns geomètrics.
4. **Planificació i Control:** Normalització de traçats, generació de camins (`nav_msgs/Path`) i seguiment de trajectòries adaptatiu amb detenció en cantonades vives i seguretat activa contra obstacles dinàmics.

---

## Arquitectura del sistema

El paquet s'organitza en els següents nodes funcionals en C++:

```text
botticelli_pkg/
├── config/
│   ├── boticelli.rviz
│   ├── drawing_follower_params.yaml
│   ├── drawing_navigator_params.yaml
│   ├── drawing_planner_params.yaml
│   └── vision_params.yaml
│
├── launch/
│   ├── boticelli_draw.launch
│   ├── boticelli_slam.launch
|   ├── drawing_follower.launch
|   ├── drawing_navigator.launch
|   ├── drawing_planner.launch
│   ├── drawing.launch
│   ├── localization.launch
│   └── vision.launch
│
├── maps/
│   ├── intent_features.txt
│   ├── intent.*
│   ├── mapa_aula2.*
│   ├── mapa_circuit_classe.*
│   └── mapa_passadis.*
│
└── src/
    ├── boticelli_interface_node.cpp
    ├── drawing_follower_node.cpp    
    ├── drawing_navigator_node.cpp
    ├── drawing_planner_node.cpp
    ├── free_space_explorer_node.cpp
    └── vision_node.cpp
```

### Detall dels Nodes i Pipeline de Comunicació

- **`boticelli_interface_node`:** Node central que actua com a orquestrador a través d'una CLI per terminal. Gestiona l'estat global, llança els fitxers `.launch` fills mitjançant subprocesos de Linux (`fork` / `exec`) i coordina les transicions publicant ordres directes.
- **`free_space_explorer_node`:** Subscrit a `/map` i `/scan`. Genera candidats aleatoris sobre l'espai mitjançant un mostreig de tipus Monte Carlo, verificant que cap cel·la del "full virtual" contingui obstacles ni zones desconegudes, seleccionant el punt que maximitzi la distància de seguretat. Publica a `/drawing_center`.
- **`vision_node`:** Processa imatges del tòpic `/camera/image/compressed`. Aplica un `GaussianBlur`, binaritza automàticament mitjançant `cv::THRESH_OTSU` invertit, i neteja la màscara amb morfologia de tancament i obertura (`close_open`). Extreu el contorn de major àrea, el simplifica amb l'algorisme de Douglas-Peucker (`approxPolyDP`) i el publica normalitzat ($[0.0, 1.0]$) a `/contour_points`.
- **`drawing_planner_node`:** Agafa la plantilla de `/contour_points` i el centre de `/drawing_center`. Escala la figura segons les dimensions en metres introduïdes per l'usuari (mantenint la relació d'aspecte de `/contour_info`) i genera el camí global a `/drawing_path`.
- **`drawing_navigator_node`:** Executa una màquina d'estats finits per desplaçar el robot fins al primer punt del camí de forma controlada (`/cmd_vel`) i orientar-lo seguint la línia tangent abans de recol·locar el retolador. Confirma l'estat a `/drawing_ready`.
- **`drawing_follower_node`:** Segueix el camí fent servir un control de persecució basat en un punt avançat (*lookahead*). Incorpora lògica geomètrica per frenar a zero i girar en cantonades tancades per mantenir el traç net, així com una aturada d'emergència si es detecta un obstacle sobtat al tòpic `/scan`.

---

## Dependències i Requisits

- **SO:** Ubuntu 20.04 LTS amb ROS Noetic Suite.
- **Paquets ROS Estàndard:** `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf`, `gmapping`, `amcl`, `map_server`.
- **Llibreries Externes:** OpenCV (`cv_bridge`).
- **Plataforma de Hardware:** TurtleBot3 (Burger/Waffle) equipat amb LiDAR 360° i Raspberry Pi Camera.

## Compilació

Navega al teu espai de treball de Catkin, compila el projecte i recarrega l'entorn:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Execució

Abans de començar, assegureu-vos que l'ordinador remot i el TurtleBot3 estan connectats a la mateixa xarxa.

### Terminal 1 — ROS Master (ordinador remot)

Configurar les variables de xarxa:

```bash
vim ~/.bashrc
```

Afegir:

```bash
export ROS_MASTER_URI=http://<IP_ORDINADOR>:11311
export ROS_IP=<IP_ORDINADOR>
```

Aplicar els canvis:

```bash
source ~/.bashrc
```

Iniciar el ROS Master:

```bash
roscore
```

### Terminal 2 — Bringup del TurtleBot3

Connectar-se al robot:

```bash
ssh ubuntu@<IP_ROBOT>
```

Configurar les variables de xarxa del robot:

```bash
vim ~/.bashrc
```

Afegir:

```bash
export ROS_MASTER_URI=http://<IP_ORDINADOR>:11311
export ROS_IP=<IP_ROBOT>
```

Aplicar els canvis:

```bash
source ~/.bashrc
```

Iniciar els nodes bàsics del TurtleBot3:

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Terminal 3 — Càmera del robot

Connectar-se al robot:

```bash
ssh ubuntu@<IP_ROBOT>
```

Executar la càmera:

```bash
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
```

### Terminal 4 — Bringup remot

A l'ordinador remot:

```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

Aquest node permet visualitzar i controlar el TurtleBot3 des de l'ordinador remot.

### Terminal 5 — Interfície principal de Bot-ticelli

Executar el node principal del projecte:

```bash
rosrun botticelli_pkg boticelli_interface_node
```

---

## Autors

- Pablo Abella Escudero
- Raul Castillo Moraz
- Blanca Mira Fradera
- Olga Obiols Fuentes
- Laida Queral Llopart

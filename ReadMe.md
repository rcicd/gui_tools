# Gui tools for Duckiebot and BotWithNoName powered by ROS2 

## How to install (native)

### Prerequisites 
+ Ubuntu 22.04 (recommended) 
+ git
+ ROS2 Iron
+ Python3
+ pip3

### Installation
1. Clone this repository:
    ```shell
    git clone https://github.com/rcicd/gui_tools.git
    ```
2. Change directory:
    ```shell 
    cd ./gui_tools
    ```
3. Execute `native_installation.sh` script
    ```shell
    ./native_installation.sh
    ```

## How to install (docker)

### Prerequisites
+ Linux (recommended)
+ Docker

### Installation
1. Clone this repository:
    ```shell
    git clone https://github.com/rcicd/gui_tools.git
    ```
2. Change directory:
    ```shell 
    cd ./gui_tools
    ```
3. Execute `docker_installation.sh` script
    ```shell
    ./docker_installation.sh
    ```

## How to use
### Virtual joystick
1. Go to repository root directory
2. Execute `virtual_joystick.sh` script with bot name you want to control:
    ```shell
    ./virtual_joystick.sh <BOT_NAME>
    ```
3. To stop the program just close the window

### Intrinsic calibration tool
1. Go to repository root directory
2. Execute `intrinsic_calibration_tool.sh` script with bot name you want to calibrate:
    ```shell
    ./intrinsic_calibration_tool.sh <BOT_NAME>
    ```
3. Start calibration. You have to use chessboard pattern to calibrate the camera. Move the robot 
so that the camera can see the chessboard from different angles. You need to make at least 30 different images:
    + From middle distance (50 cm), you need to take 9 photos one for center, 4 for each corner and 4 for each side of the image. Repeat this process three times: 
        + With bot turned left
        + With bot turned right
        + With bot in normal position
    + From close distance (10-20 cm), you need to make three images:
        + With bot turned left
        + With bot turned right
        + With bot in normal position
    
    Colorful circle will help you. If it's green it means that chessboard is visible form middle distance, if it's blue it means that chessboard is visible from close distance.
4. After taking all 30 photos you can press `OK` button to calibrate camera 
5. To stop the program just close the window.
6. Don't forget to reboot the robot after calibration.
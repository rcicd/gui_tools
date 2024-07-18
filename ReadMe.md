# Gui tools for Duckiebot powered by ROS2 and BotWithNoName

## How to install

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
3. Execute `install.sh` script
```shell
./install.sh
```

## How to use
### Virtual joystick
1. Go to repository root directory
2. Execute `virtual_joystick.sh` script with bot name you want to control:
```shell
./virtual_joystick.sh <BOT_NAME>
```
3. To stop the program press `ctrl+c`
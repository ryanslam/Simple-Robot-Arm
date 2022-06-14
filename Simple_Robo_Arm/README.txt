# Simple Robot Arm

simple_robot_arm is a software platform for control demonstration with a variety of different control laws and algorithms.

## Installation and Setup

### Flash an sd card with Raspberry Pi OS per the Rasbian instructions.
https://www.raspberrypi.org/software/
### Clone the repository in the desired location
```bash
cd simple_robot_arm
```
### Install and create a virtual environment

https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#:~:text=To%20create%20a%20virtual%20environment,virtualenv%20in%20the%20below%20commands.&text=The%20second%20argument%20is%20the,project%20and%20call%20it%20env%20. 
#### Unix
```bash
python3 -m pip install --user virtualenv
python3 -m venv env
source env/bin/activate 
```

#### Windows
```bash
py -m pip install --user virtualenv
py -m venv env
.\env\Scripts\activate
```

### Setting up raspberrypi
Use sudo raspi-config to allow the full SD card to be used, expand_rootfs.
There is an issue with numpy, the following command fixes it: 
```bash
```

The following resolves issues with package dependencies for roboticstoolbox and numpy. libatlas-base-dev is the numpy one. Not actually sure if others are necessary.:
```bash
sudo apt-get install libopenblas-dev libatlas-base-dev libblas-dev liblapack-dev libdsdp-dev libfftw3-dev libglpk-dev libgsl-dev
sudo apt-get install python-cvxopt
```

### Install Requirements
```bash
pip install -r requirements.txt
```
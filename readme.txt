## Installation
sudo apt install python3 python3-pip python3-dev python3-venv cmake libssl-dev xorg-dev libglu1-mesa-dev libudev-dev


upgrade to at least python 3.8
build pyrealsesnse2 from source

###
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-dev

sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk--dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libxml2 libxml2-dev libxslt-dev

git clone https://github.com/IntelRealSense/librealsense.git
cd ./librealsense

./scripts/setup_udev_rules.sh

mkdir build && cd build
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true

sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install

##Export pyrealsense2 to your PYTHONPATH so `import pyrealsense2` works (can add to bashrc)
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/pyrealsense2


python3.8 -m venv .venv

source .venv/bin/activate 
pip install --upgrade pip
pip install cython 
pip install wheel numpy 
pip install transformations pyserial scikit-build 
pip install dronekit apscheduler==3.8.0  py-imu-mpu6050 
pip install opencv-python dt-apriltags

on each startup 
sudo echo 400000 > /sys/bus/i2c/devices/i2c-1/bus_clk_rate


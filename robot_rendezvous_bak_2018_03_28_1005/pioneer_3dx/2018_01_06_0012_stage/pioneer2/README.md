# Pioneer 2 #

How to setup and use the Pioneer 2 software

## Usage ##

The pioneer is currently setup with IP 172.19.0.10 
username: pioneer2
password: <old lab name standard password>

## Installation ##
1. Clean 14.04 Install
2. Install ros-indigo
3. Install dependencies

```
#!bash
sudo apt-get install git ros-indigo-hokuyo-node ros-indigo-p2os-driver ros-indigo-p2os-launch ros-indigo-navigation htop
wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
bash ./enable_kernel_sources.sh
sudo apt-get install ros-indigo-librealsense ros-indigo-realsense-camera ros-indigo-depthimage-to-laserscan ros-indigo-p2os-launch 
sudo addgroup pioneer2 dialout
sudo reboot now

```
4. Clone Repo and init workspace

```
#!bash
git clone https://<username>@bitbucket.org/acrv/pioneer2.git
cd pioneer2
catkin_make
echo "source ~/pioneer2/devel/setup.bash" >> ~/.bashrc
```
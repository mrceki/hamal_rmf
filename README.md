# Hamal RMF
# RMF Visualization Project

This project involves several repositories. Follow the instructions below to clone and build the project.

## Prerequisites

Before you start, make sure to install the required Python package:

```bash
pip3 install python-socketio==5.7.2
```
### Cloning the Repositories
Clone the following repositories:
```
git clone https://github.com/mrceki/hamal_rmf.git
git clone https://github.com/mrceki/rmf_visualization.git
git clone https://github.com/mrceki/rmf-web.git
git clone https://github.com/mrceki/rmf_traffic_editor.git
git clone https://github.com/mrceki/rmf_demos.git
git clone https://github.com/mrceki/rmf_ros2.git
```
Then, clone the Cyclone DDS repository:
```
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.7.x
```
Build the rmf_hamal package:
```
cd /rmf_demos_ws
source install/setup.bash
colcon build --packages-select rmf_hamal
```

Build the rest of the project:
```
colcon build 
source install/setup.bash
```
# Running the Project
To start all RMF nodes, run:
```
ros2 launch rmf_hamal hamal.launch.xml  server_uri:="http://localhost:8000/_internal"
```
To start the free fleet server, run:
```
ros2 launch ff_examples_ros2 fake_server.launch.xml 
```
## To start the web server, navigate to the dashboard directory and start the server:
```
pnpm install
cd /rmf_demos_ws/src/rmf_web/packages/dashboard
pnpm start
```
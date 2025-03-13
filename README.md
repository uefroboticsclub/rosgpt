# rosgpt

docker exec -it rosgpt-turtlesim-demo bash

-------------------------------------

pip3 install -r requirements.txt

rm -rf build/ install/

colcon build --symlink-install --packages-select turtle_agent

source install/setup.bash

# Terminal 1

ros2 run turtlesim turtlesim_node

# Terminal 2

ros2 run turtle_agent turtle_agent

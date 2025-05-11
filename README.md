# Anscer Multimap Navigation

## Installation Instructions:
1. Build Packages
    ```
    git clone https://github.com/vaishanth-rmrj/anscer-robotics-assignment.git
    cd ./anscer-robotics-assignment/ws_ros/
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    ```
2. Pull and Spin up PostgresSQL in docker
    ```
    docker pull postgres
    docker run --name postgres-container -e POSTGRES_USER=anscer -e POSTGRES_PASSWORD=anscer -e POSTGRES_DB=wormhole_locations -p 11511:5432 -d postgres
    ```
3. Create a new Table in DB for storing wormhole locations
    ```
    pip install psycopg2
    cd ./anscer-robotics-assignment/
    python3 create_postgres_table.py 
    ```

## Mapping Rooms and Saving Map files
1. Start Gazebo Sim
    ```
    roslaunch start_anscer start_anscer.launch
    ```
2. Start Keyboard Teleop
    ```
    roslaunch anscer_teleop anscer_teleop_key.launch
    ```
3. Launch SLAM and map the env using teleop
    ```
    roslaunch anscer_slam anscer_slam.launch
    ```
4. Save Map to disk
    ```
    rosrun map_server map_saver -f ~/anscer-robotics-assignment/ws_ros/src/ar100/anscer_navigation/maps/<map_name>
    ```

## Storing Map specific Wormhole location on database
1. Start Robot navigation and Navigate to Wormhole location using 2D navigate tool in RVIZ
    ```
    roslaunch anscer_navigation anscer_navigation.launch map_name:=room1
    ```
2. Save Wormhole location to DB
    ```
    rosrun anscer_multimap_navigator wormhole_location_saver
    ```

## Multimap Navigation
1. Start the multimap navigator
    ```
    roslaunch anscer_multimap_navigator anscer_multimap_navigation.launch current_map_name:=room1
    ```
2. Provide the Goal location using navigator action client
    ```
    rosrun anscer_multimap_navigator navigator_action_client _map_name:=room2 _x:=12.50 _y:=-3.40 # location in room2

    other options:
    [Location 1] [Room1] rosrun anscer_multimap_navigator navigator_action_client _map_name:=room1 _x:=-0.47 _y:=2.28
    [Location 2] [Room1] rosrun anscer_multimap_navigator navigator_action_client _map_name:=room1 _x:=-0.33 _y:=-3.01

    ```
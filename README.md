docker pull postgres
docker run --name postgres-container -e POSTGRES_USER=anscer -e POSTGRES_PASSWORD=anscer -e POSTGRES_DB=wormhole_locations -p 11511:5432 -d postgres


<!-- saving wormhole location -->
rosrun anscer_multimap_navigator wormhole_location_saver

<!-- running action server -->
rosrun anscer_multimap_navigator navigator_action_server 

<!-- calling action client -->
rosrun anscer_multimap_navigator navigator_action_client _map_name:=room1 _x:=1.23 _y:=4.56

docker pull postgres
docker run --name postgres-container -e POSTGRES_USER=anscer -e POSTGRES_PASSWORD=anscer -e POSTGRES_DB=wormhole_locations -p 11511:5432 -d postgres

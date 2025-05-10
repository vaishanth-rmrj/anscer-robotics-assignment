import psycopg2

conn = psycopg2.connect(
    dbname="wormhole_locations",
    user="anscer",
    password="anscer",
    host="localhost",
    port=11511
)
cur = conn.cursor()

cur.execute("""
    DROP SCHEMA public CASCADE;
    CREATE SCHEMA public;
""")
conn.commit()
print("Deleted All previous data")

cur.execute("""
    CREATE TABLE IF NOT EXISTS wormhole_locations (
        map_name   TEXT PRIMARY KEY,
        location   INTEGER[]
    )
""")

conn.commit()
cur.close()
conn.close()
print("Wormhole Locations table created.")

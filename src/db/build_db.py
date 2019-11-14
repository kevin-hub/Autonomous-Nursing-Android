import sqlite3

conn = sqlite3.connect('world.db')

c = conn.cursor()
# Create tables
#item table gives a notion of which of our preprogrammed locations each object is in
c.execute('''CREATE TABLE items
             (item_id text, location_id text, weight real)''')
#location table gives real world target coordinates for each item.
c.execute('''CREATE TABLE locations
             (location_id text, pos_x real, pos_y real, pos_theta real, height real)''')

# Insert a row of data
#under items
c.execute("INSERT INTO items VALUES ('bottle','A','500')")
c.execute("INSERT INTO items VALUES ('book','B','300')")
c.execute("INSERT INTO items VALUES ('teddy','C','100')")
#under
c.execute("INSERT INTO locations VALUES ('HOME','0','0','0','2')")
c.execute("INSERT INTO locations VALUES ('BEDSIDE','5','0','0','2')")
c.execute("INSERT INTO locations VALUES ('A','0','5','0','1')")
c.execute("INSERT INTO locations VALUES ('B','10','10','1.57','1')")
c.execute("INSERT INTO locations VALUES ('C','10','10','1.57','2')")

###you can also delete rows from a db using c.execute(DELETE FROM table_name WHERE condition)

# Save (commit) the changes
conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()

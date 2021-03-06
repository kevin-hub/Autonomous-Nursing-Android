import sqlite3

conn = sqlite3.connect('world.db')

c = conn.cursor()
# Create tables
#item table gives a notion of which of our preprogrammed locations each object is in
c.execute('''CREATE TABLE items
             (item_id text, location_id text, weight real)''')
#location table gives real world target coordinates for each item.
c.execute('''CREATE TABLE locations
             (location_id text, pos_x real, pos_y real, z_ori real, w_ori real)''')

# Insert a row of data
#under items
c.execute("INSERT INTO items VALUES ('remote','RIGHT','500')")
c.execute("INSERT INTO items VALUES ('book','RIGHT','300')")
c.execute("INSERT INTO items VALUES ('teddy','FRONT','100')")
#under
c.execute("INSERT INTO locations VALUES ('HOME', '3.98385245187' ,'2.35139133361' , '0.750390288536' ,'0.660995018794')")
c.execute("INSERT INTO locations VALUES ('BEDSIDE_NEXT_TO' , '3.52314562365','3.72042830714','0.780326618642','0.625372183775')")
c.execute("INSERT INTO locations VALUES ('BEDSIDE_FACING','-1','-1.3','2.25','1')")
c.execute("INSERT INTO locations VALUES ('RIGHT', '6.29352000778' ,'3.5092050533','-0.589169881958','0.808009189424')")
c.execute("INSERT INTO locations VALUES ('FRONT', '5.31591619403' ,'4.37975566763','0.774795335762','0.632212138195')")

###you can also delete rows from a db using c.execute(DELETE FROM table_name WHERE condition)

# Save (commit) the changes
conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()

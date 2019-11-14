import sqlite3

#connect to our Database
conn = sqlite3.connect('world.db')
#define our cursor
c = conn.cursor()

#use a query like this to

#use a query like this to extract a property of an item in this case weight
item = 'book'
t = (item,)
c.execute('SELECT weight FROM items WHERE item_id=?',t)
test = c.fetchone()#fetchone fetches one as a tuple: can be assigned to a vbl
print(test)

print('@@@@@')
#use a query like this to extract the target coordinates for an item
item = 'teddy'
t = (item,)
c.execute('SELECT pos_x,pos_y,pos_theta,height FROM items NATURAL JOIN locations WHERE item_id=?', t)
test2 = c.fetchone()
print(test2)

print('#####')
#for queries that return more than one item,
#you can iterate over them using the cursor
c.execute('SELECT location_id FROM locations ORDER BY pos_y') #select all weights
test3 = c.fetchall() #fetchall returns as a list of tuples that can be iterated over
print(test3)

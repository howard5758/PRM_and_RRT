# using shapely for collision detection

from shapely.geometry import Polygon, Point

poly = Polygon(((0, 0), (0, 1), (1, 1), (1, 0)))
point = Point(2, .2)

print(poly)
print(poly.contains(point))

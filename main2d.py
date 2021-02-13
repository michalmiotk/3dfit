from collections import namedtuple
#odleglosc_kwadrat = ( a*x + b - y  )^2
#odleglosc kwadrat = ( a*x + b)^2 - 2(ax +b)y + y^2)
#odleglosc kwadrat = (a^2*x^2 + 2bax + b^2) - 2axy - 2by + y^2
#d(odl_kwadrat)/da = 2a*x^2 + 2bx - 2xy
#d(odl_kwadrat)/db = 2ax + 2b - 2y
Point = namedtuple("Point", "x y")
A = 1
B = 1

a=0.5
b=0.5

points = [] 

for x in range(10):
    y = A*x + B
    points.append(Point(x=x, y=y))
nr_epochs = 100
lr = 0.01

for e in range(nr_epochs):
    for point in points:
        x,y = point.x, point.y
        a -= lr*(2*a*x*x + 2*b*x - 2*x*y)
        b -= lr*(2*a*x + 2*b - 2*y)
    print(a,b)


import yaml
import pylab as plt

print "hello"
f = open("traces.yml")
ya = yaml.load(f)
data = ya["Time20180127-155235"]
traces = data['points']
x = []
y = []
z = []
for point in traces:
    x.append(point[0])
    y.append(point[1])
    z.append(point[2])
figule = plt.figure()
ax = figule.add_subplot(211)
ax.plot(y,z,color = 'r',marker = '.',label = 'yz diminsion')
ax.set_xlabel("y(m)")
ax.set_ylabel("z(m)")
ax.set_title("yz diminsion")
bx = figule.add_subplot(212)
bx.plot(x,y,color = 'g',marker = '.',label = 'xy diminsion')
bx.set_xlabel("x(m)")
bx.set_ylabel("y(m)")
bx.set_title("xy diminsion")
plt.tight_layout(1.5)
figule.legend()
figule.show()
figule.waitforbuttonpress()

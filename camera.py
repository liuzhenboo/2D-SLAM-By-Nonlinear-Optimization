import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
	x = []
	y = []
	axes = plt.gca()
	axes.set_xlim(0, 10)
	axes.set_ylim(-1, 1)
	line, = axes.plot(x, y, 'r-')

	data = 0.00

	while True:
		if len(x) == 900:
			x.pop(-1)
			y.pop(0)
		x.append(data)
		y.append(np.sin(data * np.pi))
		line.set_xdata(x)
		line.set_ydata(y)
		#plt.pause(1)
		data += 0.01
	#plt.pause(1)

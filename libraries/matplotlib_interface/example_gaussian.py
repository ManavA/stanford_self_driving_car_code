#from pylab import * #Done in example.cpp.

delta = 0.05
x = y = arange(-5.0, 5.0, delta)
X, Y = meshgrid(x, y)
Z = bivariate_normal(X, Y, var_x, var_y, mu_x, mu_y, var_xy)
CS = contour(X, Y, Z)
clabel(CS, inline=1, fontsize=10)
draw()
waitforbuttonpress()
clf()

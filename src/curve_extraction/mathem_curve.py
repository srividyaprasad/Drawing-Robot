#!/usr/bin/env python 
from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np
import cv2

img = cv2.imread('bellCurve.png')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret,thresh_img = cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY)

dimensions = img.shape
height = img.shape[0]
width = img.shape[1]
y=[]
x=[]
coords = list(np.column_stack(np.where(thresh_img==0)))
coords.sort(key = lambda x: x[1])

print(coords)

for i in range(img.shape[1]):
    y.append(coords[i][0])
    
for i in range(img.shape[1]):
    x.append(coords[i][1])


tck = interpolate.splrep(x, y, s=0, k=5)
x_new = np.linspace(min(x), max(x), 100)
y_fit = interpolate.BSpline(*tck)(x_new)

plt.title("BSpline curve fitting")
plt.plot(x, y, 'ro', label="original")
plt.plot(x_new, y_fit, '-c', label="B-spline")
plt.legend(loc='best', fancybox=True, shadow=True)
plt.grid()
plt.show() 

#get all black pixel coordinates
#plot in matplotlib as x and y coordinates
#use function

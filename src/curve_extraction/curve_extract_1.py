import cv2
import matplotlib.pyplot as plt
import numpy as np


def extract_and_measure_edges(img_bin):

    # Detect possible corners, and extract candidates
    dst = cv2.cornerHarris(img_bin, 2, 3, 0.04)
    cand = []
    for i, c in enumerate(np.argwhere(dst > 0.1 * np.max(dst)).tolist()):
        c = np.flip(np.array(c))
        if len(cand) == 0:
            cand.append(c)
        else:
            add = True
            for j, d in enumerate(cand):
                d = np.array(d)
                if np.linalg.norm(c - d) < 5:
                    add = False
                    break
            if add:
                cand.append(c)

    # Get indices of actual, nearest matching contour points
    corners = sorted([np.argmin(np.linalg.norm(c - cnt.squeeze(), axis=1))
                      for c in cand])

    # Extract edges from contour, and measure their lengths
    output = cv2.cvtColor(np.zeros_like(img_bin), cv2.COLOR_GRAY2BGR)
    
    for i_c, c in enumerate(corners):
        if i_c == len(corners) - 1:
            edge = np.vstack([cnt[c:, ...], cnt[0:corners[0], ...]])
        else:
            edge = cnt[c:corners[i_c + 1], ...]
        loc = tuple(np.mean(edge.squeeze(), axis=0, dtype=int).tolist())
        color = tuple(np.random.randint(0, 255, 3).tolist())
        length = cv2.arcLength(edge, False)
        
        cv2.polylines(output, [edge], False, color, 2)
        cv2.putText(output, '{:.2f}'.format(length), loc, cv2.FONT_HERSHEY_COMPLEX, 0.5, color, 1)
        '''for i in edge:
            list_x.append(i[0])
            list_y.append(i[1])'''
    return output

import os
#os.chdir('curve_extraction')
# Read and pre-process image, extract contour of shape
# TODO: MODIFY TO FIT YOUR INPUT IMAGES

img = cv2.imread(r'flower.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur=cv2.GaussianBlur(gray,(3,3),cv2.BORDER_DEFAULT)
#more the kernel size, more the blurring
canny=cv2.Canny(blur,125,175)
#cv.imshow("cannyimage",canny)
thinned = cv2.ximgproc.thinning(canny,thinningType=cv2.ximgproc.THINNING_GUOHALL )
cv2.imshow("thin",thinned)

ret,thr = cv2.threshold(gray,125,255,cv2.THRESH_BINARY)
#thr = cv2.threshold(gray, 16, 255, cv2.THRESH_BINARY_INV)[1]
cnts = cv2.findContours(thr, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
cnt = max(cnts, key=cv2.contourArea)
thr = cv2.drawContours(np.zeros_like(thr), [cnt], -1, 255, 1)
#contours,hierarchies=cv2.findContours(thinned,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
list_x=[]
list_y=[]
# Extract and measure edges, and visualize output
out = extract_and_measure_edges(thr)

plt.figure(figsize=(18, 6))
plt.subplot(1, 3, 1), plt.imshow(img), plt.title('Original input image')

plt.subplot(1, 3, 2), plt.imshow(thr, cmap='gray'), plt.title('Contour needed')
plt.subplot(1, 3, 3), plt.imshow(out), plt.title('Results')
plt.tight_layout(), plt.show()

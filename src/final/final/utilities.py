import cv2
import numpy as np

def imfill(image): # function to fill the holes in the image
  cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0] # finding the contours in the image
  for idx,_ in enumerate(cnts): # iterating over each contour
    cv2.drawContours(image, cnts, idx, 255,-1) # filling the contour

def ret_largest_obj(img): # function to return the largest object in the image
    # find the two contours for which we want to find the min distance between them
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0] # finding the contours in the image
    Max_Cntr_area = 0 # initializing the maximum contour area
    Max_Cntr_idx = -1 # initializing the maximum contour index
    for index, cnt in enumerate(cnts): # iterating over each contour
        area = cv2.contourArea(cnt) # finding the area of the contour
        if area > Max_Cntr_area: # if the area of the contour is greater than the maximum contour area
            Max_Cntr_area = area # update the maximum contour area
            Max_Cntr_idx = index # update the maximum contour index
    img_largestobject = np.zeros_like(img) # creating a black image of the same size as the input image
    if (Max_Cntr_idx != -1): # if the maximum contour index is not -1
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, -1) # [ contour = less then min-area contour, contourIDx, Colour , Thickness ]
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, 2) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    return img_largestobject, cnts[Max_Cntr_idx] # return the largest object and the largest contour

def ret_smallest_obj(cnts, noise_thresh = 10): # function to return the smallest object in the image
  min_cntr_area = 1000 # initializing the minimum contour area
  min_cntr_idx= -1 # initializing the minimum contour index
  for index, cnt in enumerate(cnts): # iterating over each contour
      area = cv2.contourArea(cnt) # finding the area of the contour
      if (area < min_cntr_area) and (area > 10): # if the area of the contour is less than the minimum contour area and greater than 10
          min_cntr_area = area # update the minimum contour area
          min_cntr_idx = index # update the minimum contour index
          SmallestContour_Found = True # set the flag to true
  print("min_area" , min_cntr_area) # printing the minimum contour area
  return min_cntr_idx # return the minimum contour index
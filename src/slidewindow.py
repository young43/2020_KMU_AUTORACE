import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import *

class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None

	self.lane_width_hold = 250

    def slidewindow(self, img):

        x_location = None
        # init out_img, height, width        
        out_img = np.dstack((img, img, img)) 
        height = img.shape[0]
        width = img.shape[1]

        # num of windows and init the height
        window_height = 5
        nwindows = 30
        
        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y 
        nonzero = img.nonzero()
        #print nonzero 
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        #print nonzerox
        # init data need to sliding windows
        margin = 20
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []
        
        # first location and segmenation location finder
        # draw line
        # 130 -> 150 -> 180
        pts_left = np.array([[width/2 - 70, height], [width/2 - 70, height - 60], [width/2 - 170, height - 90], [width/2 - 170, height]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
        
	pts_right = np.array([[width/2 + 70, height], [width/2 + 70, height - 80], [width/2 + 170, height - 110], [width/2 + 170, height]], np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)



        # indicies before start line(the region of pts_left)
        # 337 -> 310
        good_left_inds = ((nonzerox >= width/2 - 170) & (nonzeroy >= nonzerox * 0.33 + 300) & (nonzerox <= width/2 - 70)).nonzero()[0]
        good_right_inds = ((nonzerox >= width/2 + 70) & (nonzeroy >= nonzerox * (-0.48) + 500) & (nonzerox <= width/2 + 170)).nonzero()[0]

        # left line exist, lefty current init
        line_exist_flag = None 
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None

        # check the minpix before left start line
        # if minpix is enough on left, draw left, then draw right depends on left
        # else draw right, then draw left depends on right
        if len(good_left_inds) > minpix:
            line_flag = 1
            x_current = np.int(np.mean(nonzerox[good_left_inds]))
            y_current = np.int(np.mean(nonzeroy[good_left_inds]))
            max_y = y_current
        elif len(good_right_inds) > minpix:
            line_flag = 2
            x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            y_current = np.int(np.max(nonzeroy[good_right_inds]))
        else:
            line_flag = 3
            

        if line_flag != 3:
            # it's just for visualization of the valid inds in the region
            for i in range(len(good_left_inds)):
                    img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
            # window sliding and draw
            for window in range(0, nwindows):
                if line_flag == 1: 
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low + self.lane_width_hold, win_y_low), (win_x_high + self.lane_width_hold, win_y_high), (255, 0, 0), 1)
                    # indicies of dots in nonzerox in one square
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    # check num of indicies in square and put next location to current 
                    
  		    if len(good_left_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_left_inds]))
                    elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                        p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_left, win_y_high))
                    # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)


	   	    left_lane_inds.extend(good_left_inds)

                else: # change line from left to right above(if)
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    cv2.rectangle(out_img, (win_x_low - self.lane_width_hold, win_y_low), (win_x_high - self.lane_width_hold, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    
		    if len(good_right_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_right_inds]))
                    elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_right, win_y_high))
 
                    right_lane_inds.extend(good_right_inds)

	    leftx = nonzerox[left_lane_inds]
    	    lefty = nonzeroy[left_lane_inds]
    	    rightx = nonzerox[right_lane_inds]
    	    righty = nonzeroy[right_lane_inds]

	    if lefty != [] and leftx != [] and len(leftx) > len(rightx):
      	        left_fit = np.polyfit(lefty, leftx, 2)
      	        lx_current = np.int(np.polyval(left_fit, 380))
	        x_location = lx_current + int(self.lane_width_hold * 0.5)

	
    	    elif righty != [] and rightx != [] and len(rightx) > 300:
                right_fit = np.polyfit(righty, rightx, 2)
                rx_current = np.int(np.polyval(right_fit, 380))
	        x_location = rx_current - int(self.lane_width_hold * 0.5)
	
	    else:
	        print("no lines")            

        return out_img, x_location

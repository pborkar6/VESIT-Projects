# -*- coding: utf-8 -*-
'''
􁂛 *
􁂛 * Team Id:            eYRC+#1575
􁂛 * Author List:        Namrata Sampat, Pranjali Borkar, Chirag Bafna, Mohit Khatri
􁂛 * Filename:           eYRC+#1575_task6_python.py
􁂛 * Theme:              Caretaker Robot
􁂛 * Functions:          get_perspective_image(frame)
                        dijkstra(graph,src,dest,visited=[],distances={},predecessors={})
                        firebird()
􁂛 * Global Variables:   route_length, route_path, start_o
􁂛 * Variables:          Serial object and Image-    ser, cap, ret, src, img_sq,
                        Pink Stripe Detection-      low_pi, high_pi, lower_pi, higher_pi, mask_pi, pinkstripe, gray_pi, ret_pi, thresh_pi, contours_pi, hierarchy_pi, area_pi, M_pi, cx_pi, cy_pi, 
                        Orange Stripe Detection-    low_or, high_or, lower_or, higher_or, mask_or, orangestripe, gray_or, ret_or, thresh_or, contours_or, hierarchy_or, area_or, M_or, cx_or, cy_or,
                        Updated with each step-     on_board, led_sequence,
                        Miscellaneous-              da_map, x, y, i, j, G_, k,
                        Demand, Storage and Start-  demand_pos, demand, storage_pos, storage, start, start_,
                        Step 1-                     dist_start_str1, path_start_str1, index_start_str1_min,
                        Step 2-                     dist_str1_str2, path_str1_str2, index_str1_str2_min,
                        Step 3-                     on_board_del1, dist_str2_pat1, path_str2_pat1, index_str2_pat1_min,
                        Step 4-                     on_board_del2, dist_pat1_pat2, path_pat1_pat2,
                        Step 5-                     dem_index, dist_pat2_str3, path_pat2_str3,
                        Step 6-                     dem_last, dist_str3_pat3, path_str3_pat3,
                        Compiled steps-             steps_len, steps_path,
                        Coordinates-                x1co_ord, x2co_ord, y1co_ord, y2co_ord
􁂛 *
'''


############################################
## Import OpenCV
import numpy as np
import cv2
import serial
import time
############################################



############################################
## For XBee serial commands via Python program
#creating the serial object ser for connecting to the serial port
#if your com port is com21 put 20 as the com port number
ser = serial.Serial(4) 

#printing ser object attributes: baudrate, open, close etc
print ser

#for changing attributes just try following
ser.baudrate = 9600
############################################



############################################
## Capturing the arena image and performing perspective crop
'''
▪ *
▪ * Function Name:  get_perspective_image
▪ * Input:          frame - image captured from the camera i.e. input image
▪ * Output:         dst - 500 x 440 image after perspective crop
▪ * Logic:          1. Black contours detected
                    2. Biggest polyline detected
                    3. Points remapped from camera image to cropped image
▪ * Example Call:   cap = cv2.VideoCapture(0)
                    ret, src = cap.read()
                    img_sq= get_perspective_image(src)
▪ *
'''
def get_perspective_image(frame):
    #cv2.imshow('frame',frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray',gray)
    lower = np.array([0, 0, 0]) #black color mask
    upper = np.array([120, 120, 120])
    mask = cv2.inRange(frame, lower, upper)
    #res = cv2.bitwise_and(frame, frame, mask = mask)
    ret,thresh1 = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
    #ret,thresh1 = cv2.threshold(res,127,255,cv2.THRESH_BINARY)
    #cv2.imshow('ret',thresh1)
    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(frame,contours,-1,(0,255,0),3)
    biggest = 0
    max_area = 0
    min_size = thresh1.size/4
    index1 = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 10000:
            peri = cv2.arcLength(i,True)
        if area > max_area: 
            biggest = index1
            max_area = area
        index1 = index1 + 1
    approx = cv2.approxPolyDP(contours[biggest],0.05*peri,True)
    #drawing the biggest polyline
    #cv2.polylines(frame, [approx], True, (0,0,255), 3)
    #cv2.imshow('contour',frame)
    x1 = approx[0][0][0]
    y1 = approx[0][0][1]
    x2 = approx[1][0][0]
    y2 = approx[1][0][1]
    x3 = approx[3][0][0]
    y3 = approx[3][0][1]
    x4 = approx[2][0][0]
    y4 = approx[2][0][1]

    #print x1, y1
    #print x2, y2
    #print x3, y3
    #print x4, y4

    
    #points remapped from source image from camera
    #to cropped image try to match x1, y1,.... to the respective near values
    ### To simplify calculations, we have modified the image to 500 x 440 size as the arena itself measures 55" x 44" 
    pts1 = np.float32([[x2,y2],[x4,y4],[x1,y1],[x3,y3]]) 
    pts2 = np.float32([[0,440],[500,440],[0,0],[500,0]])
    persM = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,persM,(500,440))
    return dst


cap = cv2.VideoCapture(0)
ret, src = cap.read()
#cv2.imshow('src', src)
cv2.imwrite("input_image_final.jpg", src)

##getting the perspective image
img_sq= get_perspective_image(src)
#print img_sq.shape
cv2.imwrite("output_image_final.jpg", img_sq)
#cv2.imshow('dst', img_sq)
############################################



## Non-essential step to draw an 18x18 grid on the cropped image
## We have assigned coordinates from 0-323 based on this grid
'''
############################################
## Function to take image as argument and draw nxn grid of red lines on it
def draw_grid(filename,n):
    
    img = cv2.imread(filename)  #read image from file
    #print img.shape
    
    # Loops to draw the grid FOR n>1 ONLY
    i=0
    j=0
    # To start drawing from (x,y) = (0,0)
    x1=0
    x2=0
    y1=0
    y2=0
    
    # Loop to draw vertical lines
    while i<n:
        cv2.line(img, (x1,0), (x1,440), (0,0,255), 2)
        i=i+1
        x1=x1+(500/(n-1))   #500 image size
        print (500/(n-1))
       
    # Loop to draw horizontal lines
    while j<n:
        cv2.line(img, (0,y2), (500,y2), (0,0,255), 2)
        j=j+1
        y2=y2+(440/(n-1))   #440 image size

    cv2.imshow("Image with grid", img)

    return(img)

## Function call


img_ret = draw_grid('output_image_final.jpg',19)
cv2.imwrite("output_image_draw_grid.jpg", img_ret)
############################################
'''



############################################
## Detecting Pink Stripe
low_pi = [150,100,220]
high_pi = [200,170,255]

lower_pi = np.array(low_pi)
upper_pi = np.array(high_pi)

mask_pi = cv2.inRange(img_sq,lower_pi,upper_pi)

pinkstripe = cv2.bitwise_and(img_sq, img_sq, mask= mask_pi)
#cv2.imshow("Pink Stripe",pinkstripe)


# Pink Stripe Binary Image
gray_pi = cv2.cvtColor(pinkstripe,cv2.COLOR_BGR2GRAY)
##cv2.imshow("Pink Stripe - Grayscale",gray_pi)


## Pink Stripe Contour
ret_pi,thresh_pi = cv2.threshold(gray_pi,50,255,0)
##cv2.imshow("Pink Stripe B&W",thresh_pi)

contours_pi, hierarchy_pi = cv2.findContours(thresh_pi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

## Finding and drawing the largest pink contour
area_pi = cv2.contourArea(contours_pi[0])
#print "Pink Stripe area = ", area_pi
if area_pi >= 150:
    M_pi = cv2.moments(contours_pi[0])
    cx_pi = int(M_pi['m10']/M_pi['m00'])
    cy_pi = int(M_pi['m01']/M_pi['m00'])
#print "Centroid of pink stripe = ", cx_pi, ",", cy_pi
############################################


############################################
## Detecting Orange Stripe
low_or = [80,180,220]
high_or = [150,230,255]

lower_or = np.array(low_or)
upper_or = np.array(high_or)

mask_or = cv2.inRange(img_sq,lower_or,upper_or)

orangestripe = cv2.bitwise_and(img_sq, img_sq, mask= mask_or)
#cv2.imshow("Orange Stripe",orangestripe)


# Orange Stripe Binary Image
gray_or = cv2.cvtColor(orangestripe,cv2.COLOR_BGR2GRAY)
##cv2.imshow("Orange Stripe - Grayscale",gray_or)


## Orange Stripe Contour
ret_or,thresh_or = cv2.threshold(gray_or,50,255,0)
##cv2.imshow("Orange Stripe B&W",thresh_or)

contours_or, hierarchy_or = cv2.findContours(thresh_or,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

## Finding and drawing the largest orange contour
area_or = cv2.contourArea(contours_or[0])
print "Orange Stripe area = ", area_or
if area_or >= 150:
    M_or = cv2.moments(contours_or[0])
    cx_or = int(M_or['m10']/M_or['m00'])
    cy_or = int(M_or['m01']/M_or['m00'])
print "Centroid of orange stripe = ", cx_or, ",", cy_or
############################################



############################################
## Orientation Detection Logic
# 0-N;   1-S;   2-E;   3-W
# start = (x,y)
# start_o = orientation
# start_ = 0-323 (explained in the next section)

#Original Configuration
start = (9,3)
start_ = 44

if cx_pi <= cx_or+2 and cx_pi >= cx_or-2:
    if cy_pi > cy_or:
        start_o = 0 #North
    else:
        start_o = 1 #South

elif cy_pi <= cy_or+2 and cy_pi >= cy_or-2:
    if cx_pi > cx_or:
        start_o = 3 #West
    else:
        start_o = 2 #East
############################################



############################################
## Global variables for dijkstra()
route_length=0
route_path=[(0,0)]

## What's on board the robot at any time
on_board = []
## Sequence of LED changes
led_sequence = []

## Map of "coordinates" along which bot is allowed to move
'''
Coordinates assigned as per an 18x18 grid assumed. (0,0)=0; (1,0)=1; .... (0,1)=18 and so on.
Map includes coordinates of all 8 wall divisions.
We are later testing the pixel values to detect the green wall divisions and removing those coordinates from the map.
'''
da_map = [39,40,41,42,43,44,45,46,47,48,58,62,66,67,68,76,80,84,94,98,102,112,113,114,115,116,117,118,119,120,130,134,138,148,152,156,165,166,170,174,175,176,
          184,188,192,202,203,204,205,206,207,208,209,210,220,224,228,238,242,246,256,260,264,274,278,282,283,284,291,292,293,294,295,296,297,298,299,300]
############################################



############################################
## X axis: 1 unit = 27
## Y axis: 1 unit = 24

#centroid of first block
x=13.5
y=12

#initialising loop variables
i=0
j=0

## Generating a new map (dictionary) of allowed coordinates which excludes present wall divisions.
'''
This dictionary will have indices 0-323 (18x18 = 324 coordinates available)
For each index i.e. coordinate, there exist 4 possible neighbours (indices in a nested dictionary)
For each neighbour:
    0 - Neighbour is forbidden
    1 - Neighbour is accessible
'''
G_ = {}

while i<18:
    while j<18:

        #Converting x,y coordinates to 0-323 numbering
        #(0,0) = 0; (0,1) = 18 etc.
        k = j + (18*i)

        #Generating nested dictionary using conditional statements in terms of i(row) & j(col) coordinates
        G_[k] = {(k-1):1 if j>0 else None, (k+1):1 if j<17 else None, (k-18):1 if i>0 else None, (k+18):1 if i<17 else None}

        #Deleting neighbour entry if neighbour does not exist in the da_map or if map point is green (wall division)
        for z,v in G_[k].items():
            
            #for allowed neighbours only
            if k in da_map:
                #for 8 wall division locations only
                if (k==42 or k==46 or k==114 or k==118 or k==204 or k==208 or k==294 or k==298):
                    #if green colour i.e. presence of wall division is detected
                    #print img_sq[y][x]
                    if ((img_sq[y][x][0] <=120) and (img_sq[y][x][1] >= 50) and (img_sq[y][x][2] <= 20)):
                        #deleting the coordinate from allowed coordinates
                        del G_[k][z]            
            else:
                #deleting non-da_map coordinates from allowed coordinates
                del G_[k][z]

        #Updating inner while variable and incrementing x by 1 block i.e. 24
        j=j+1
        x=x+27

    #Updating outer while variable and incrementing y by 1 block i.e. 27
    i=i+1
    y=y+24

    #Reinitialising inner while variable and x value
    j=0
    x=13.5

#print G_
############################################



############################################
## Detecting the location and type of demands
#The two lists will have corresponding indices i.e. the demand type at position demand_pos[0] is stored in demand[0]
demand_pos_=[68,176,284]    #0-323 form
demand = []
for i in range(0,3):

    y=445
    if i==0:
        x=60
    elif i==1:
        x=204
    else:
        x=348

    #print "Demand 0",img_sq[60][y]
    #print "Demand 1",img_sq[204][y]
    #print "Demand 2",img_sq[348][y]
    
    if ((img_sq[x][y][0] >=70 and img_sq[x][y][0] <=150) and (img_sq[x][y][1] >=160 and img_sq[x][y][1] <=230) and (img_sq[x][y][2] >=150 and img_sq[x][y][2] <=200)):
        demand.append('Yellow')
    elif ((img_sq[x][y][0] >=150) and (img_sq[x][y][1] <=120) and (img_sq[x][y][2] <=80)):
        demand.append('Blue')
    elif ((img_sq[x][y][0] <=50) and (img_sq[x][y][1] <=80) and (img_sq[x][y][2] >=130 and img_sq[x][y][2] <=200)):
        demand.append('Red')
    else:
        demand.append(None)
                
print "Demand: ",demand


## Detecting the location and type of storage area
#The two lists will have corresponding indices (Ref: demand)
storage_pos_=[39,165,291]   #0-323 form
storage = []
for n in range(0,3):

    y=67.5
    if n==0:
        x=72
    elif n==1:
        x=228
    else:
        x=372

    #print "Storage 0",img_sq[72][y]
    #print "Storage 1",img_sq[228][y]
    #print "Storage 2",img_sq[372][y]

    if ((img_sq[x][y][0] >=70 and img_sq[x][y][0] <=150) and (img_sq[x][y][1] >=160 and img_sq[x][y][1] <=230) and (img_sq[x][y][2] >=150 and img_sq[x][y][2] <=200)):
        storage.append('Yellow')
    elif ((img_sq[x][y][0] >=150) and (img_sq[x][y][1] <=120) and (img_sq[x][y][2] <=80)):
        storage.append('Blue')
    elif ((img_sq[x][y][0] <=100) and (img_sq[x][y][1] <=100) and (img_sq[x][y][2] >=150 and img_sq[x][y][2] <=255)):
        storage.append('Red')
    else:
        storage.append(None)

print "Storage: ",storage
############################################



############################################
## IMPLEMENTING DIJKSTRA'S ALGORITHM FOR SHORTEST PATH
    ## This is done using a recursive function
    ## Operates on nested dictionary representation of image

'''
▪ *
▪ * Function Name:  dijkstra
▪ * Input:          graph - nested dictionary representation of the map
                    src - start point on graph
                    dest - end point on graph
                    visited - list of visited points to avoid traversing the same point twice
                    distances - dictionary containing distance from source
                    predecessors - dictionary assigning previous point as the new source
▪ * Output:         NONE
▪ * Logic:          <Very complex. Kindly refer to in-function comments>
▪ * Example Call:   dijkstra(G_,20,40,visited=[],distances={},predecessors={})
▪ *
''' 
def dijkstra(graph,src,dest,visited=[],distances={},predecessors={}):

    #Ending condition
    if src == dest:
        #We build the shortest route and display it
        path=[] #Stores path in 0-323 form
        path_coord=[(0,0)]  #Stores path in (x,y) coordinate form
        pred=dest   #predecessor = destination

        while pred != None:
            path.append(pred)
            #Converting "pred" from (0-323) to (x,y)
            x=((pred%18)+1)
            y=((pred/18)+1)
            path_coord.append((x,y))
            pred=predecessors.get(pred,None)
            
        path_coord.reverse()    #Converting dest>src path to src>dest path 
        global route_length
        route_length = distances[dest] #Calculating length of route
            
        global route_path
        route_path = path_coord[0:(distances[dest]+1)]  #Including src in path
            
    else :     
        #Initializing route length for initial run
        if not visited: 
            distances[src]=0
        #Visiting available neighbours
        for neighbor in graph[src] :
            if neighbor not in visited:
                new_distance = distances[src] + graph[src][neighbor]
                if new_distance < distances.get(neighbor,float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src
        #Marking neighbours as visited
        #Necessary to avoid going the same path backwards
        visited.append(src)

        #Recursing as all neighbours are now visited                         
        #Selecting non-visited element with least length 'x'
        #Running "dijskstra" with src='x'

        #Dictionary of unvisited neighours' length
        unvisited={}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k,float('inf'))        
            x=min(unvisited, key=unvisited.get) #x = least length neighbour

        #Recursive function call
        dijkstra(graph,x,dest,visited,distances,predecessors)
####################################################



####################################################
## STEP 1: MOVE FROM START TO FIRST STORAGE AREA

#Calculating distance between start and all storage areas
#Then finding the closest storage

## Reinitialising Global variables
route_length=0
route_path=[(0,0)]
dist_start_str1=[]
path_start_str1=[]

for i in range(0,3):
    if storage[i] in demand:
        # Find the shortest path by calling dijkstra function
        dijkstra(G_,start_,storage_pos_[i],visited=[],distances={},predecessors={})

        dist_start_str1.append(route_length)
        #print dist_start_str1[i]

        path_start_str1.append(route_path)
        #print path_start_str1[i]
    else:
        #Ridiculously high value to avoid it being detected as closest
        #If we don't append anything to the list, the correspondence of the indices across lists will be destroyed
        dist_start_str1.append(500)
        path_start_str1.append(500)

#print "STEP 1 ALL POSSIBLE DISTANCES: ",dist_start_str1
#print "STEP 1 ALL POSSIBLE PATHS: ",path_start_str1


# Finding index of closest storage
index_start_str1_min = dist_start_str1.index(min(dist_start_str1))
# path of closest storage
print "STEP 1 PATH: ",path_start_str1[index_start_str1_min]

# Updating the on_board list
if index_start_str1_min == 0:
    on_board.append(storage[0])
    led_sequence.append(storage[0])
elif index_start_str1_min == 1:
    on_board.append(storage[1])
    led_sequence.append(storage[1])
elif index_start_str1_min == 2:
    on_board.append(storage[2])
    led_sequence.append(storage[2])
####################################################
print "ON BOARD THE ROBOT: ",on_board




####################################################
## STEP 2: MOVE FROM FIRST STORAGE AREA TO SECOND STORAGE AREA

#Calculating distance between first storage area and remaining storage areas
#Then finding the closest storage area

## Reinitialising Global variables
route_length=0
route_path=[(0,0)]
dist_str1_str2=[]
path_str1_str2=[]

for i in range(0,3):
    if (storage[i] in demand) and (storage[i] not in on_board):
        # Find the shortest path by calling dijkstra function
        dijkstra(G_,storage_pos_[index_start_str1_min],storage_pos_[i],visited=[],distances={},predecessors={})

        dist_str1_str2.append(route_length)
        #print dist_str1_str2[i]

        path_str1_str2.append(route_path)
        #print path_str1_str2[i]
    else:
        #Ridiculously high value to avoid it being detected as closest
        #If we don't append anything to the list, the correspondence of the indices across lists will be destroyed
        dist_str1_str2.append(500)
        path_str1_str2.append(500)

#print "STEP 2 ALL POSSIBLE DISTANCES: ",dist_str1_str2
#print "STEP 2 ALL POSSIBLE PATHS: ",path_str1_str2


# Finding index of closest storage
index_str1_str2_min = dist_str1_str2.index(min(dist_str1_str2))
# path of closest storage
print "STEP 2 PATH: ",path_str1_str2[index_str1_str2_min]

# Updating the on_board list
if index_str1_str2_min == 0:
    on_board.append(storage[0])
    led_sequence.append(storage[0])
elif index_str1_str2_min == 1:
    on_board.append(storage[1])
    led_sequence.append(storage[1])
elif index_str1_str2_min == 2:
    on_board.append(storage[2])
    led_sequence.append(storage[2])
####################################################
print "ON BOARD THE ROBOT: ",on_board




####################################################
## STEP 3: MOVE FROM SECOND STORAGE AREA TO FIRST PATIENT

#Calculating distance between second storage area and the two patients
#Then finding the closest patient

## Reinitialising Global variables
route_length=0
route_path=[(0,0)]
dist_str2_pat1=[]
path_str2_pat1=[]


on_board_del1 = []
for i in range(0,3):
    for p in range (0,2):
        if (demand[i] == on_board[p]):     
            on_board_del1.append(i)

#print on_board_del1

for i in range(0,2):
    # Find the shortest path by calling dijkstra function
    dijkstra(G_,storage_pos_[index_str1_str2_min],demand_pos_[on_board_del1[i]],visited=[],distances={},predecessors={})

    dist_str2_pat1.append(route_length)
    #print dist_str2_pat1[i]

    path_str2_pat1.append(route_path)
    #print path_str2_pat1[i]

#print "STEP 3 ALL POSSIBLE DISTANCES: ",dist_str2_pat1
#print "STEP 3 ALL POSSIBLE PATHS: ",path_str2_pat1


# Finding index of closest patient
index_str2_pat1_min = dist_str2_pat1.index(min(dist_str2_pat1))
# path of closest patient
print "STEP 3 PATH: ",path_str2_pat1[index_str2_pat1_min]


# Updating the on_board list
on_board.remove(demand[on_board_del1[index_str2_pat1_min]])
led_sequence.append(demand[on_board_del1[index_str2_pat1_min]])
####################################################
print "ON BOARD THE ROBOT: ",on_board




####################################################
## STEP 4: MOVE FROM FIRST PATIENT TO SECOND PATIENT

#Calculating distance and route between first patient and second patient

for i in range (0,3):
    if demand[i] in on_board:
        on_board_del2 = i
print on_board_del2

# Find the shortest path by calling dijkstra function
dijkstra(G_,demand_pos_[on_board_del1[index_str2_pat1_min]],demand_pos_[on_board_del2],visited=[],distances={},predecessors={})

dist_pat1_pat2 = route_length
path_pat1_pat2 = route_path

#print "STEP 4 DISTANCE: ",dist_pat1_pat2
print "STEP 4 PATH: ",path_pat1_pat2


# Updating the on_board list
on_board.remove(demand[on_board_del2])
led_sequence.append(demand[on_board_del2])
####################################################
print "ON BOARD THE ROBOT: ",on_board




####################################################
## STEP 5: MOVE FROM SECOND PATIENT TO THIRD STORAGE AREA

# Finding the storage index of unfulfilled demand
for i in range(0,3):
    if (dist_str1_str2[i]!=500 and i != dist_str1_str2.index(min(dist_str1_str2))):
        dem_index=i
#print dem_index


#Calculating distance and route from second patient to third storage area
dijkstra(G_,demand_pos_[on_board_del2],storage_pos_[dem_index],visited=[],distances={},predecessors={})

dist_pat2_str3 = route_length
path_pat2_str3 = route_path

#print "STEP 5 DISTANCE: ",dist_pat2_str3
print "STEP 5 PATH: ",path_pat2_str3


# Updating the on_board list
if dem_index == 0:
    on_board.append(storage[0])
    led_sequence.append(storage[0])
elif dem_index == 1:
    on_board.append(storage[1])
    led_sequence.append(storage[1])
elif dem_index == 2:
    on_board.append(storage[2])
    led_sequence.append(storage[2])
####################################################
print "ON BOARD THE ROBOT: ",on_board




####################################################
## STEP 6: MOVE FROM THIRD STORAGE AREA to THIRD PATIENT

# Finding the demand index of unfulfilled demand
for i in range(0,3):
    if (i != on_board_del1[index_str2_pat1_min] and i != on_board_del2):
        dem_last=i
#print dem_last


#Calculating distance and route from third storage area to third patient
dijkstra(G_,storage_pos_[dem_index],demand_pos_[dem_last],visited=[],distances={},predecessors={})

dist_str3_pat3 = route_length
path_str3_pat3 = route_path

#print "STEP 6 DISTANCE: ",dist_str3_pat3
print "STEP 6 PATH: ",path_str3_pat3


# Updating the on_board list
led_sequence.append(on_board[0])
on_board.remove(on_board[0])
####################################################
print "ON BOARD THE ROBOT: ",on_board
print led_sequence




####################################################
## Defining the bot movement function according to the orientation

# ORIENTATION VAR = start_o
# 0-North;  1-South;  2-East;  3-West+

'''
#Original Configuration
start = (9,3)
start_ = 44
start_o = 1
'''


'''
▪ *
▪ * Function Name:  firebird
▪ * Input:          NONE
▪ * Output:         NONE
▪ * Logic:          The cropped image has been divided into an 18x18 grid.
                    Based on the orientation of the Fire Bird V and the (x,y) coordinates relations, motion along coordinates is defined as "Forward", "backward", "Right" and "Left"
                    1 unit along X axis differs from 1 unit along the Y axis.
                    Hence, based on the orientation of the Fire Bird V (whether N-S or E-W), delay for moving one unit in the grid is given accordingly.
▪ * Example Call:   firebird()
▪ *
'''

### DELAYS FOR FULLY-CHARGED FIRE BIRD V ###
def firebird():
    ## ORIENTATION = NORTH
    global start_o
    if start_o == 0:
        if y1co_ord==y2co_ord:
            if x1co_ord > x2co_ord:
                ser.write("8")
                print "Forward"
                time.sleep(0.45)
            if x1co_ord < x2co_ord:
                ser.write("9")
                time.sleep(0.72)
                ser.write("8")
                print "Backward"
                time.sleep(0.45)
                start_o = 1
        if x1co_ord==x2co_ord:
            if y1co_ord > y2co_ord:
                ser.write("9")
                time.sleep(1.05)
                print "Left"
                start_o=3
            if y1co_ord < y2co_ord:
                ser.write("9")
                time.sleep(0.3550)
                print "Right"
                start_o=2
            
    ## ORIENTATION = SOUTH
    if start_o == 1:
        if y1co_ord==y2co_ord:
            if x1co_ord < x2co_ord:
                ser.write("8")
                print "Forward"
                time.sleep(0.45)
            if x1co_ord > x2co_ord:
                ser.write("9")
                time.sleep(0.72)
                ser.write("8")
                print "Backward"
                time.sleep(0.45)
                start_o=0
        if x1co_ord==x2co_ord:
            if y1co_ord < y2co_ord:
                ser.write("9")
                time.sleep(1.05)
                print "Left"
                start_o=2
            if y1co_ord > y2co_ord:
                ser.write("9")
                time.sleep(0.3550)
                print "Right"
                start_o=3
            
    ## ORIENTATION = EAST
    if start_o == 2:
        if y1co_ord==y2co_ord:
            if x1co_ord > x2co_ord:
                ser.write("9")
                time.sleep(0.3550)
                print "Right"
                start_o=1
            if x1co_ord < x2co_ord:
                ser.write("9")
                time.sleep(1.05)
                print "Left"
                start_o=0
        if x1co_ord==x2co_ord:
            if y1co_ord > y2co_ord:
                ser.write("9")
                time.sleep(0.72)
                ser.write("8")
                print "Backward"
                time.sleep(0.462)
                start_o=3
            if y1co_ord < y2co_ord:
                ser.write("8")
                print "Forward"
                time.sleep(0.462)

    ## ORIENTATION = WEST
    if start_o == 3:
        if y1co_ord==y2co_ord:
            if x1co_ord < x2co_ord:
                ser.write("9")
                time.sleep(0.3550)
                print "Right"
                start_o=0
            if x1co_ord > x2co_ord:
                ser.write("9")
                time.sleep(1.05)
                print "Left"
                start_o=1
        if x1co_ord==x2co_ord:
            if y1co_ord < y2co_ord:
                ser.write("9")
                time.sleep(0.72)
                ser.write("8")
                print "Backward"
                time.sleep(0.462)
                start_o=2
            if y1co_ord > y2co_ord:
                ser.write("8")
                print "Forward"
                time.sleep(0.462)
####################################################



####################################################
#Compiling all paths in one list
steps_path= [path_start_str1[index_start_str1_min],
             path_str1_str2[index_str1_str2_min],
             path_str2_pat1[index_str2_pat1_min],
             path_pat1_pat2,
             path_pat2_str3,
             path_str3_pat3]

#Compiling all path lengths in one list
steps_len= [dist_start_str1[index_start_str1_min],
            dist_str1_str2[index_str1_str2_min],
            dist_str2_pat1[index_str2_pat1_min],
            dist_pat1_pat2,
            dist_pat2_str3,
            dist_str3_pat3]
####################################################



####################################################
##The action begins i.e. Using the distances, paths and firebird function to send serial commands to the Fire Bird V to carry out the task.

for i in range(0,6):    #one for each step
    for j in range(0,steps_len[i]): #one per length of the path
        # x1, y1 indicate present position on grid
        # x2, y2 indicate next position on grid
        x1co_ord=steps_path[i][j][1]
        x2co_ord=steps_path[i][j+1][1]
        y1co_ord=steps_path[i][j][0]
        y2co_ord=steps_path[i][j+1][0]
        firebird()

    ser.write("6") #stop when location reached

    ##Code to blink appropriate LEDs
    if i==0:
        if led_sequence[0]=='Yellow':
            ser.write("1") #LED1_GREEN
            time.sleep(2)
            ser.write("1") #LED1_GREEN
        if led_sequence[0]=='Red':
            ser.write("0") #LED1_RED
            time.sleep(2)
            ser.write("0") #LED1_RED
        if led_sequence[0]=='Blue':
            ser.write("2") #LED1_BLUE
            time.sleep(2)
            ser.write("2") #LED1_BLUE
            
    if i==1:
        if led_sequence[1]=='Yellow':
            ser.write("4") #LED2_GREEN
            time.sleep(2)
            ser.write("4") #LED2_GREEN
        if led_sequence[1]=='Red':
            ser.write("3") #LED2_RED_ON
            time.sleep(2)
            ser.write("3") #LED2_RED_ON
        if led_sequence[1]=='Blue':
            ser.write("5") #LED2_BLUE_ON
            time.sleep(2)
            ser.write("5") #LED2_BLUE_ON
            
    if i==2:
        if led_sequence[2]==led_sequence[0]:
            if led_sequence[0]=='Yellow':
                ser.write("1") #LED1_GREEN
                time.sleep(2)
                ser.write("1") #LED1_GREEN
            if led_sequence[0]=='Red':
                ser.write("0") #LED1_RED
                time.sleep(2)
                ser.write("0") #LED1_RED
            if led_sequence[0]=='Blue':
                ser.write("2") #LED1_BLUE
                time.sleep(2)
                ser.write("2") #LED1_BLUE
        else:
            if led_sequence[1]=='Yellow':
                ser.write("4") #LED2_GREEN
                time.sleep(2)
                ser.write("4") #LED2_GREEN
            if led_sequence[1]=='Red':
                ser.write("3") #LED2_RED_ON
                time.sleep(2)
                ser.write("3") #LED2_RED_ON
            if led_sequence[1]=='Blue':
                ser.write("5") #LED2_BLUE_ON
                time.sleep(2)
                ser.write("5") #LED2_BLUE_ON

    if i==3:
        if led_sequence[3]==led_sequence[0]:
            if led_sequence[0]=='Yellow':
                ser.write("1") #LED1_GREEN
                time.sleep(2)
                ser.write("1") #LED1_GREEN
            if led_sequence[0]=='Red':
                ser.write("0") #LED1_RED
                time.sleep(2)
                ser.write("0") #LED1_RED
            if led_sequence[0]=='Blue':
                ser.write("2") #LED1_BLUE
                time.sleep(2)
                ser.write("2") #LED1_BLUE
        else:
            if led_sequence[1]=='Yellow':
                ser.write("4") #LED2_GREEN
                time.sleep(2)
                ser.write("4") #LED2_GREEN
            if led_sequence[1]=='Red':
                ser.write("3") #LED2_RED_ON
                time.sleep(2)
                ser.write("3") #LED2_RED_ON
            if led_sequence[1]=='Blue':
                ser.write("5") #LED2_BLUE_ON
                time.sleep(2)
                ser.write("5") #LED2_BLUE_ON
        
    if i==4 or i==5:
        if led_sequence[4]=='Yellow':
            ser.write("1") #LED1_GREEN_ON
            time.sleep(2)
            ser.write("1") #LED1_GREEN_ON
        if led_sequence[4]=='Red':
            ser.write("0") #LED1_RED_ON
            time.sleep(2)
            ser.write("0") #LED1_RED_ON
        if led_sequence[4]=='Blue':
            ser.write("2") #LED1_BLUE_ON
            time.sleep(2)
            ser.write("2") #LED1_BLUE_ON
    
ser.write("7") #continuous buzzer
ser.close()
####################################################


###  Tada! *looks proudly at Fire Bird V at the finish position* ###

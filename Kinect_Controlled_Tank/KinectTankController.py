#!/usr/bin/env python

from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv2 as cv
import numpy as np
import bluetooth as blt
from inputs import get_gamepad
import time
import sys

###################################################################################################

class KinectController(object):

    _lt = 3         # left track
    _rt = 3         # right track
    _stop = True    # if hands are too far from each other don't move

    #---------------------------------------------------------------------------------------------#

    # Initiallize class/ bluetooth interface
    def __init__(self):
        print("Welcome to Kinect Tank Controller")
        print("Searching for nearby bluetooth devices")
        self._nearby_devices = blt.discover_devices(lookup_names=True)

    #---------------------------------------------------------------------------------------------#

    # Connect with the HC-05/06 bluetooth module
    def Connect(self):
        print("Choose a device to connect")
        # Print the addresses and names of the nearby devices
        for i in range(0, len(self._nearby_devices)):
            print(i, self._nearby_devices[i])

        # Read device number to connect
        nb = int(input('Choose a number: '))

        # Start socket
        self._client_socket = blt.BluetoothSocket(blt.RFCOMM)

        # Connect to device
        self._client_socket.connect((self._nearby_devices[nb][0], 1))

        print("Device :", self._nearby_devices[nb][1], " Connected Successfully !!!")

    #---------------------------------------------------------------------------------------------#

    # Send 'Compressed' speed data to the arduino
    def SendData(self):

        data = (self._lt) * 10 + (self._rt)
        print(self._lt, self._rt, data)
   
        self._client_socket.send(str(data))
    
    #---------------------------------------------------------------------------------------------#

    # Disconnect from the HC-05/06 module and terminate the app
    def Disconnect(self):
        print("Device is disconnecting")
        
        # Ensure that the robot is not moving before disconnecting
        self._lt = 3
        self._rt = 3
        self.SendData()
        
        self._client_socket.close()
        print("Thanks for playing")

    #---------------------------------------------------------------------------------------------#

    # Get Rgb image and depth data from kinect
    def GetDepth(self):
        
        # get data from the sensor
        (depth,_), (rgb,_) = get_depth(), get_video()
        
        # convert data
        depth = depth.astype(np.uint8)
        rgb = cv.cvtColor(rgb, cv.COLOR_RGB2BGR)

        return depth, rgb

    #---------------------------------------------------------------------------------------------#

    # Delete the background from the image
    def Preprocessing(self, src, depth):

        W, H = src.shape[:2]

        gsrc = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

        # Use depth image to delete the background
        d, filt_depth = cv.threshold(depth, 150, 255, cv.THRESH_BINARY_INV)

        fsrc = cv.bitwise_and(gsrc, filt_depth)

        return fsrc

    #---------------------------------------------------------------------------------------------#
    
    # Detect the Human in the image
    def DetectHuman(self, img, fsrc):

        # Use find contours to detect the human   
        im2, contours, hierarchy = cv.findContours(fsrc,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

        hum = []
        img2 = fsrc
        for i in range(0, len(contours)):
            if (len(contours[i]) > 500):
                hum.append(contours[i])
                x,y,w,h = cv.boundingRect(contours[i])
                # cv.rectangle(img,(x, y),(x + w, y + h),(0,255,0),2)
                img2 = fsrc[y: y + h, x: x + w]
                # cv.drawContours(img, contours, i, (255, 0, 0), 3)

        return img, img2, hum

    #---------------------------------------------------------------------------------------------#

    # Detect the user's hands
    def DetectHands(self, img, hum):
        
        Bound_Box = []

        for h in hum:
            
            hd = h[:, :, 1].argmin()    # head index
            lp = h[:, :, 0].argmax()    # left palm index
            rp = h[:, :, 0].argmin()    # right palm index

            if ((lp  / len(h) > 0.2) and (rp  / len(h) < 0.8)):

                body_c_x = 0
                body_c_y = 0

                # Left palm
                lsp = lp - 1         # Left start pointer
                rsp = lp + 1         # Right start pointer
                lp_v = []            # Head average points
                b_b_lp_c = [0, 0]    # Bounding Box center for the Left palm
                sz = 30              # Number of points
                for i in range(0, sz):
                    pl = h[lsp - i] 
                    pr = h[rsp + i]
                    # print(pr.item(0), pr.item(1), type(pr), len(pr))
                    pav_x = int(0.5 * (pl.item(1) + pr.item(1)))
                    pav_y = int(0.5 * (pl.item(0) + pr.item(0)))
                    # cv.circle(img, (pav_y, pav_x), 2, (0,0,255), -1)
                    lp_v.append([pav_y, pav_x])
                    b_b_lp_c[0] += pav_x
                    b_b_lp_c[1] += pav_y

                # Draw Left palm Bounding Rectangle
                d1 = int( 0.3 * np.sqrt((lp_v[0][0] - lp_v[len(lp_v) - 1][0])**2 + (lp_v[0][1] - lp_v[len(lp_v) - 1][1])**2))
                c_y =  int(b_b_lp_c[0] / sz)
                c_x =  int(b_b_lp_c[1] / sz)
                body_c_y += c_y
                body_c_x += c_x
                # cv.rectangle(img, (c_x - d1, c_y - d1), (c_x + d1, c_y + d1), (0, 255, 0), 2)
                # cv.rectangle(img, (c_x + int(1.5 * d1), c_y - 3 * d1), (c_x, c_y - d1), (0, 255, 0), 2)
                Bound_Box.append([c_x - d1, c_y - d1, c_x + d1, c_y + d1])                  # Store the bounding box for the left palm
                # Bound_Box.append([c_x, c_y - 3 * d1, c_x + int(1.5 * d1), c_y - d1])        # Store the bounding box for the left thumb

                # Right palm
                lsp = rp - 1         # Left start pointer
                rsp = rp + 1         # Right start pointer
                rp_v = []            # Average points
                b_b_rp_c = [0, 0]    # Bounding Box center for the Right palm
                sz = sz              # Number of points
                for i in range(0, sz):
                    pl = h[lsp - i] 
                    pr = h[rsp + i]
                    # print(pr.item(0), pr.item(1), type(pr), len(pr))
                    pav_x = int(0.5 * (pl.item(1) + pr.item(1)))
                    pav_y = int(0.5 * (pl.item(0) + pr.item(0)))
                    # cv.circle(img, (pav_y, pav_x), 2, (0,0,255), -1)
                    rp_v.append([pav_y, pav_x])
                    b_b_rp_c[0] += pav_x
                    b_b_rp_c[1] += pav_y

                # Draw Right palm Bounding Rectangle
                d1 = int( 0.3 * np.sqrt((rp_v[0][0] - rp_v[len(rp_v) - 1][0])**2 + (rp_v[0][1] - rp_v[len(rp_v) - 1][1])**2))
                c_y =  int(b_b_rp_c[0] / sz)
                c_x =  int(b_b_rp_c[1] / sz)
                body_c_y += c_y
                body_c_x += c_x
                # cv.rectangle(img, (c_x - d1, c_y - d1), (c_x + d1, c_y + d1), (0, 255, 0), 2)
                # cv.rectangle(img, (c_x - int(1.5 * d1), c_y - 3 * d1), (c_x, c_y - d1), (0, 255, 0), 2)
                Bound_Box.append([c_x - d1, c_y - d1, c_x + d1, c_y + d1])                  # Store the bounding box for the right palm
                # Bound_Box.append([c_x - int(1.5 * d1), c_y - 3 * d1, c_x, c_y - d1])        # Store the bounding box for the right thumb

                # Draw a Bounding Rectangle in the stomach
                body_c_x = int(0.5 * body_c_x)
                body_c_y = int(0.5 * body_c_y)
                d1 = 20
                # cv.rectangle(img, (body_c_x - d1, body_c_y - d1), (body_c_x + d1, body_c_y + d1), (0, 255, 0), 2)
                Bound_Box.append([body_c_x - d1, body_c_y - d1, body_c_x + d1, body_c_y + d1])                  # Store the bounding box for the stomach

        return img, Bound_Box

    #---------------------------------------------------------------------------------------------#

    # Compute the distance between the body and the palms and convert it to rpm
    def BodyPalmDistance(self, depth, BBox, depth2):

        # Calculate the average distance for each of the bounding
        # boxes using the brightness in the depth image

        # Stomach
        box = depth[BBox[2][1]:BBox[2][3], BBox[2][0]:BBox[2][2]]
        st_avg = np.sum(box) / ((BBox[2][3] - BBox[2][1]) * (BBox[2][2] - BBox[2][0]))
        
        # Left palm / Left track speed
        box = depth[BBox[0][1]:BBox[0][3], BBox[0][0]:BBox[0][2]]
        lp_avg = np.sum(box) / ((BBox[0][3] - BBox[0][1]) * (BBox[0][2] - BBox[0][0]))
        self._lt = (lp_avg / st_avg * 5)             
        if (self._lt < 1.5):
            self._lt = 5
        elif (self._lt < 2.5):
            self._lt = 4
        elif (self._lt < 3.5):
            self._lt = 3
        elif (self._lt < 4.5):
            self._lt = 2
        else:
            self._lt = 1


        # Right palm / Right track speed
        box = depth[BBox[1][1]:BBox[1][3], BBox[1][0]:BBox[1][2]]
        rp_avg = np.sum(box) / ((BBox[1][3] - BBox[1][1]) * (BBox[1][2] - BBox[1][0]))
        self._rt = (rp_avg / st_avg * 5)
        if (self._rt < 1.5):
            self._rt = 5
        elif (self._rt < 2.5):
            self._rt = 4
        elif (self._rt < 3.5):
            self._rt = 3
        elif (self._rt < 4.5):
            self._rt = 2
        else:
            self._rt = 1

        # Distance Between the Hands
        p1 = [0.5 * (BBox[0][1] + BBox[0][3]), 0.5 * (BBox[0][0] + BBox[0][2])]
        p2 = [0.5 * (BBox[1][1] + BBox[1][3]), 0.5 * (BBox[1][0] + BBox[1][2])]
        dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        if ((dist > 200) and (dist < 300)):
            self._stop = False
        else:
            self._stop = True
            self._lt = 3    # Zero speed
            self._rt = 3    # Zero speed
        
        # Print the speeds in the image
        cv.putText(depth2, "rpm_l: " + str(self._lt), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
                   1.1, (255, 0, 255), 2, cv.LINE_AA)
        cv.putText(depth2, "rpm_r: " +  str(self._rt), (10, 70), cv.FONT_HERSHEY_SIMPLEX,
                   1.1, (255, 0, 255), 2, cv.LINE_AA)   
        cv.putText(depth2, "stop: " + str(self._stop), (10, 110), cv.FONT_HERSHEY_SIMPLEX,
                   1.1, (255, 0, 255), 2, cv.LINE_AA) 

        return depth2

    #---------------------------------------------------------------------------------------------#

    # 
    def Controller(self):
        self.Connect()

        while 1:
            t0 = time.time()
            depth, rgb = self.GetDepth()

            rgb_c = rgb.copy()
            depth_c = depth.copy()
            del_back = self.Preprocessing(rgb, depth)
            cont, img2, hum = self.DetectHuman(rgb, del_back)
            h_and_p, Bound_Box = self.DetectHands(depth, hum)
            if (Bound_Box):
                h_and_p = self.BodyPalmDistance(depth_c, Bound_Box, rgb)

            cv.namedWindow("Kinect Controller", cv.WINDOW_NORMAL)
            cv.imshow("Kinect Controller", h_and_p)


            # quit program when 'esc' key is pressed
            k = cv.waitKey(5)
            if ((k & 0xFF) == 27):
                break
            elif ((k & 0xFF) == 32):
                cv.imwrite('rgb_c' + str(i) + '.jpg', rgb_c)
                cv.imwrite('depth_c' + str(i) + '.jpg', depth_c)
                print('image saved')
                i += 1
            
            # Sychronize with the Arduino Bluetooth Timeout
            if (time.time() - t0 < 0.05):
                time.sleep(0.05 - (time.time() - t0))
            elif (time.time() - t0 < 0.1):
                time.sleep(0.1 - (time.time() - t0))
            elif (time.time() - t0 < 0.15):
                time.sleep(0.15 - (time.time() - t0))

            # Send the speed data to the arduino
            self.SendData()

        cv.destroyAllWindows()
        self.Disconnect()


    #---------------------------------------------------------------------------------------------#

###################################################################################################
###################################################################################################

if (__name__ == '__main__'):
    kc = KinectController()
    kc.Controller()

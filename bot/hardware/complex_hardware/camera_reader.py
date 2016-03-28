import numpy as np
import time
import subprocess

import zbar
import cv2
from PIL import Image
from PIL import ImageEnhance
import cv2
import math


import bot.lib.lib as lib
from bot.hardware.qr_code import QRCode
from bot.hardware.complex_hardware.QRCode2 import QRCode2
from bot.hardware.complex_hardware.QRCode2 import Block
from bot.hardware.complex_hardware.partial_qr import *



class Camera(object):

    L = 1.5
    qr_verts = np.float32([[-L/2, -L/2, 0],
                            [-L/2,  L/2, 0],
                            [ L/2, -L/2, 0],
                            [ L/2,  L/2, 0]])

    def __init__(self, cam_config):
        self.logger = lib.get_logger()

        udev_name = cam_config["udev_name"]
        print udev_name

        cam_num = self.find_cam_num(udev_name)
        
        # extract calib data from cam_config
        self.a = cam_config["a"]
        self.n = cam_config["n"]
        
        self.cam = cv2.VideoCapture(cam_num)
        self.cam.set(3, 1280)
        self.cam.set(4, 720)
        
        self.resX = int(self.cam.get(3))
        self.resY = int(self.cam.get(4))

        # QR scanning tools
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
        
    def find_name(self, symlink):
            # find where symlink is pointing (/dev/vide0, video1, etc)
        cmd = "readlink -f /dev/" + symlink 
        process = subprocess.Popen(cmd.split(), 
                                   stdout=subprocess.PIPE)
        out = process.communicate()[0]

        #extract ints from name video0, etc
        nums = [int(x) for x in out if x.isdigit()]
        # There should nto be more than one digit
        if len(nums) != 0:
            print "error with camera interfaces"
            return
        interface_num = nums[0]    
        return interface_num        

    def apply_filters(self, frame):
        """Attempts to improve viewing by applying filters """
        # grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        santized_frame = cv2.bilateralFilter(thresh, 9, 75, 75) 
        return santized_frame

    @lib.api_call
    def get_current_frame(self):
        self.cam.grab()        
        self.cam.grab()        
        self.cam.grab()        
        self.cam.grab()       
        return self.cam.read()
 
    def get_zbar_im(self, qr_frame):
        cv2.imwrite('buffer.png', qr_frame)
        pil_im = Image.open('buffer.png').convert('L')
        width, height = pil_im.size
        raw = pil_im.tobytes()

        return zbar.Image(width, height, 'Y800', raw)

    @lib.api_call
    def get_qr_list(self, frame):
        """Recieves frame, assuming it's already been sanitizes, 
        and returns a list of all qr codes in it.
        """
 
        z_im = self.get_zbar_im(frame)
        self.scanner.scan(z_im)
        qr_list = []
        # Gather list of all QR objects
        for symbol in z_im:
            print "qr found"
            tl, bl, br, tr = [item for item in symbol.location]
            points = np.float32(np.float32(symbol.location))
            qr_list.append(QRCode(symbol, self.L))
        return qr_list
 
    @lib.api_call
    def sort_closest_qr(self, qr_list):
        """returns sortest list with closest qr first"""

        for code in qr_list:
            _, code.rvec, code.tvec = cv2.solvePnP(code.verts
                                               , code.points
                                               , self.cam_matrix
                                               , self.dist_coeffs)
            
        # Finds qr with smallest displacement 
        qr_list.sort(key=lambda qr: qr.displacement, reverse=False)
        return qr_list 

    @lib.api_call
    def get_target_qr(self):

        while True:
            ret, frame = self.get_current_frame()
            clean_frame = self.apply_filters(frame)
            z_im = self.get_zbar_im(clean_frame)
            qr_list = self.get_qr_list(z_im)
            sorted_qr = self.sort_closest_qr
            if sorted_qr != None:
                break

    def draw_qr_on_frame(self, frame, qr):
        cv2.line(frame, qr.topLeft, qr.topRight, (20, 20, 255), 8, 8)
        cv2.line(frame, qr.topRight, qr.bottomRight, (20, 20, 255), 8, 8)
        cv2.line(frame, qr.bottomRight, qr.bottomLeft, (20, 20, 255), 8, 8)
        cv2.line(frame, qr.bottomLeft, qr.topLeft, (20, 20, 255), 8, 8)
        return frame

    @lib.api_call 
    def infinite_demo(self):
        while True:
            ret, frame = self.get_current_frame()
            cv2.imshow('frm', frame)
            clean_frame = self.apply_filters(frame)

            found_qrs = self.get_qr_list(clean_frame)
  
            qr_frame = clean_frame
            for qr in found_qrs:
                qr_frame = self.draw_qr_on_frame(qr_frame, qr)
            cv2.imshow('qrs',  qr_frame) 
            print "found_qrs", found_qrs           
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if found_qrs == None or found_qrs == []:
                continue

            sorted_qr = self.sort_closest_qr(found_qrs)
            targetQR = sorted_qr[0]

        self.cam.release()
        cv2.destroyAllWindows

    @lib.api_call
    def QRSweep(self):
        
        QRList = []
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        #cv2.waitKey(50)
        ret, frame = self.cam.read()   
        
        # Direct conversion cv -> zbar images did not work.
        # Buffer file used to have native data structures.

        # PIL -> zbar
        #frame = cv2.bilateralFilter(frame, 9, 75, 75) 
        frame = cv2.GaussianBlur(frame,(5,5),0)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Other possible constants 
        # 19, 0
        frame = cv2.adaptiveThreshold(frame, 255
                                      , cv2.ADAPTIVE_THRESH_MEAN_C
                                      , cv2.THRESH_BINARY, 19, 0)
        
        #ret, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)
        cv2.imwrite('buffer.png', frame)
        pil_im = Image.open('buffer.png').convert('L')

        width, height = pil_im.size
        raw = pil_im.tobytes()
        z_im = zbar.Image(width, height, 'Y800', raw)

        # Find codes in image
        self.scanner.scan(z_im)
        count = 0
        for symbol in z_im:
            tl, bl, br, tr = [item for item in symbol.location]
            points = np.float32([[tl[0], tl[1]],
                                 [tr[0], tr[1]],
                                 [bl[0], bl[1]],
                                 [br[0], br[1]]])

            tvec = self.solveQR(tl, tr, bl, br)

            print symbol.data
            print "X_Displacement = ", tvec[0]
            print "Y_Displacement = ", tvec[1]
            print "Z_Displacement = ", tvec[2]
            print "========================================="

            QRList.append(QRCode2(tvec, symbol.data, tr)) 
            count += 1
        
        #cleanup
        del(z_im)
        del(pil_im)
        del(raw)
        del(frame)

        if count == 0:
            print "No QRCode Found"
            return None
        
        targetQR = self.selectQR(QRList)
        if targetQR == None:
            print "No QRCode Found"
        else: 
            print "value: ", targetQR.value
            print "X:     ", targetQR.tvec[0]
            print "Y:     ", targetQR.tvec[1]
            print "X:     ", targetQR.tvec[2]
        #self.cam.release()
        return targetQR
    
    def selectQR(self, QRList):
        # find the best QRCode to grab (closest x then highest y)
        QRList.sort(key=lambda qr: qr.tvec[0], reverse=False)       # sort the list of qr codes by x, smallest to largest
        x_min = 100
        min_qr = 0
        count = 0
        for qr in QRList:
            #find smallest x
            if abs(qr.tvec[0]) < abs(x_min):
                x_min = qr.tvec[0]
                min_qr = count
            count += 1
        targetQR = None
        #check upper and lower qrs in list for height
        if (min_qr > 0):
            if (abs(QRList[min_qr -1].tvec[0] - x_min) < .1):
                if QRList[min_qr -1].tvec[1] < QRList[min_qr].tvec[1]:
                    targetQR = QRList[min_qr-1]
                else:
                    targetQR = QRList[min_qr]
        if ((min_qr < (len(QRList) - 1)) and (targetQR == None)):
            if (abs(QRList[min_qr +1].tvec[0] - x_min) < .1):
                if QRList[min_qr +1].tvec[1] < QRList[min_qr].tvec[1]:
                    targetQR = QRList[min_qr+1]
                else:
                    targetQR = QRList[min_qr]
        if targetQR == None:
            targetQR = QRList[min_qr]
            
        return targetQR
    
    #new solvepnp
    def solveQR(self, tl, tr, bl, br):
        QRSize = 1.5                # units = inches
        center = [self.resX/2, self.resY/2]

        #find the center of the QRCode
        QRCenter = [int((tl[0] + tr[0] + bl[0] + br[0])/4), int((tl[1] + tr[1] + bl[1] + br[1])/4)]

        #find pixel displacement
        pixel_displacement = [(QRCenter[0] - center[0]),(QRCenter[1] - center[1])]

        #find Z distance to QRCode

        # Find longest edge
        north = int(math.sqrt(pow(abs(tl[0] - tr[0]), 2) + pow(abs(tl[1] - tr[1]), 2)))
        south = int(math.sqrt(pow(abs(bl[0] - br[0]), 2) + pow(abs(bl[1] - br[1]), 2)))
        east = int(math.sqrt(pow(abs(tr[0] - br[0]), 2) + pow(abs(tr[1] - br[1]), 2)))
        west = int(math.sqrt(pow(abs(tl[0] - bl[0]), 2) + pow(abs(tl[1] - bl[1]), 2)))
        if (north >= south and north >= east and north >= west):
            largest_edge = north
        elif (south >= east and south >= west):
            largest_edge = south
        elif (east >= west):
            largest_edge = east
        else:
            largest_edge = west

        displacement = self.getDistance(largest_edge)
        
        #find x and y based on distance
        x_units = (QRSize)*(pixel_displacement[0]/float(largest_edge))
        y_units = -1 * (QRSize)*(pixel_displacement[1]/float(largest_edge))
        
        z_units = math.sqrt(displacement**2 - x_units**2 - y_units**2)

        return [x_units, y_units, z_units, displacement]

    def getDistance(self, length):
        return self.a*math.pow(length, self.n)
    
    #color library
    def check_color(self):
        """
        Looks through the camera and detects blocks size and color.
        returns a Block object of with the size and color data fields.
        """
        largest = None
        print "Getting the frame"
        #get the image
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        ret, bgr = self.cam.read()
        
        print "got the Frame, cropping the image"

        #enhance and show the image with gaussian and colorenhance
        bgr_enhanced = self.enhance_color(bgr)

        # define the list of boundaries
        boundaries = [
            ([0,0,120], [230,50,255]),                          #red bgr  GOOD
            ([220,0,0], [255,150,150]),                         #blue bgr GOOD
            ([0,125,125], [100,255,255]),                       #yellow bgr GOOD
            ([75,120,0], [185,255,75])                          #green: bad
        ]

        # loop over the boundaries
        print "looking for colors"
        count = 0
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower)
            upper = np.array(upper)

            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(bgr_enhanced, lower, upper)
            mask = cv2.erode(mask, None, iterations=10)
            output = cv2.dilate(mask, None, iterations=5)

            #find contours of the masked output
            (cnts,_) = cv2.findContours(output.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

            cnt = 0
            for contour in cnts:
                size = cv2.contourArea(contour)
                if largest != None:
                    if (size > largest.size):
                        largest = Block(size, self.num_to_color(count))
                else:
                    largest = Block(size, self.num_to_color(count))

                center = self.find_contour_center(contour)
                cv2.circle(output, center, 5, (0,0,255), -1) 
                cnt += 1

            count += 1
        
        if  largest == None:
            print  "no color found"
        else:
            print "Color found: ", largest.color
        return largest

    def enhance_color(self, bgr):
        """
        resizes the opencv image and enhances it for color
        """
        size = 640,480
        #crop and enhance the image for color
        rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)

        pil_im = Image.fromarray(rgb)

        pil_im.thumbnail(size, Image.ANTIALIAS)

        w, h = pil_im.size
        w_3 = int(w/3)
        h_6 = int(w/6)

        pil_cropped = pil_im.crop((w_3,h_6,w-w_3, h- h_6))
        converter = ImageEnhance.Color(pil_cropped)
        enhanced = converter.enhance(3.0)
        pil_image = enhanced.convert('RGB') 
        open_cv_image = np.array(pil_image)

        # Convert RGB to BGR 
        rgb_enhanced = open_cv_image[:, :, ::-1].copy()
        bgr_enhanced = cv2.GaussianBlur(rgb_enhanced,(9,9),0)
        return bgr_enhanced

    def num_to_color(self, number):
        if number == 0:
            return "red"
        if number == 1:
            return "blue"
        if number == 2:
            return "yellow"
        if number == 3:
            return "green"
        else:
            return None

    def find_contour_center(self, cnt):
        """
        finds the center of a contour using a bounding circle.
        """
        (x,y), r = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        return center
    
    def partial_qr_scan(self):
        #Capture frame-by-frame
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        self.cam.grab()
        ret, frame = self.cam.read() 

        frame_orig = frame

        # gray and blur
        frame = cv2.GaussianBlur(frame,(0,0),3)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 2)
        
        #edge detect and find contours
        frame = cv2.Canny(frame, 50, 150)
        (cnts, hierarchy) = cv2.findContours(frame.copy()
                                            , cv2.RETR_TREE
                                            , cv2.CHAIN_APPROX_SIMPLE)

        markers = find_markers(cnts, hierarchy)

        # Sort Markers by largest edge
        markers.sort(key=lambda x: largest_edge(x), reverse = True)
        
        #iterate through the markers and find the qr_codes
        qr_list = []
        for mrk in markers:
            markers.remove(mrk)
            matches = [m for m in markers if same_qr(mrk, m)]
            for mtch in matches:
                markers.remove(mtch)

            if len(matches) > 2:
                n = 0
            elif len(matches) == 2:
                qr_list.append(PartialQR(mrk, matches[0], matches[1]))
            elif len(matches) == 1:
                print "Only one match"
                # Find the next marker based on new marker
                found=0
                for m in markers:
                    if same_qr(matches[0], m):
                        matches.append(m)
                        markers.remove(m)
                        found=1
                if found==0:
                    print "error, no matches"
            elif len(matches) == 0:
                n=0

        return qr_list
    
    def partial_qr_select(self, qr_list):
        """
        Takes in an array of partial QRcode objects and looks for the most centered QRCode. 
        Returns a QRcode2 object of the chosen partial qr.
        """
        
        center_x = int(self.resX/2)
        best_x = -10000
        best_y = -10000
        count = 0
        
        if len(qr_list) <= 0 or qr_list == None:
            return None

        else:
            for qr in qr_list:
                x,y = partial_center(qr)
                if best_x < 0:
                    best_x = x
                    best_y = y
                    best = count
                elif abs(center_x - x) < abs(center_x - best_x):
                    best_x = x
                    best_y = y
                    best = count
                count += 1
        
        if best_x < 0:
            return None
        else:
            pqr = qr_list[best]
            avg_length = float((pqr.marker1.length + pqr.marker2.length + pqr.marker3.length)/3.0)
            x_disp = 0.354331 * float(best_x - center_x)/float(avg_length)
            y_disp = 0.354331 * float(best_y/float(avg_length))
            target_qr = QRCode2([x_disp,y_disp,0], None, 0)
            return target_qr
        
        
        



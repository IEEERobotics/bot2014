

"""Encapsulates functionality of moving around robot arm"""
import os

import zbar
from PIL import Image
import cv2
import numpy as np
import time

import bot.lib.lib as lib
from bot.hardware.servo_cape import ServoCape

from bot.hardware.qr_code import QRCode
from bot.hardware.complex_hardware.QRCode2 import QRCode2
from SeventhDOF import Rail_Mover
from bot.hardware.complex_hardware.camera_reader import Camera



class RobotArm(object):

    JUNK_BUFFER = [0]*5
    HOME = [90]*5
    GRAB = 5


    """An object that resembles a robotic arm with n joints"""
    def __init__(self, arm_config):
        
        self.logger = lib.get_logger()
        self.bot_config = lib.get_config()
        
        self.servo_cape \
            = ServoCape(self.bot_config["dagu_arm"]["servo_cape"])     
        # Empty list of zeros representing each joint   
        self.joints = [0]*5
        
        
        # QR scanning tools.
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')

        # Figure out what camera is being used
        cam_model = arm_config["camera"]
        self.cam = Camera(self.bot_config[cam_model])
        self.rail = Rail_Mover()  
        
        # initialize vertices of QR code
        l = 1.5
        self.qr_verts = np.float32([[-l/2, -l/2, 0],
                            [-l/2,  l/2, 0],
                            [ l/2, -l/2, 0],
                            [ l/2,  l/2, 0]])

        # Angles of all of the joints. 
        # DO NOT SEND ANGLES ANY OTHER WAY
        self.joints = self.HOME

        self.hopper = [None, None, None, None]
    @lib.api_call
    def draw_qr_on_frame(self, zbar_dat, draw_frame):

        self.scanner.scan(zbar_dat)
        for symbol in zbar_dat:
            tl, bl, br, tr = [item for item in symbol.location]
            points = np.float32([[tl[0], tl[1]],
                                 [tr[0], tr[1]],
                                 [bl[0], bl[1]],
                                 [br[0], br[1]]])

            cv2.line(draw_frame, tl, bl, (100,0,255), 8, 8)
            cv2.line(draw_frame, bl, br, (100,0,255), 8, 8)
            cv2.line(draw_frame, br, tr, (100,0,255), 8, 8)
            cv2.line(draw_frame, tr, tl, (100,0,255), 8, 8)

        return draw_frame
    @lib.api_call
    def grab(self):
 
        self.servo_cape.transmit_block([5] + self.JUNK_BUFFER)
        
    @lib.api_call   
    def release(self):
        self.servo_cape.transmit_block([6] + self.JUNK_BUFFER)
        
    @lib.api_call
    def simple_center_on_qr(self, target_qr):
        """Attempts to center arm on qr code using only arm itself.
        Only the rotational joints, 
        joint 0 corrects X 
        joint 3 corrects Y
        joint 5 corrects rotation
        """
    @lib.api_call
    def set_angles(self):
        while(1):
            A1 = (input("What is Servo 1's angle?: "))
            if (A1 < 0 or A1> 180):
                print "Error: Given angle is out of range."
                continue
            else:
                break
                
        while(1):
            A2 = (input("What is Servo 2's angle?: "))
            if (A2 < 0 or A2> 180):
                print "Error: Given angle is out of range."
                continue
            else:
                break
                
        while(1):
            A3 = (input("What is Servo 3's angle?: "))
            if (A3 < 0 or A3> 180):
                print "Error: Given angle is out of range."
                continue
            else:
                break
                
        while(1):
            A4 = (input("What is Servo 4's angle?: "))
            if (A4 < 0 or A4> 180):
                print "Error: Given angle is out of range."
                continue
            else:
                break
                
        while(1):
            A5 = (input("What is Servo 5's angle?: "))
            if (A5 < 0 or A5> 180):
                print "Error: Given angle is out of range."
                continue
            else:
                break
                
        while(1):
            answer = raw_input("Do you want to send these angles(y/n)?: ")
            if (answer == 'n'):
                return
            elif (answer == "y"):
                array = [A1,A2,A3,A4,A5]
                self.servo_cape.transmit_block([0] + array)
                return
            else:
                print "Error: Invalid reply. Please answer in y/n format."
        
        
        
    @lib.api_call
    def reset_home_position(self):
        """sets angles back to default position."""
        self.servo_cape.transmit_block([0] + HOME)
        
    @lib.api_call
    def fancy_demo(self):
        os.system('clear')
        print "Welcome to the Team 26: Robotic Arm Mainipulation and Vision demo function."
        print "Demo number      Function       "
        print "1                Input custom angles"
        print "2                Block grap demo"
        print "3                Wave"
        print "4                Stand up straight"
        print "5                Grab"
        print "6                Release"
        print "7                Show range of servos"
        print "8                Initial position"
        print "9                Exit"
        print ""
        while(1):
            demo_number = (input("Please input your desired operation number: "))
            if (demo_number is "exit"):
                return
            elif (demo_number > 9 or demo_number < 0):
                print "Number not within valid range (1-9)."
                continue
            elif (demo_number == 9):
                return
            elif (demo_number == 1):
                self.set_angles()
            else:
                self.demo(demo_number - 1)
        
    @lib.api_call
    def demo(self, demo_number):
        """runs demos 1-7"""
        self.servo_cape.transmit_block([demo_number]
                                         + self.JUNK_BUFFER)        
    
    
    def rail_test(self):
        
        while True:
            #time.sleep(2)
            ret = self.cam.QRSweep()
            if ret != None:
                x_disp = ret.tvec[0]
                print "Checking Alignment with x_disp = ", x_disp
                if abs(x_disp) > .2:
                    self.rail.DisplacementConverter(-1 * x_disp)
            
            ret = None
                
    
    
    def basic_control(self, signal):
        #Start signal recieved
        barge_level = signal
        grab_pos = 0;

        if barge_height == 1:
            grab_pos = self.BARGE_GRAB1
            look_pos = self.BARGE_LOOK1
        elif barge_height == 2:
            grab_pos = self.BARGE_GRAB2
            look_pos = self.BARGE_LOOK2
        elif barge_height == 3:
            grab_pos = self.BARGE_GRAB3
            look_pos = self.BARGE_LOOK3
            
        if self.hopper[0] != None:
            hopper_pos = 1
        elif self.hopper[1] != None:
            hopper_pos = 2
        elif self.hopper[2] != None:
            hopper_pos = 3
        elif self.hopper[3] != None:
            hopper_pos = 4
        else:
            print "Error: Hopper is full."
            return

        #Set the Arm to its default looking position
        self.demo(self.DEFAULT_LOOK)
        #Read from webcam
        time.sleep(2)
        ret = self.camera.readQR()
        
        #No QRs found
        if ret == None:
            time.sleep(2)
            ret = self.readQR()
            if ret == None:
                print "No QRCode Found"
                return

        #Multiple QRs Found
            # Choose one (closest X  then highest Y) -> 1 QR found
            # built into readQR()

        #adjust 7DOF to center on QR
        while(True):
            disp_x = ret.tvec[0]
            print "Checking Alignment with x_disp = ", x_disp
            if abs(x_disp) > .2:
                self.rail.DisplacementConverter(x_disp)
            else:
                break
        
        #extend arm towards block for 2nd check
        self.set_pos(look_pos)
        time.sleep(2)
        
        ret = self.cam.readQR()

        #If not aligned adjust again with 7 DOF
        while(True):
            x_disp = ret.tvec[0]
            print "Checking Alignment with x_disp = ", x_disp
            if abs(x_disp) > .2:
                self.rail.DisplacementConverter(x_disp)
            else:
                break
            
        #Once Aligned: extend arm fully to grab position
        self.demo(grab_pos)
        time.sleep(2)
        
        #Check for QR again if possible (if even needed) then grab
        self.grab()
        time.sleep(1)

        #Pick up and place in empty hopper bin
        self.demo(self.DEFAULT_GRABBED)
        time.sleep(2)

        self.rail.Orientor(hopper_pos)
        time.sleep(1.5)
        self.release()
        time.sleep(1)
        self.hopper[hopper_pos] = ret

        #return to default looking position
        self.demo(DEFAULT)
        time.sleep(1)
        return 1

    
    

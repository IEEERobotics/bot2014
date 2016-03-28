

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
from QRCode2 import QRCode2, Block 

from SeventhDOF import Rail_Mover
from bot.hardware.complex_hardware.camera_reader import Camera
import pyDMCC 

import generic_blocks



JUNK_BUFFER = [0]*5
HOME = [0, 25, 170, 0, 180]
GRAB = 5


class RobotArm(object):

    """An object that resembles a robotic arm with n joints"""
    def __init__(self, arm_config):
        
        self.logger = lib.get_logger()
        self.bot_config = lib.get_config()
        
        self.servo_cape \
            = ServoCape(self.bot_config["dagu_arm"]["servo_cape_arm"])     
        self.servo_cape_grabber \
            = ServoCape(self.bot_config["dagu_arm"]["servo_cape_grabber"])     
        
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
        self.joints = HOME

        self.hopper = [None, None, None, None]

    @lib.api_call
    def get_props(self):
        print "rail pos", self.rail.get_position()
        

    @property
    def joints(self):
        return self.__joints

    @joints.setter
    def joints(self, vals):
        vals =  [int(x) for x in vals]
        # validate values
        if len(vals) == 5:
            self.__joints = vals
        else:
            vals[:] = [0 for i in a if i < 0]
            vals[:] = [180 for i in a if i > 180]
            self.__joints[:len(vals)] = vals
        print "Joints to be sent: ", vals
        self.servo_cape.transmit_block([0] + self.__joints)


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
 
        self.servo_cape_grabber.transmit_block([5] + JUNK_BUFFER)
        
    @lib.api_call   
    def release(self):
        self.servo_cape_grabber.transmit_block([6] + JUNK_BUFFER)

    @lib.api_call
    def set_joints(self, a0=None
                    , a1=None
                    , a2=None
                    , a3=None
                    , a4=None):
        set_vals = [a0, a1, a2, a3, a4]
        for i in range(len(set_vals)):
            if set_vals[i] != None:
                self.joints[i] = set_vals[i]
        self.joints = self.joints
        
    @lib.api_call
    def joint_center_on_qr(self):
        """Attempts to center arm on qr code using only arm itself.
        Only the rotational joints, 
        joint 0 corrects X 
        joint 3 corrects Y
        joint 5 corrects rotation
        """
        
        # Correction constants for P(ID) controller.
        # unlikely that we'll bother using I or D
        p_x = 50
        p_y = 6

        while True:
            ret = self.cam.QRSweep()
            
            # Calculate new vector for change
            if ret != None:
                
                dx = ret.tvec[0]
                dy = ret.tvec[1]
                
                if abs(dx) > 0.1:
                    self.move_rail(p_x * dx)
                if abs(dy) > 0.1:
                    self.joints[3] += p_y * dy
                #print "Joints = ", self.joints
                self.joints = self.joints
                #TODO Find method for calculating rotational oreientation

        return True

    @lib.api_call
    def demo_set_angles(self):
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
        """
        sets angles back to default position. Also resets the position of the 7th DOF
        """
        
        self.servo_cape.transmit_block([0] + HOME)
        self.rail.RunIntoWall()
        
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
                                         + JUNK_BUFFER)        
    
    @lib.api_call    
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
            
    def rail_feedback(self):
        """
        align the arm on the rail to a qrcode and return the code
        """
        count = 0
        direction = 1
        while(True):
            ret = None
            partial_list = self.cam.partial_qr_scan()
            ret = self.cam.partial_qr_select(partial_list)
            if ret != None:
                count = 0                           #reset the count.
                x_disp = ret.tvec[0]
                if abs(x_disp) < .1:
                    return ret
                else:
                    print "Checking Alignment with x_disp = ", x_disp
                    if abs(x_disp) > .1:
                        rail_ret = self.rail.DisplacementConverter(x_disp)
                        if rail_ret == 0:
                            #out of range, reset to middle and try again
                            self.rail.MoveToPosition(3500)
            else:       # if no qrcodes are found
                if count >= 3:
                    count = 0
                    limit = self.rail.DisplacementConverter(1.5*direction)
                    if limit == 0:                  #out of range
                        direction = -1*direction    #reverse direction
                        ret = self.rail.DisplacementConverter(.75*direction)
                        if ret == 0:
                            print "Error: out of range on both ends, shouldn't be possible."
                            
                count += 1
                
    
    def test_look(self):
        self.servo_cape.transmit_block([0] + [0, 125, 0, 170, 0])
    
    def basic_solver(self):
        i = 4
        while(i>0):
            self.joints = HOME
            self.rail.RunIntoWall()
            time.sleep(4)
            self.Tier_Grab('B')
            i= i-1
           
    @lib.api_call 
    def MoveToQR(self):
    
        LOOK = [0, 15, 170, 10, 180]
        time.sleep(1)
        self.servo_cape.transmit_block([0] + HOME)
        time.sleep(2)
        self.rail.DisplacementConverter(3.5)  #get the rail to the middle
        qr = self.rail_feedback()           #position infront of QRCode
        return qr

    def MoveToGenericBlock(self):
        block_dist = 12.5 #adjust to correct block distance from camera
        self.rail.DisplacementMover(3600 - self.rail.rail_motor.position) #goto middle
        for i in xrange(1): #potentially move multiple times to get it right
            img = self.cam.get_current_frame() #needs to be bottom camera
            offsets = generic_blocks.get_lateral_offset(img, block_dist)
            if len(offsets) == 0: return 0
            self.rail.DisplacementConverter(-offsets[0])
        return 1
    
    @lib.api_call
    def Tier_Grab(self, Tier, Case):
           ### Tier is the level of the barge the block is being grabbed from
           ### Case is whether or not a block is on top of another
        #if Tier == 'B' or Tier == 'C':
        #    qr = self.MoveToQR()
         
        if Tier == 'A':
            ## Generic Blocks 
            print "Not coded yet" 
            BLOCK_MOVE_5 = [0, 90, 90, 90, 0]
            BLOCK_GRAB_5 = [0, 90, 90, 90, 0]

        elif Tier == 'B':
            ## Mixed QR Blocks 
            if Case == 1:  ## Block on top
                BLOCK_GRAB_5 = [0, 120, 80, 35, 180]
            elif Case == 2: ## Block on bottom
                BLOCK_GRAB_5 = [0, 120, 110, 55, 180]
                
            LOOK_5 = [0, 25, 170, 10, 180]
            HOPPER1 = [0, 90, 100, 55, 180]
            HOPPER2 = [0, 40, 180, 0, 180]
            

        elif Tier == 'C':
            BLOCK_MOVE_5 = [0, 60, 20, 40, 0]
            BLOCK_GRAB_5 = [0, 0, 10, 50, 0]
            LOOK_5 = [0, 25, 170, 10, 180]
            HOPPER1 = [0, 90, 100, 45, 180]
            HOPPER3 = [0, 40, 180, 0, 180]







        hopper_pos = 5

        if self.hopper[0] == None:
            hopper_pos = 1
        elif self.hopper[1] == None:
            hopper_pos = 2
        elif self.hopper[2] == None:
            hopper_pos = 3
        elif self.hopper[3] == None:
            hopper_pos = 4	
        else:
            print "error~Hopper Full"
            return 0 

        
        self.servo_cape.transmit_block([0] + BLOCK_GRAB_5)
        time.sleep(3)                     #wait for arm to move to location
        self.grab()
        time.sleep(3)                       #wait for arm to grab
        self.servo_cape.transmit_block([0] + HOPPER1)

        time.sleep(3)                     #wait for arm to move to location
        self.rail.Orientor(hopper_pos)
        time.sleep(3)                     #wait for rail to move to bin location
        self.joints = HOME
        time.sleep(3)                       #wait for arm to move to location
        self.servo_cape.transmit_block([0] + HOPPER2)
        time.sleep(3) 

        self.release()
        time.sleep(1) 

        self.hopper[hopper_pos-1] = 1 
        
      

    @lib.api_call
    def FindAndGetBlock(self,color):
        """ 
        Function which takes a given color, and gets that block out of the hopper
        """
        self.reset_home_position() 
        if self.hopper[0] != None:
            if self.hopper[0].value == color:
                self.EmptyHopper(1) 
                
                time.sleep(4)
                
                
        if self.hopper[1] != None:
            if self.hopper[1].value == color: 
                self.EmptyHopper(2)
                
                time.sleep(4)
                
        if self.hopper[2] != None:
            if self.hopper[2].value == color:
                self.EmptyHopper(3)
                
                time.sleep(4)
                
            
        if self.hopper[3] != None:
            if self.hopper[3].value == color:
                self.EmptyHopper(4)
                time.sleep(4)
            	
        else:
            print "No blocks in the hopper" 
            return 0 
        self.reset_home_position()
        return 1 
        
    @lib.api_call
    def EmptyHopper(self,Bin):
        
        
        Hopper = [0,80,173,28,180]
        PullBack = [0,15,170,30,180]
        OffSide = [90,60,110,10,180]
        
      
        
        self.reset_home_position()
        self.rail.Orientor(Bin)
        time.sleep(1)
        self.servo_cape.transmit_block([0] + Hopper)
        time.sleep(3)
        self.grab() 
        time.sleep(2)
        self.servo_cape.transmit_block([0] + PullBack) 
        time.sleep(3) 
        if Bin != 1:
            
            self.rail.Orientor(1) 
        
        self.servo_cape.transmit_block([0] + OffSide) 
        time.sleep(6)
        self.release()
        time.sleep(2)
        
        self.hopper[Bin-1] = None 
        
        return 0 
        
        
    @lib.api_call   
    def check_block_color(self, hopper_pos):
        """
        Takes the hopper posisiton 0-3 as input and will look at the hopper posistion
        to see what clor it is then update the hopper array with the new data.
        """
        HOPPER_LOOK = [0,75,170,10,180]
        #check hopper in array
        if (self.hopper[hopper_pos] != None):
            if (self.hopper[hopper_pos].data != None):
                print "Color already known."
                return 1
        #look at the hopper physically
        self.rail.Orientor(hopper_pos + 1)
        self.joints = HOPPER_LOOK
        time.sleep(3)
        #look for a color
        largest = self.cam.check_color()
        #udate with color found
        if largest != None:
            if self.hopper[hopper_pos] != None:
                self.hopper[hopper_pos] = QRCode2(0,largest.color,0) 
            else:
                self.hopper[hopper_pos].value = largest.color 
        else: 
            print "Error: No color Found."
            
    @lib.api_call 
    def check_box_color(self):
        Look = [90, 80, 170, 15, 180]
        
        self.reset_home_position()
        self.joints = Look
        time.sleep(8)
        largest = self.cam.check_color()
        if largest == None:
            qr = self.cam.QRSweep()
            if qr != None:
                return qr.value
            return 0
        
        return largest.color 
    
    @lib.api_call 
    def competition_solver_barge(self,Tier):
        if Tier == 'A':
            a=0 
        elif Tier == 'B' or Tier == 'C': 
            i = 0
            while i < 2: 
                
                self.MoveToQR()
                
                Position = self.rail.rail_motor.position 
                time.sleep(3)
                self.Tier_Grab(Tier,1) 
                 
                time.sleep(2)
                self.rail.MoveToPosition(Position) 
                time.sleep(2) 
                self.Tier_Grab(Tier,2) 
                self.reset_home_position()
                
                i = i + 1
            self.reset_home_position() 
            return 1 
            
    @lib.api_call 
    def check_hopper(self):
        i = 0
        while i < 4:
            self.check_block_color(i) 
            i = i + 1
        return 1 
        
    @lib.api_call
    def orient(self, pos):
        self.rail.Orientor(pos)

    @lib.api_call
    def move_rail(self, displacement):
        return self.rail.DisplacementMover(displacement)
 
    @lib.api_call
    def GrabQR(self):
        self.cam.QRSweep()
        
    @lib.api_call
    def GrabColor(self):
        self.cam.check_color()
        
    @lib.api_call
    def color_test_loop(self, hopper_pos):
        """
        Takes the hopper posisiton 0-3 as input and will look at the hopper posistion
        to see what clor it is then update the hopper array with the new data.
        
        this function loops to test how color detection works with different lighting conditions
        """
        HOPPER_LOOK = [0,65,170,20,180]
        #check hopper in array
        if (self.hopper[hopper_pos] != None):
            if (self.hopper[hopper_pos].data != None):
                print "Color already known."  
                 
                return 1
        #look at the hopper physically
        self.rail.Orientor(hopper_pos + 1)
        self.joints = HOPPER_LOOK
        time.sleep(3)
        while True:
            #look for a color
            largest = self.cam.check_color()
            #udate with color found
            if largest != None:
                print "Color Found: ", largest.color
                if self.hopper[hopper_pos] != None:
                    self.hopper[hopper_pos] = QRCode2(0,largest.color,0)
                else:
                    #self.hopper[hopper_pos].value = largest.color
                    continue
            else: 
                print "Error: No color Found."
    
    

    

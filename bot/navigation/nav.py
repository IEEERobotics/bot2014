from bot.hardware.IR import IR
from side import Side
from bot.driver.omni_driver import OmniDriver
import bot.lib.lib as lib
from time import sleep
from time import time
import os.path
import yaml

bound = lambda x, l, u: l if x < l else u if x > u else x


class Navigation(object):
    def __init__(self, rail_cars="west"):
        #TODO: Read the PID values from the config and pass in to side 
        # Change parameters for Side()
        # Change the read_values to average filter values
        
        self.config = lib.get_config()
        self.PID_values = self.config["IR_PID"]


        self.device = IR() # INSTANTIATE ONLY ONCE
        self.north = Side("North Left", "North Right",  self.device.read_values, self.PID_values["North"]["diff"], self.PID_values["North"]["dist"])
        self.south = Side("South Left", "South Right",  self.device.read_values, self.PID_values["South"]["diff"], self.PID_values["South"]["dist"])
        self.east = Side("East Top", "East Bottom",  self.device.read_values, self.PID_values["East"]["diff"], self.PID_values["East"]["dist"])
        self.west = Side("West Top", "West Bottom",  self.device.read_values, self.PID_values["West"]["diff"], self.PID_values["West"]["dist"])

        self.driver = OmniDriver()
        self.sides = {"north": self.north,
                      "south": self.south,
                      "west": self.west,
                      "east": self.east}
        self.moving = False
        self.logger = lib.get_logger()
        self.rail_cars_side = rail_cars

    def stop_unused_motors(self, direction):
        direction = direction.lower()
        if direction == "north" or direction == "south":
            self.driver.set_motor("north", 0)
            self.driver.set_motor("south", 0)
        elif direction == "east" or direction == "west":
            self.driver.set_motor("east", 0)
            self.driver.set_motor("west", 0)

    @lib.api_call
    def move_correct(self, direction, side, target, speed, timestep):
        # speed >= 0
        side = side.lower()
        diff_err = self.sides[side].get_diff_correction( timestep)

        # setting speed bounds
        sne = bound(speed-diff_err, -100, 100)
        # sne = -100 if sne < -100 else 100 if sne > 100 else sne
        spe = bound(speed+diff_err, -100, 100)
        #spe = -100 if spe < -100 else 100 if spe > 100 else spe
        #self.logger.info("Error from PID : %d", diff_err)

        dist_err = self.sides[side].get_dist_correction(target, timestep)
        #self.logger.info("dist Error from PID : %d", dist_err)
        dist_err = bound(dist_err, -100, 100)
        if side == "north":
            self.driver.set_motor("east", -dist_err)
            self.driver.set_motor("west", -dist_err)
            if direction == "west":
                self.driver.set_motor("north", -spe)
                self.driver.set_motor("south", -sne)
            if direction == "east":
                self.driver.set_motor("north", sne)
                self.driver.set_motor("south", spe)
        elif side == "south":
            self.driver.set_motor("east", dist_err)
            self.driver.set_motor("west", dist_err)
            if direction == "west":
                self.driver.set_motor("north", sne)
                self.driver.set_motor("south", spe)
            if direction == "east":
                self.driver.set_motor("north", -sne)
                self.driver.set_motor("south", -spe)
        elif side == "east":
            self.driver.set_motor("north", -dist_err)
            self.driver.set_motor("south", -dist_err)
            if direction == "north":
                self.driver.set_motor("west", sne)
                self.driver.set_motor("east", spe)
            elif direction == "south":
                self.driver.set_motor("west", -spe)
                self.driver.set_motor("east", -sne)
        elif side == "west":
            self.driver.set_motor("north", dist_err)
            self.driver.set_motor("south", dist_err)
            if direction == "north":
                self.driver.set_motor("west", spe)
                self.driver.set_motor("east", sne)
            elif direction == "south":
                self.driver.set_motor("west", -sne)
                self.driver.set_motor("east", -spe)
        else:
            raise Exception()

    @lib.api_call
    def move_dead(self, direction, speed):
        direction = direction.lower()
        dirs = { "north": 0, "west": 90, "south": 180, "east": 270 }
        self.driver.move(speed, dirs[direction])
        self.moving = True

    @lib.api_call
    def drive_dead(self, direction, speed, duration):
        self.move_dead(direction, speed)
        sleep(duration)
        self.stop()

    @lib.api_call
    def drive_along_wall(self, direction, side, duration):
        time_elapsed = time()
        final_time = time_elapsed + duration
        while time_elapsed < final_time:
            timestep = time()-time_elapsed
            time_elapsed = time()
            self.move_correct(direction, side, 300, 50, timestep)
            sleep(0.01)
        self.stop()

    @lib.api_call
    def test(self):
        #self.drive_along_wall("west", "north", 5)
        self.move_until_wall("north","east", 400)

    #TODO(Vijay): This function is buggy. Needs to be fixed.
    @lib.api_call
    def move_until_wall(self, direction, side, target, dist=150):
        direction = direction.lower()
        mov_side = self.sides[direction]
        mov_target = dist
        self.moving = True
        time_elapsed = time()
        while self.moving:
            timestep = time() - time_elapsed
            time_elapsed = time()
            self.move_correct(direction, side, mov_target, 70, timestep)
            if mov_side.get_distance() <= target:
                self.stop()

    def move_to_position(self, x, y):
        self.move_until_wall(self, "west", "north", x)
        sleep(0.5)
        self.move_until_wall(self, "north", "west", y)
        sleep(0.5)

    @lib.api_call
    def stop(self):
        self.driver.move(0)
        self.moving = False
        
    @lib.api_call
    def set_PID_values(self, side_to_set, pid, kp, kd, ki):
        set_side = self.sides[side_to_set]
        if (pid == "diff"):
            set_side.diff_pid.set_k_values(kp, kd, ki)
        elif(pid =="dist"):
            set_side.dist_pid.set_k_values(kp, kd, ki)
        # write updated PID values to the IR_config file
        #with open("IR_config.yaml") as f:
        #    a = yaml.load(f)
        #a["IR_PID"][side_to_set][pid] = [kp, kd, ki]
        #with open("IR_config.yaml", "w") as f:
        #    yaml.dump(a, f)
    
    @lib.api_call
    def read_IR_values(self):
        return self.device.read_values()
    
    
    @lib.api_call
    def move_until_color(self, direction, side, target, color):
        direction = direction.lower()
        mov_side = self.sides[direction]
        mov_target = 300
        self.moving = True
        time_elapsed = time()
        while self.moving:
            timestep = time() - time_elapsed
            time_elapsed = time()
            self.move_correct(direction, side, mov_target, 50, timestep)
            ir_values = mov_side.get_values()
            # IR sensor for line detection is attached to South Left
            ir_value = ir_values["South Left"]
            if color == "white":
                if ir_value >= 1000:
                    self.stop()
            else:
                if ir_value <= 1000:
                    self.stop()


    @lib.api_call
    def rotate_start(self):
        ir_values = self.device.read_values()
        ir_diff = abs(ir_values["East Bottom"] - ir_values["East Top"])
        while ( ir_diff > 20):
            if ir_values["East Bottom"] < ir_values["East Top"]:
                if ir_values["West Bottom"] > ir_values["West Top"]:
                #clockwise
                    self.driver.rotate_t(-60,.1)
                    sleep(0.1)
            elif ir_values["East Bottom"] > ir_values["East Top"]:
                if ir_values["West Bottom"] < ir_values["West Top"]:
                # counter clockwise
                    self.driver.rotate_t(60,.1)
                    sleep(0.1)
            else:
                break
            ir_values = self.device.read_values()
            ir_diff = abs(ir_values["East Bottom"] - ir_values["East Top"])
        

    def goto_top(self):
        if self.east.get_distance() < MAX_VALUE:
            self.move_until_wall("north", "east", 100)
        elif self.west.get_distance() < MAX_VALUE:
            self.move_until_wall("north", "west", 100)

    @lib.api_call
    def goto_railcar(self):
        self.goto_top()

        if self.rail_cars_side == "west":
            self.move_until_wall("west", "north", 100)
        elif self.rail_cars_side == "east":
            self.move_until_wall("east", "north", 100)

    #TODO: Make a gotoBoat function
    # go north towards block, then towards rail cars and straight down
    @lib.api_call
    def goto_boat(self):
        self.goto_railcar()

        if self.rail_cars_side == "west":
            self.move_until_wall("south", "west", 200)
        elif self.rail_cars_side == "east":
            self.move_until_wall("south", "east", 200)

    @lib.api_call
    def goto_truck(self):
        self.goto_top()

        if self.rail_cars_side == "west":
            self.move_until_wall("east", "north", 150)
        if self.rail_cars_side == "east":
            self.move_until_wall("west", "north", 150)

        self.move_until_wall("south", "south", 150)


    @lib.api_call
    def goto_block_zone_A(self):
        self.goto_railcar()

    def goto_block_zone_B(self):
        pass

    @lib.api_call
    def goto_block_zone_C(self):
        self.goto_top()

        if self.rail_cars_side == "west":
            self.move_until_wall("east", "north", 100)
        if self.rail_cars_side == "east":
            self.move_until_wall("west", "north", 100)

    
    #TODO: Make a getIrSensorValue function
    # find a value of a specific sensor
        
    @lib.api_call
    def set_bias(self, side, bias):
        side = side.replace("_", " ")
        self.device.set_bias(side,bias)
        # write updated bias value to IR_config file
        #with open("IR_config.yaml") as f:
        #    a = yaml.load(f)
        #a["IR_Bias"][side] = bias
        #with open("IR_config.yaml", "w") as f:
        #    yaml.dump(a, f)

    @lib.api_call
    def test_nav(self):
        self.driver.drive(80, 45, 0.5)
        self.logger.info("Climbed the tunnel")
        sleep(0.1)
        self.rotate_start()
        self.logger.info("Auto-corrected inside tunnel")
        sleep(0.1)
        self.move_until_wall("north","east", 500)
        self.logger.info("Reached the barge")
        sleep(0.1)
        self.move_until_color

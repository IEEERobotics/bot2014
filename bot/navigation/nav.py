from bot.hardware.IR import IR
from side import Side
from bot.driver.omni_driver import OmniDriver
import bot.lib.lib as lib
from time import sleep
from time import time
from pid import PID
import os.path
import yaml

from bot.hardware.complex_hardware import Camera

bound = lambda x, l, u: l if x < l else u if x > u else x

MAX_VALUE = 800

class Navigation(object):
    def __init__(self, rail_cars=0):
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
        mapping = ["EXIT", "west", "east", "EXIT"]
        self.rail_cars_side = mapping[rail_cars]

        # camera for checking block
        c_config = self.config["generic_cam"]
        self.cam = Camera(c_config)

    @lib.api_call
    def check_current_car(self, expected_color):
        """ Verifies that car is currenly facing a bin"""
        frame = self.cam.get_current_frame()
        qr_list = self.cam.get_qr_list(frame)
        
        # Should not happen unless far away
        if len(qr_list) > 2:
            return False
        
        # assure that at least 2 are same
        colors = {"red":0, "blue": 0, "green":0, "yellow":0}
        for q in qr_list:
            colors[q.value] += 1

        if any( i >= 2 for i in colors.itervalues()):
            return True

        # If none of these conditions met, assum not lined up
            return False        


    def stop_unused_motors(self, direction):
        direction = direction.lower()
        if direction == "north" or direction == "south":
            self.driver.set_motor("north", 0)
            self.driver.set_motor("south", 0)
        elif direction == "east" or direction == "west":
            self.driver.set_motor("east", 0)
            self.driver.set_motor("west", 0)

    @lib.api_call
    def move_correct(self, direction, side, target, speed, timestep, threshold=1000000):
        # speed >= 0
        side = side.lower()
        diff_err = self.sides[side].get_diff_correction( timestep, threshold)

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
                self.driver.set_motor("north", sne)
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

    def move_dead(self, direction, speed):
        direction = direction.lower()
        dirs = {"north": 0, "west": 90, "south": 180, "east": 270}
        self.driver.move(speed, dirs[direction])

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
            if direction == "west" or direction == "east": 
                self.move_correct(direction, side, 300, 65, timestep)
            else:
                self.move_correct(direction, side, 300, 50, timestep)
            sleep(0.01)
        self.stop()


    @lib.api_call
    def test(self):
        #self.drive_along_wall("west", "north", 5)
        self.move_until_wall("north", "east", 400)

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
            self.move_correct(direction, side, mov_target, 60, timestep)
            if mov_side.get_distance() <= target:
                self.stop()

    #TODO: Update the controller in this function
    @lib.api_call
    def move_smooth_until_wall(self, direction, side, target, dist=150, t_type="avg"):
        direction = direction.lower()
        mov_side = self.sides[direction]
        mov_target = dist
        self.moving = True
        time_elapsed = time()
        speed = 0
        speed_pid = PID()
        speed_pid.set_k_values(4, 0.01, 0)
        while self.moving:
            timestep = time() - time_elapsed
            time_elapsed = time()
            speed = speed_pid.pid(0, target - mov_side.get_distance(), timestep)
            if direction == "east" or direction == "west":
                speed = bound(speed, -65, 65)
            else:
                speed = bound(speed, -65, 65)

            self.move_correct(direction, side, mov_target, speed, timestep)
            if mov_side.get_distance(t_type) <= target:
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
        elif(pid == "dist"):
            set_side.dist_pid.set_k_values(kp, kd, ki)
        # write updated PID values to the IR_config file
        # with open("IR_config.yaml") as f:
        #    a = yaml.load(f)
        # a["IR_PID"][side_to_set][pid] = [kp, kd, ki]
        # with open("IR_config.yaml", "w") as f:
        #    yaml.dump(a, f)

    @lib.api_call
    def read_IR_values(self):
        return self.device.read_values()

    @lib.api_call
    def move_until_color(self, direction, side, color):
        direction = direction.lower()
        mov_side = self.sides[direction]
        mov_target = self.sides[side].get_distance()
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
        # If North side is facing inner wall of tunnel rotate until East top can see inner wall.
        while ir_values["East Bottom"] > 200 and ir_values["East Top"] > 200:
            # counter clockwise
            self.driver.rotate_t(60, .1)
            sleep(.1)
            ir_values = self.device.read_values()
        self.logger.info("Now straightening out")

        # Straighten out inside the tunnel
        ir_values = self.device.read_values()
        ir_diff = abs(ir_values["East Bottom"] - ir_values["East Top"])
        while (ir_diff > 20):
            if ir_values["East Bottom"] < ir_values["East Top"]:
                # clockwise
                self.driver.rotate_t(-60, .1)
                sleep(0.1)
            elif ir_values["East Bottom"] > ir_values["East Top"]:
                # counter clockwise
                self.driver.rotate_t(60, .1)
                sleep(0.1)
            else:
                break
            ir_values = self.device.read_values()
            ir_diff = abs(ir_values["East Bottom"] - ir_values["East Top"])

    @lib.api_call
    def goto_top(self):
        if self.east.get_distance() < MAX_VALUE:
            self.move_smooth_until_wall("north", "east", 300)
        elif self.west.get_distance() < MAX_VALUE:
            self.move_smooth_until_wall("north", "west", 300)

    @lib.api_call
    def goto_railcar(self):
        self.goto_top()
        self.logger.info("Currently near barge")


        if self.rail_cars_side == "west":
            self.logger.info("Going west towards railcars")
            self.move_smooth_until_wall("west", "north", 200, t_type="min")
        elif self.rail_cars_side == "east":
            self.logger.info("Going east towards railcars")
            self.move_smooth_until_wall("east", "north", 200, t_type="min")

    # TODO: Make a gotoBoat function
    # go north towards block, then towards rail cars and straight down
    def goto_boat(self):
        self.goto_railcar()

        if self.rail_cars_side == "west":
            self.move_until_wall("south", "west", 200)
        elif self.rail_cars_side == "east":
            self.move_until_wall("south", "east", 200)

    def goto_truck(self):
        self.goto_top()

        if self.rail_cars_side == "west":
            self.move_until_wall("east", "north", 150)
        if self.rail_cars_side == "east":
            self.move_until_wall("west", "north", 150)

        self.move_until_wall("south", "south", 150)

    def goto_block_zone_A(self):
        self.goto_railcar()

    @lib.api_call
    def goto_block_zone_B(self):
        self.goto_top()
        self.logger.info("sensor value: %d",self.east.get_distance())
        self.logger.info("sensor value: %d", self.west.get_distance())
        if self.east.get_distance() < MAX_VALUE:
            self.logger.info("I'm on right side of course. Going to white line on my left")
            self.move_until_color("west", "north", "white")
        elif self.west.get_distance() < MAX_VALUE:
            self.logger.info("I'm on left side of course. Going to white line on my right")
            self.move_until_color("east", "north", "white")
        self.bang()
        self.logger.info("Reached Zone B")

    @lib.api_call
    def bang(self, side="north"):
        self.drive_dead(side, 50, 0.5)

    @lib.api_call
    def get_off_wall(self):
        self.drive_dead("south", 50, 0.333)

    @lib.api_call
    def correct_bang(self):
        self.get_off_wall()
        self.drive_dead("north", 50, 0.7)
        self.logger.info("Aligned with barge")

    def goto_block_zone_C(self):
        self.goto_top()

        if self.rail_cars_side == "west":
            self.move_until_wall("east", "north", 100)
        if self.rail_cars_side == "east":
            self.move_until_wall("west", "north", 100)

    @lib.api_call
    def set_bias(self, side, bias):
        side = side.replace("_", " ")
        self.device.set_bias(side,bias)

    @lib.api_call
    def get_sensor_value(self, value):
        if "_" in value:
            value = value.replace("_", " ")
        try:
            return self.read_IR_values()[value]
        except KeyError:
            self.logger.warning("Invalid Key for IR Values %s"%value)

    #TODO: to be tested
    @lib.api_call
    def goto_next_railcar(self):
        def avg(vals):
            return sum(vals)/float(len(vals))
        self.moving = True
        speed = 50
        sensor = "West Bottom"
        last_value = self.get_sensor_value(sensor)
        self.logger.info("sensor value: %d", last_value)
        last_set = [last_value for i in xrange(10)]
        time_elapsed = time()
        self.move_dead("south", speed)
        while self.moving:
            timestep = time() - time_elapsed
            time_elapsed = time()
            curr_value = self.get_sensor_value(sensor)
            self.logger.info("sensor Type: %s, sensor value: %d, avg: %d", sensor, curr_value, avg(last_set))
            diff = curr_value - avg(last_set)
            self.move_correct("south", self.rail_cars_side, 150, speed, timestep, threshold=100)
            if diff > 50:
                break
                if sensor == "West Bottom":
                    sensor = "West Top"
                    speed = 35
                    last_set = [curr_value for i in xrange(10)]
                else:
                    self.moving = False
                    break
            last_set.pop(0)
            last_set.append(curr_value)
            sleep(0.01)
        self.stop()
        
    @lib.api_call
    def drive_through_tunnel(self):
        self.move_through_tunnel(-75 ,-75 ,75 ,90 ,.8)
        sleep(.8)
        self.logger.info("Climbed the tunnel")
        self.rotate_start()
        self.logger.info("Auto-corrected inside tunnel")
        sleep(0.8)
        if self.rail_cars_side == "west":
            self.move_smooth_until_wall("north", "east", 500)
        else:
            self.move_smooth_until_wall("north", "west", 500)
        self.logger.info("Reached the barge")
        sleep(0.8)
        
    @lib.api_call
    def move_s(self, north=-100, south=-100, west=80, east=80):
        self.driver.set_motor("east", east)
        self.driver.set_motor("north", north)
        self.driver.set_motor("south", south)
        self.driver.set_motor("west", west)
        
    @lib.api_call
    def move_through_tunnel(self, north=-100, south=-100, west=80, east=80, duration=.75):
        self.move_s(north,south,west,east)
        sleep(duration)
        self.stop()

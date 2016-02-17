import smbus
from time import sleep
import bot.lib.lib as lib


class IR(object):
    """ interface for IR rangefinder."""

    def __init__(self):
        self.config = lib.get_config()
        self.bus = smbus.SMBus(1)
        self.hash_values = self.config["IR"]
        self.irDistancesFilt = [0] *10

    def parse_packets(self, msg):
        """ Return the 20 bytes of data from the IR Rangefinders.
        :params msg:
        :value: int
        :returns: Boolean value indicating success.

        """
        first = msg[::2]
        second = msg[1::2]
        data = [second[i] * 256 + first[i] for i in xrange(len(first))]
        return data

    @lib.api_call
    def read_values(self):
        ms = range(20)
        for i in range(20):
            ms[i] = self.bus.read_byte(8)
        data = self.parse_packets(ms)
        data[3] = data[3] - 40
        data = [((i ** -1.55) * 2000000 if i != 0 else 0) for i in data]
        return_dict = {}
        for j in self.hash_values:
            return_dict[j] = int(data[self.hash_values[j] - 1])
        return return_dict
        
     
    def moving_average_filter(self, ):
        movingAVG_N = 4
        irDistances = self.read_values()
        i = 0
        for side in irDistances:
            self.irDistancesFilt[i] = self.irDistancesFilt[i] << movingAVG_N - self.irDistancesFilt[i]
            self.irDistancesFilt[i] += irDistances[side]
            self.irDistancesFilt[i] >>= movingAVG_N
            irDistances[side] = self.irDistancesFilt[i]
            i += 1
            
            
            
            

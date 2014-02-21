"""Abstraction layer for DMCC-based motors."""

from collections import namedtuple

dmcc_testing = False  # a hard testing flag, if DMCC is not found
try:
    import DMCC
except ImportError:
    print "DMCC ImportError; forcing test mode."
    dmcc_testing = True

import lib.lib as lib


# TODO: Move this to a util module?
class MinMaxRange(namedtuple('MinMaxRange', ['min', 'max'])):
    """Represents a simple min-max range, with some handy functions."""

    def check(self, value):
        return self.min <= value <= self.max

    def clamp(self, value):
        if value < self.min:
            return self.min
        elif value > self.max:
            return self.max
        else:
            return value


class DMCCMotor(object):
    """A motor controlled by a DMCC cape."""

    board_num_range = MinMaxRange(min=0, max=3)  # 4 boards
    motor_num_range = MinMaxRange(min=1, max=2)  # 2 motors each
    power_range = MinMaxRange(min=-10000, max=10000)  # raw PWM value
    position_range = MinMaxRange(min=-(1 << 31), max=(1 << 31) - 1)  # int
    velocity_range = MinMaxRange(min=-(1 << 15), max=(1 << 15) - 1)  # short

    def __init__(self, board_num, motor_num):
        """Initialize a DMCC motor, given board (cape) and motor numbers.

        :param board_num: Board number in board_num_range.
        :type board_num: int

        :param motor_num: Motor number in motor_num_range.
        :type motor_num: int

        """
        self.config = lib.get_config()
        self.logger = lib.get_logger()
        self.is_testing = self.config["testing"] or dmcc_testing
        # TODO: Optimize testing setup, with dummy DMCC module?

        assert (self.board_num_range.check(board_num) and
                self.motor_num_range.check(motor_num))
        self.board_num = board_num
        self.motor_num = motor_num

        self._power = 0  # last set power; DMCC can't read back power (yet!)
        if self.is_testing:
            self._position = 0  # last set position, only when testing
            self._velocity = 0  # last set velocity, only when testing
        self.logger.debug("Setup {}".format(self))

    @property
    def voltage(self):
        """Return the motor supply voltage connected to the board.

        :returns: Motor supply voltage (volts).

        """
        if self.is_testing:
            return 0.0
        return DMCC.getMotorVoltage(self.board_num)

    @property
    def power(self):
        """Return the current motor power.

        :returns: Motor power in power_range (raw PWM value).

        """
        return self._power

    @power.setter
    def power(self, value):
        """Set the power level of this motor.

        :param value: Desired motor power in power_range (raw PWM value).
        :type value: int

        :returns: Boolean value indicating success.

        """
        value = self.power_range.clamp(value)
        self._power = value  # if someone asks later on
        if self.is_testing:
            return True
        return DMCC.setMotor(self.board_num, self.motor_num, self._power) == 0

    @property
    def position(self):
        """Return the current motor position (encoder ticks).

        :returns: Motor position in position_range (ticks).

        """
        if self.is_testing:
            return self._position
        return DMCC.getQEI(self.board_num, self.motor_num)
        # TODO: Ensure getQEI() actually returns ticks in position_range
        # TODO: Ensure setTargetPos() and getQEI() operate on the same units

    @position.setter
    def position(self, value):
        """Set the target position of this motor (encoder ticks).

        :param value: Desired motor position in position_range (ticks).
        :type value: int

        :returns: Boolean value indicating success.

        """
        value = self.position_range.clamp(value)
        if self.is_testing:
            self._position = value
            return True
        return DMCC.setTargetPos(
            self.board_num, self.motor_num, value) == 0

    @property
    def velocity(self):
        """Return the current motor velocity (encoder ticks per unit time).

        :returns: Motor velocity in velocity_range (ticks per sec).

        """
        if self.is_testing:
            return self._velocity
        return DMCC.getQEIVel(self.board_num, self.motor_num)
        # TODO: Ensure getQEIVel() actually returns ticks/sec in velocity_range
        # TODO: Ensure setTargetVel() and getQEIVel() operate on the same units

    @velocity.setter
    def velocity(self, value):
        """Set the target velocity of this motor (encoder ticks per sec).

        :param value: Desired motor velocity in velocity_range (ticks per sec).
        :type value: int

        :returns: Boolean value indicating success.

        """
        value = self.velocity_range.clamp(value)
        if self.is_testing:
            self._velocity = value
            return True
        return DMCC.setTargetVel(
            self.board_num, self.motor_num, value) == 0

    def setPID(self, target, P, I, D):
        """Set PID target parameter (position or velocity) and constants.
        NOTE: Call specialized methods setXXXPID() instead of this directly.

        :param target: Desired parameter to control (0=position, 1=velocity).
        :type target: int
        :param P: Proportional gain constant.
        :type int:
        :param I: Integral gain constant.
        :type int:
        :param D: Differential gain constant.
        :type int:

        :returns: Boolean value indicating success.

        """
        if self.is_testing:
            return True
        return DMCC.setPIDConstants(self.board_num, self.motor_num,
                                    target, P, I, D) == 0

    def setPositionPID(self, P, I, D):
        """Set PID constants to control position [see setPID()]."""
        return self.setPID(0, P, I, D)

    def setVelocityPID(self, P, I, D):
        """Set PID constants to control velocity [see setPID()]."""
        return self.setPID(1, P, I, D)

    def __str__(self):
        """Returns basic motor ID information as a string."""
        # NOTE: Properties (position, velocity) would be slow to read
        return "{}: {{ board_num: {}, motor_num: {} }}".format(
            self.__class__.__name__, self.board_num, self.motor_num)
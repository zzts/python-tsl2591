'''
This code is basically an adaptation of the Arduino_TSL2591 library from 
adafruit: https://github.com/adafruit/Adafruit_TSL2591_Library

for configuring I2C in a raspberry 
https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c

datasheet: 
http://ams.com/eng/Products/Light-Sensors/Light-to-Digital-Sensors/TSL25911

'''
import smbus
import time

VISIBLE = 2  # channel 0 - channel 1
INFRARED = 1  # channel 1
FULLSPECTRUM = 0  # channel 0

ADDR = 0x29

COMMAND_BIT = 0xA0  # bits 7 and 5 for 'command normal'

SPECIAL_FUNCTION = 0xE0 # bits 7-5 for special function ie interupt control
# special function flags
FORCE_INT = 0x04 # forces an interrupt
CLEAR_ALS_INT = 0x06  # Clears any pending ALS interrupt (write 1 to clear)
CLEAR_INTS = 0x07 # Clears and pending ALS and No Persit interrupts
CLEAR_NPALS_INT = 0x0A # Clear any pending No Persit interrupts

WORD_BIT = 0x20  # 1 = read/write word (rather than byte)
BLOCK_BIT = 0x10  # 1 = using block read/write

ENABLE_POWERON = 0x01
ENABLE_POWEROFF = 0x00
ENABLE_AEN = 0x02 # ALS enable
ENABLE_AIEN = 0x10 # ALS interrupt uses persist filter
ENABLE_SAI = 0x40 # Sleep after interrupt
ENABLE_NPIEN = 0x80 # No Persist Interrupt enable

LUX_DF = 408.0
LUX_COEFB = 1.64  # CH0 coefficient
LUX_COEFC = 0.59  # CH1 coefficient A
LUX_COEFD = 0.86  # CH2 coefficient B

REGISTER_ENABLE = 0x00
REGISTER_CONTROL = 0x01 # also called CONFIG

# interrupt thresholds 16 bit values
REGISTER_AILTL = 0x04 # ALS Interrupt Low Threshhold Low byte with persistence filter
REGISTER_AILTH = 0x05 # ALS Interrupt Low Threshhold high byte with persistence filter
REGISTER_AIHTL = 0x06 # ALS Interrupt High Threshhold with persistence filter low byte
REGISTER_AIHTH = 0x07 # ALS Interrupt High Threshhold with persistence filter high byte
REGISTER_NPAILTL = 0x08 # No Persistence ALS Interrupt Low Threshold low byte
REGISTER_NPAILTH = 0x09 # No Persistence ALS Interrupt Low Threshold high byte
REGISTER_NPAIHTL = 0x0A # No Persistence ALS Interrupt High Threshold low byte
REGISTER_NPAIHTH = 0x0B # No Persistence ALS Interrupt High Threshold high byte
# # of succesive reading to trigger interrupt
REGISTER_PERSIST_FILTER = 0x0C # 8 bit register

REGISTER_PID = 0x11
REGISTER_ID = 0x12
REGISTER_STATUS = 0x13 # interrupt and integration status

REGISTER_CHAN0_LOW = 0x14
REGISTER_CHAN0_HIGH = 0x15
REGISTER_CHAN1_LOW = 0x16
REGISTER_CHAN1_HIGH = 0x17

# CONTROL flags
DEVICE_SRESET = 0x80 # system reset

INTEGRATIONTIME_100MS = 0x00
INTEGRATIONTIME_200MS = 0x01
INTEGRATIONTIME_300MS = 0x02
INTEGRATIONTIME_400MS = 0x03
INTEGRATIONTIME_500MS = 0x04
INTEGRATIONTIME_600MS = 0x05

GAIN_LOW = 0x00  # low gain (1x)
GAIN_MED = 0x10  # medium gain (25x)
GAIN_HIGH = 0x20  # medium gain (428x)
GAIN_MAX = 0x30  # max gain (9876x)


class Tsl2591(object):
    def __init__(
                 self,
                 i2c_bus=1,
                 sensor_address=ADDR,
                 integration=INTEGRATIONTIME_100MS,
                 gain=GAIN_LOW
                 ):
        self.bus = smbus.SMBus(i2c_bus)
        self.sensor_address = sensor_address
        self.integration_time = integration
        self.gain = gain
        self.set_timing(self.integration_time)
        self.set_gain(self.gain)
        self.disable()  # to be sure

    def write_special(self, val): # writes special functions to command register
        self.bus.write_byte(self.sensor_address, SPECIAL_FUNCTION | val)

    def device_resest(self): # reset chip
        self.write_byte_register(REGISTER_CONTROL, DEVICE_SRESET)

    def read_byte_register(self,register):
        return self.bus.read_byte_data(
            self.sensor_address, COMMAND_BIT | register)

    def write_byte_register(self,register,data):
        self.bus.write_byte_data(self.sensor_address, COMMAND_BIT | register, data)

    def read_word_register(self,register):
        return self.bus.read_word_data(
            self.sensor_address, COMMAND_BIT | register)

    def write_word_register(self, register,data):
        self.bus.write_word_data(self.sensor_address, COMMAND_BIT | register, data)

    def dump_regs(self): # dump short list of registers
        print("Enable = %i   CONTROL(CONFIG) = %i" % (self.read_byte_register(
            REGISTER_ENABLE), self.read_byte_register(REGISTER_CONTROL)))
        print("Status register = %i" % self.read_byte_register(REGISTER_STATUS))
        print("Chan 0 = % i   Chan 1 = %i \n" % (self.read_word_register(REGISTER_CHAN0_LOW),
                                              self.read_word_register(REGISTER_CHAN1_LOW)))
        
    def full_dump_regs(self):  # dump registers
        print("Enable = %i   CONTROL(CONFIG) = %i" % (self.read_byte_register(
            REGISTER_ENABLE), self.read_byte_register(REGISTER_CONTROL)))
        print("Threshold Registers:")
        print("AILT = %i   AIHT = %i" % (self.read_word_register(REGISTER_AILTL),
                                         self.read_word_register(REGISTER_AIHTL)))
        print("NAILT = %i NAIHT = %i" % (self.read_word_register(REGISTER_NPAILTL),
                                         self.read_word_register(REGISTER_NPAIHTL)))
        print("Persistance Filter = %i" % self.read_byte_register(REGISTER_PERSIST_FILTER))
        print("Package ID = %i Device ID = %i" % (self.read_byte_register(REGISTER_PID),
                                    self.read_byte_register(REGISTER_ID)))
        print("Status register = %i" % self.read_byte_register(REGISTER_STATUS))
        print("Chan 0 = % i   Chan 1 = %i\n" % (self.read_word_register(REGISTER_CHAN0_LOW),
                                              self.read_word_register(REGISTER_CHAN1_LOW)))


    def set_timing(self, integration):
##        self.enable()
        self.integration_time = integration
        self.bus.write_byte_data(
                    self.sensor_address,
                    COMMAND_BIT | REGISTER_CONTROL,
                    self.integration_time | self.gain
                    )
##        self.disable()

    def get_timing(self):
        return self.integration_time

    def read_timing(self):
        return self.read_byte_register(REGISTER_CONTROL) & 0b111

    def set_gain(self, gain):
##        self.enable()
        self.gain = gain
        self.bus.write_byte_data(
                    self.sensor_address,
                    COMMAND_BIT | REGISTER_CONTROL,
                    self.integration_time | self.gain
                    )
##        self.disable()

    def get_gain(self):
        return self.gain

    def calculate_lux(self, full, ir):
        # Check for overflow conditions first
        ovrflow_test = 0xFFFF if (self.integration_time != INTEGRATIONTIME_100MS) \
        else 0x9400 # if integration time is 100 ms it overflows at 37888 per data sheet
        if (full == ovrflow_test) | (ir == ovrflow_test):
            return 0
            
        case_integ = {
            INTEGRATIONTIME_100MS: 100.,
            INTEGRATIONTIME_200MS: 200.,
            INTEGRATIONTIME_300MS: 300.,
            INTEGRATIONTIME_400MS: 400.,
            INTEGRATIONTIME_500MS: 500.,
            INTEGRATIONTIME_600MS: 600.,
            }
        if self.integration_time in case_integ.keys():
            atime = case_integ[self.integration_time]
        else:
            atime = 100.

        case_gain = {
            GAIN_LOW: 1.,
            GAIN_MED: 25.,
            GAIN_HIGH: 428.,
            GAIN_MAX: 9876.,
            }

        if self.gain in case_gain.keys():
            again = case_gain[self.gain]
        else:
            again = 1.

        # cpl = (ATIME * AGAIN) / DF
        cpl = (atime * again) / LUX_DF
        lux1 = (full - (LUX_COEFB * ir)) / cpl

        lux2 = ((LUX_COEFC * full) - (LUX_COEFD * ir)) / cpl

        # The highest value is the approximate lux equivalent
        return max([lux1, lux2])

    def enable(self):
        self.bus.write_byte_data(
                    self.sensor_address,
                    COMMAND_BIT | REGISTER_ENABLE,
                    ENABLE_POWERON | ENABLE_AEN )  # Enable

    def disable(self):
##        print("disable routine")
##        self.dump_regs()
        self.bus.write_byte_data(
                    self.sensor_address,
                    COMMAND_BIT | REGISTER_ENABLE,
                    ENABLE_POWEROFF
                    )

    def get_full_luminosity(self):
        self.enable()
        time.sleep(0.120*self.integration_time+.2)  # need to delay. This is empirical
        if not (self.read_byte_register(REGISTER_STATUS) & 1): # did not complete integration
            print("Integration not complete")
            self.dump_regs()
            return

        full = self.bus.read_word_data(
                    self.sensor_address, COMMAND_BIT | REGISTER_CHAN0_LOW
                    )
        ir = self.bus.read_word_data(
                    self.sensor_address, COMMAND_BIT | REGISTER_CHAN1_LOW
                    )

        self.disable()
        return full, ir

    def get_luminosity(self, channel):
        full, ir = self.get_full_luminosity()
        if channel == FULLSPECTRUM:
            # Reads two byte value from channel 0 (visible + infrared)
            return full
        elif channel == INFRARED:
            # Reads two byte value from channel 1 (infrared)
            return ir
        elif channel == VISIBLE:
            # Reads all and subtracts out ir to give just the visible!
            return full - ir
        else: # unknown channel!
            return 0


if __name__ == '__main__':

    tsl = Tsl2591()  # initialize
    tsl.full_dump_regs()
    full, ir = tsl.get_full_luminosity()  # read raw values (full spectrum and ir spectrum)
    lux = tsl.calculate_lux(full, ir)  # convert raw values to lux
    print (lux, full, ir)
    print ()

    def test(int_time=INTEGRATIONTIME_100MS, gain=GAIN_LOW):
        tsl.set_gain(gain)
        tsl.set_timing(int_time)
        full_test, ir_test = tsl.get_full_luminosity()
        lux_test = tsl.calculate_lux(full_test, ir_test)
        print("*******Test Routine*******")
        tsl.dump_regs()
        print ('Lux = %f  full = %i  ir = %i' % (lux_test, full_test, ir_test))
        print("integration time = %i" % tsl.get_timing())
        print("gain = %i \n" % tsl.get_gain())        

    for i in [INTEGRATIONTIME_100MS,
              INTEGRATIONTIME_200MS,
              INTEGRATIONTIME_300MS,
              INTEGRATIONTIME_400MS,
              INTEGRATIONTIME_500MS,
              INTEGRATIONTIME_600MS]:
        test(i, GAIN_LOW)

    for i in [GAIN_LOW,
              GAIN_MED,
              GAIN_HIGH,
              GAIN_MAX]:
        test(INTEGRATIONTIME_100MS, i)

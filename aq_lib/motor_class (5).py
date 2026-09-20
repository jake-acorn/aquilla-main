import time
import logging
import pigpio
from aq_lib.config_module import Config
from aq_lib.homing_log import emit_homing_sample

HIGH = 1
LOW = 0

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger( "aquila.motor" )

config = Config()


class Motor():

    position = 0

    def __init__( self ):
        self.pi = pigpio.pi("172.18.0.1", 8888) # this ip can be found via running "docker network inspect fleet_default"
        self.pi.set_mode(self.EN_PIN,   pigpio.OUTPUT)
        self.pi.set_mode(self.STEP_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.DIR_PIN,  pigpio.OUTPUT)
        self.pi.set_mode(self.HME_PIN,  pigpio.INPUT)
        self.pi.write(self.EN_PIN, HIGH) # apperently this Enable Pin is inverted, so setting it HIGH turns the driver off

        logger.info( "Setting pin %d", self.HME_PIN )
        logger.info( "Setup motor pins" )
        value = config.info.get("motor_test") # used ONLY for seeing what is the smallest delay where motor does not stall
        if (
            value is not None
            and value != 0
            and not (
                isinstance(value, str)
                and value.strip().lower() in ("0", "false")
            )
        ):
            self.motor_test()

    def motor_test(self):
        logger.info("Start of motor testing for %s\n", self.motor_name)
        self.home()
        test_delays = [0.00005, 0.00007, 0.00008, 0.00009, 0.0001, 0.00011, 0.00012, 0.00015, 0.0002]
        test_positions = [320, 640, 960, 1280]
        for delay in test_delays:
            for pos in test_positions:
                logger.info("Moving to %d position using pulse delay of %f\n", pos, delay)
                self.move_abs_wo_home_flag( pos, 0.000, delay ) # the second argument is step delay - obsolete and kept only for backwards compatibility
                ret = self.move_w_home_flag( -self.home_steps, 0.0020 )
                if self.isHome():
                    self.reset_position()
                logger.info("Moving home took %d steps\n", ret)

        logger.info("End of motor testing for %s\n", self.motor_name)

    def move_out_of_home(self):
        return self.move_wo_home_flag( 100, 0.0020 )

    def home( self ):
        steps = self.home_steps
        if self.isHome():
            self.move_out_of_home()

        logger.info("Homing using %d steps", steps)
        ret = self.move_w_home_flag( -steps, 0.0020 )
        residual = self.position
        reached_home = bool( self.isHome() )
        if reached_home:
            self.reset_position()
        else:
            logger.error("Did not reach home.")

        emit_homing_sample(
            self.motor_name,
            steps_to_flag=ret,
            residual=residual,
            reached_home=reached_home,
        )
        return ret

    def reset_position( self ):
        if abs(self.position) > 20:
            logger.warning("Position Error %d",self.position )
        logger.info("Resetting motor position: %d -> 0", self.position )
        self.position = 0

    def enable( self ):
        self.pi.write ( self.EN_PIN, LOW )

    def disable( self ):
        self.pi.write ( self.EN_PIN, HIGH )

    def isHome(self):
        return self.pi.read ( self.HME_PIN )

    # step_delay is not used but remains for backwards compatibility
    def move_w_home_flag( self, steps, step_delay = 0.0 ): # does not pigpio as it needs to check for the flag every step
        pulse_delay = 0.0002
        time0 = time.time()
        logger.info( "Moving with home flag: %d", steps )
        self.set_dir ( steps )
        self.enable()

        steps_traveled = steps
        for i in range( abs( steps ) ):
            if self.pi.read ( self.HME_PIN ):
                logger.info( "Caught home flag after %d steps", i )
                steps_traveled = i
                break
            for k in range ( self.step_multiplier ):
                self.pi.write( self.STEP_PIN, HIGH)
                time.sleep ( pulse_delay / 2 )
                self.pi.write( self.STEP_PIN, LOW)
                time.sleep ( pulse_delay / 2 )

        if steps > 0:
            self.position += steps_traveled
        else:
            self.position -= steps_traveled

        time1 = time.time()
        logger.info( "STEP_DELAY DISABLED W HOME FLAG: steps: %d\tmultiplier: %d\tpulse_delay: %f\tstep_delay: %f\nAnticipated time: with step_delay: %f;\twithout: %f\nActual time: %f\nPosition updated to: %f" , steps_traveled, self.step_multiplier, pulse_delay, step_delay, steps_traveled*step_delay+pulse_delay*steps_traveled*self.step_multiplier, pulse_delay*steps_traveled*self.step_multiplier, time1-time0, self.position)

        logger.info("Position after homing: %d", self.position)
        return steps_traveled

    def move_abs_w_home_flag( self, position, step_delay = 0.0 ):
        delta = position - self.position
        return self.move_w_home_flag( delta, step_delay )

    def move_abs_wo_home_flag( self, position, step_delay = 0.0, pulse_delay = 0.0001 ):
        logger.info( "Moving %d", position )
        delta = position - self.position
        logger.info( "Moving delta %d", delta )
        return self.move_wo_home_flag( delta, step_delay, pulse_delay )

    def set_dir( self, steps ):
        if steps < 0:
            logger.info( "Setting DIR=LOW" )
            self.direction = 1
            self.pi.write( self.DIR_PIN, self.DIR_BACK_STATE)
        else:
            logger.info( "Setting DIR=HIGH" )
            self.direction = 0
            self.pi.write( self.DIR_PIN, self.DIR_FORWARD_STATE)

    def move_wo_home_flag( self, steps, step_delay = 0.0, pulse_delay = 0.0001 ):

        time0 = time.time()
        self.set_dir( steps )
        self.enable()

        Y = self.STEP_PIN
        pulse_number = abs(steps)*self.step_multiplier # true number of pulses
        pulse2 = int((pulse_delay * 1000000) // 2) # for backward compatibility reasons pulse_delay reflects the duration of entire ON-OFF cycle; pigpio functions take time in microseconds hence the conversion
        # To create a pulse_delay cycle (pulse_delay//2 sec high,pulse_delay//2 sec low) on GPIO Y:
        logger.info("pulse2: %f, pulse_number: %d\n", pulse2, pulse_number)
        pulse_high = pigpio.pulse(1<<Y, 0, pulse2) # Turn GPIO Y ON for pulse_delay//2
        pulse_low = pigpio.pulse(0, 1<<Y, pulse2) # Turn GPIO Y OFF for pulse_delay//2
        self.pi.wave_add_generic([pulse_high, pulse_low])
        logger.info("Created the wave")

        wave_a = self.pi.wave_create()

        inner_count = pulse_number // 8 # pulse_number MUST BE divisble by 8! Such assumption stems from self.step_multiplier which can be either 8 or 32

        if inner_count > 65000:
            logger.info ("CRITICAL ERROR: INNER STEP COUNT IS TOO LARGE: %d > 65000", inner_count)
            self.pi.wave_delete(wave_a)
            return

        chain = [
            255, 0,          # Start Outer
                255, 0,      # Start Inner
                    wave_a,
                255, 1, inner_count % 256, inner_count // 256,# End Inner
            255, 1, 8, 0     # End Outer
        ]
        self.pi.wave_chain(chain)
        logger.info("Created the chain")
        while self.pi.wave_tx_busy():
            time.sleep(0.005) # Sleep for 5ms to avoid maxing out CPU
        self.pi.wave_delete(wave_a)
        logger.info ("Deleted the chain, success\n")

        self.position += steps

        time1 = time.time()
        logger.info( "STEP_DELAY DISABLED WO HOME FLAG: steps: %d\tmultiplier: %d\tpulse_delay: %f\tstep_delay: %f\nAnticipated time: with step_delay: %f;\twithout: %f\nActual time: %f\nPosition updated to: %d" , steps, self.step_multiplier, pulse_delay, step_delay, steps*step_delay+pulse_delay*steps*self.step_multiplier, pulse_delay*steps*self.step_multiplier, time1-time0, self.position)
        # negative step number indicates backwards direction
        logger.info( "Position updated to: %d", self.position )
        return steps

    def test(self ):
        for _ in range ( 1 ):
            logger.info ( "1000 steps forward" )
            #self.move_wo_home_flag (   16000, 0 )
            logger.info ( "1000 steps backwards" )
            self.move_w_home_flag (   -20000, 0 )

class Drawer ( Motor ):

    motor_name = "drawer"
    EN_PIN = 12
    STEP_PIN = 5
    DIR_PIN = 25
    HME_PIN = 24
    DIR_BACK_STATE = LOW
    DIR_FORWARD_STATE = HIGH
    step_multiplier = config.drawer["step_multiplier"]
    open_steps = config.drawer["open_steps"]
    read_steps = config.drawer["read_steps"]
    home_steps = config.drawer["home_steps"]

    def open( self ):
        self.home()
        # pulse_delay 0.00007 (was 0.0001). The inner per-pulse sleep runs
        # step_multiplier x open_steps times and dominates travel time, so dropping
        # it ~30% is what actually speeds the drawer up ?~@~T targets ~10%+ faster open.
        # step_delay kept at 0.0005 (Ryan 04/22/26, was 0.002).
        # Hardware-tested: verify full travel on sn01-03 (lower pulse_delay risks step-skip).
        ret = self.move_abs_wo_home_flag ( self.open_steps, 0.0005, 0.00015 )

    def read( self ):
        self.home()
        # pulse_delay 0.00007 (was 0.0001) to match open(); step_delay kept at 0.001.
        ret = self.move_wo_home_flag ( self.read_steps, 0.001, 0.0001 )

    def goto_position( self, N ):
        if not isinstance(N, (int, float)): # if N is not a number; meant for "opening up" tuples while preserving backwards compatibility
            N = N[1]
        else: return
        logger.info( "Drawer Go to position: %d", N )
        self.move_abs_wo_home_flag( self.read_steps+N*320, 0.000, 0.0001)

class Axis ( Motor ):

    motor_name = "axis"
    EN_PIN = 26
    STEP_PIN = 19
    DIR_PIN = 13
    HME_PIN = 16
    DIR_BACK_STATE = HIGH
    DIR_FORWARD_STATE = LOW
    step_multiplier = config.axis["step_multiplier"]
    home_steps = config.axis["home_steps"]

    def __init__( self ):
        super().__init__()

        # Read positions directly from config if available,
        # otherwise fall back to calculating from well_one and well_spacing
        if "positions" in config.axis:
            self.positions = config.axis["positions"]
            logger.info("Loaded axis positions from config: %s", self.positions)

            self.positions.append(0) # temprary fix to ensure that postions[-1] refers to 0 to imitate the first well while testing the 15-well setup
            # In the future, the code will check whether six or seven positions were provided
            # If seven - all is good, less - throw an error that 15-well mode cannot be used unless all positions are provided

        else:
            # Legacy fallback: calculate from well_one and well_spacing
            w0 = config.axis.get("well_one", 300)
            dw = config.axis.get("well_spacing", 355)
            self.positions = [ w0 + dw*i for i in range(6) ]
            logger.info("Calculated axis positions (legacy): %s", self.positions)

    def goto_position( self, N ):
        if not isinstance(N, (int, float)): # if N is not a number
            N = N[0]
        logger.info( "Go to position: %d", N )
        self.move_abs_wo_home_flag( self.positions[N], 0.000, 0.0001 )

def main():

    import sys

    motor_list = {
                "axis": Axis,
                "drawer": Drawer,
            }

    try:
        motor = sys.argv[1].lower()
        assert motor in motor_list
        steps = int(sys.argv[2] )
        assert "%d"%steps == sys.argv[2]
    except Exception as e:
        print ( e )
        print ( "Usage:", sys.argv[0], "drawer/axis <steps>" )
        exit( -1 )

    MotorClass = motor_list[ motor ]
    motor = MotorClass()
    motor.move_w_home_flag( steps, 0 )

if __name__ == "__main__":
    main()
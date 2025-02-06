from biorobotics import PWM, Encoder
from pyb import Pin
from pyb import Switch


class RobotComponents(object):
    """Class in which we define constants, motors, and the blueswitch. """

    class States(object):
        """
        Define all possible states in which the robot is.
        """
        WAITING = "waiting" # this state is never entered in our code and can therefore be neglected 
        MOVING = "moving"
        HITTING = "hitting"

    class Motor(object):
        def __init__(self, pin_direction: str, pin_pwm: str, pwm_frequency: int):
            self.pwm = PWM(pin_pwm, pwm_frequency)
            self.direction_pin = Pin(pin_direction, Pin.OUT)

        def step(self, direction: bool, duty_cycle: float):
            self.direction_pin.value(direction)  # set the direction pin
            self.pwm.write(abs(duty_cycle) / 0.8 + 0.2)  # set the PWM duty cycle with a pre-load value of 0.2
            return

    class BlueSwitch(object):

        def __init__(self):
            self.switch = Switch()
            self.switch.callback(self.callback)
            self.button_state = False
            return

        def callback(self):
            self.button_state = self.switch.value()
            return

        def offload_value(self):
            offload_value = self.button_state
            self.button_state = False
            return offload_value

    class RobotSettings(object):
        class Pins(object):
            MOTOR_DIRECTION_PIN_1 = "D4"
            MOTOR_PWM_PIN_1 = "D5"
            MOTOR_PWM_PIN_2 = "D6"
            MOTOR_DIRECTION_PIN_2 = "D7"
            DEMO_BUTTON_1 = "D8"  # Corresponds to button 1 of the Biorobotics shield
            DEMO_BUTTON_2 = "D9"  # Corresponds to button 2 of the Biorobotics shield
            POTMETER_1 = "A0"
            POTMETER_2 = "A1"
            RIGHT_BICEP = "A2"
            LEFT_BICEP = "A3"
            CALF = "A4"

            ENCODER1_A = "D12"
            ENCODER1_B = "D11"
            ENCODER2_A = "D0"
            ENCODER2_B = "D1"

            SOLENOID = "D2"

        class Constants(object):
            pwm = 1000
            EMG_GAIN_1 = 0.638945525159022476024972547747893258929
            EMG_GAIN_2 = 0.959773568953520062052575667621567845345

            #high pass second order Butterworth filter
            HP_A1 = -1.142980502539901133118860343529377132654
            HP_A2 = 0.412801598096188770981029847462195903063
            HP_B0 = 1.0
            HP_B1 = -2.0
            HP_B2 = 1.0

            #bandstop second order Butterworth filter
            BS_A1 = -0.960616192564186621716260106040863320231
            BS_A2 = 0.919547137907040124105151335243135690689
            BS_B0 = 1.0
            BS_B1 = -1.000877940003687793790732030174694955349
            BS_B2 = 1.0

            #threshold values for light squeeze
            LB_LIGHT_SQUEEZE = 0.2
            HB_LIGHT_SQUEEZE = 0.35

            #threshold values for hard squeeze
            LB_HARD_SQUEEZE = 0.45
            HB_HARD_SQUEEZE = 0.9

            #threshold values for calf squeeze
            LB_CALF_SQUEEZE = 0.4
            HB_CALF_SQUEEZE = 1

            L2 = 0.159  # length of arm L2
            L5 = 0.255  # length of arm L5

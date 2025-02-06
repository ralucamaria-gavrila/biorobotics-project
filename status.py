from pyb import LED, Pin
from biorobotics import AnalogIn, SerialPC, Encoder
from robot_components import RobotComponents as rc
import EMGproces
from controller import PIDcontroller
from robotkin import RKIModel


class Status(object):
    def __init__(self, frequency):
        """
        The robot is initialized in the Moving state because this state does not involve any hitting actions.
        Here we declare all the components needed for the functionality of the moving robot.
        """
        self.current_state = rc.States.MOVING
        self.move = False
        self.hit = False
        self.leds = [LED(1), LED(2), LED(3)]  # led1 = green, led2 = yellow, led3=red

        self.button_forward_backward = Pin(rc.RobotSettings.Pins.DEMO_BUTTON_1, Pin.IN, Pin.PULL_UP)
        self.button_forward_backward_status = 0
        self.button_left_right = Pin(rc.RobotSettings.Pins.DEMO_BUTTON_2, Pin.IN, Pin.PULL_UP)
        self.button_left_right_status = 0

        self.potmeter1 = AnalogIn(rc.RobotSettings.Pins.POTMETER_1)
        self.potmeter_state_1 = 0

        self.potmeter2 = AnalogIn(rc.RobotSettings.Pins.POTMETER_2)
        self.potmeter_state_2 = 0
        self.blue_switch = rc.BlueSwitch()

        self.pidcontroller = PIDcontroller(1 / 300, 0.45, 0, 0.001)  # directing the PID
        self.pidcontroller_2 = PIDcontroller(1 / 300, 0.61, 0, 0.001)
        self.motor = rc.Motor(rc.RobotSettings.Pins.MOTOR_DIRECTION_PIN_1, rc.RobotSettings.Pins.MOTOR_PWM_PIN_1, 1000)  # moving the motors
        self.motor2 = rc.Motor(rc.RobotSettings.Pins.MOTOR_DIRECTION_PIN_2, rc.RobotSettings.Pins.MOTOR_PWM_PIN_2, 1000)
        self.encoder1_AB = Encoder(rc.RobotSettings.Pins.ENCODER1_A, rc.RobotSettings.Pins.ENCODER1_B) # getting encoder count from encoder
        self.encoder2_AB = Encoder(rc.RobotSettings.Pins.ENCODER2_A, rc.RobotSettings.Pins.ENCODER2_B)
        self.pc = SerialPC(6)

        # These are the "modes" in which we control the robot. If the robot is in emg or buttons mode,
        # the arm will actually move. In calibration mode, the robot just records the maximum value of
        # a hard squeeze, in order to normalize the EMG signal. The robot will not move in calibration mode.
        self.in_emg_mode = False
        self.in_buttons_mode = True
        self.in_calibration_mode = False

        self.Right_biceps = EMGproces.FilterEMG(rc.RobotSettings.Pins.RIGHT_BICEP, 30, "RIGHT_BICEP")
        self.Left_biceps = EMGproces.FilterEMG(rc.RobotSettings.Pins.LEFT_BICEP, 30, "LEFT_BICEP")
        self.Right_calf = EMGproces.FilterEMG(rc.RobotSettings.Pins.CALF, 30, "CALF")

        self.solenoid = Pin(rc.RobotSettings.Pins.SOLENOID, Pin.OUT)

        self.rki = RKIModel(frequency)

    def State(State):
        return

    def moving(self):
        self.move = True
        self.hit = False
        return

    def Hit(self):
        self.hit = True
        self.move = False
        return

    def stop(self):
        self.move = self.hit = False
        self.current_state = rc.States.WAITING
        return

    def left_bicep_calibrate(self):
        self.Left_biceps.read_EMG()

    def right_bicep_calibrate(self):
        self.Right_biceps.read_EMG()

    def calf_calibrate(self):
        self.Right_calf.read_EMG()

    def calibrate(self):
        self.left_bicep_calibrate()
        self.right_bicep_calibrate()
        self.calf_calibrate()

    def update_sensors(self):
        # State machine to switch between buttons mode, calibration mode and EMG mode
        if self.blue_switch.offload_value():
            if self.in_emg_mode:
                self.in_emg_mode = False
                self.in_buttons_mode = True
                self.in_calibration_mode = False
            elif self.in_buttons_mode:
                self.in_buttons_mode = False
                self.in_calibration_mode = True
                self.in_emg_mode = False
            elif self.in_calibration_mode:
                self.in_calibration_mode = False
                self.in_emg_mode = True
                self.in_buttons_mode = False
        if self.in_calibration_mode:
            self.calibrate()
        elif self.in_buttons_mode:
            self.potmeter_state_1 = self.potmeter1.read()
            self.potmeter_state_2 = self.potmeter2.read()
            self.button_forward_backward_status = self.button_forward_backward.value()
            self.button_left_right_status = self.button_left_right.value()

            self.hit = self.button_forward_backward_status == 0 and self.button_left_right_status == 0
            if self.hit:
                pass
            elif self.button_forward_backward_status == 0:
                if self.potmeter_state_1 > 0.5:
                    self.rki_directions = [1, 0]  # right (when standing behind robot)
                else:
                    self.rki_directions = [0, 1]  # forward (when standing behind robot)
            elif self.button_left_right_status == 0:
                if self.potmeter_state_2 > 0.5:
                    self.rki_directions = [0, -1]  # backwards (when standing behind robot)
                else:
                    self.rki_directions = [-1, 0]  # left (when standing behind robot)
            else:
                self.rki_directions = [0, 0]  # no movement
        elif self.in_emg_mode: 
            left_biceps = self.Left_biceps.read_EMG()   #checks of threshold in left bicep is reached
            right_bicep = self.Right_biceps.read_EMG()  #checks of threshold in right bicep is reached
            right_calf = self.Right_calf.read_EMG()     #checks of threshold in right calf is reached

            if left_biceps == "BACKWARD":
                self.hit = False
                self.rki_directions = [0, 1]  # backwards for DMD patient (sitting in front of robot)
                self.pc.set(1, self.Left_biceps.normalize)
                self.pc.send()
            elif left_biceps == "LEFT":
                self.hit = False
                self.rki_directions = [1, 0]  # left for DMD patient (sitting in front of robot)
                self.pc.set(1, self.Left_biceps.normalize)
                self.pc.send()
            elif right_bicep == "FORWARD":
                self.hit = False
                self.rki_directions = [0, -1]  # forward for DMD patient (sitting in front of robot)
                self.pc.set(0, self.Right_biceps.normalize)
            elif right_bicep == "RIGHT":
                self.hit = False
                self.rki_directions = [-1, 0]  # right for DMD patient (sitting in front of robot)
                self.pc.set(0, self.Right_biceps.normalize)
            elif right_calf == "HITTING":
                self.hit = True
                self.rki_directions = [0, 0]
                self.pc.set(2, self.Right_calf.normalize)
            else:
                self.hit = False
                self.rki_directions = [0, 0]
            self.pc.send()

        return self.rki_directions

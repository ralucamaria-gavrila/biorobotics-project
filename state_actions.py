from robot_components import RobotComponents as rc
from controller import counter_to_angle_Short, counter_to_angle_Long

#The state actions are based on the general state machine explained by dr.ir. M. Vlutters in session 4

class BaseAction(object):
    def __init__(self, status):
        self.status = status
        return

    def guard(self):
        return False

    def entry(self):
        return

    def run(self):
        return

    def exit(self):
        return


class Waiting(BaseAction):
    # This state is never entered in the code since we did not define boundaries and could therefore be neglected.
    # However, this state is a good start for future improvement of the code when boundaries of working spaces are defined.
    #Below a few comments are writen as what could be improved in the future

    def __init__(self, status):
        super().__init__(status)
        return

    def guard(self):
        # If position of end-effector is out of working space, and not already in waiting state, go to waiting state
        return False

    def entry(self):
        self.status.leds[0].on()
        self.status.leds[1].on()
        self.status.leds[2].on()
        return

    def run(self):
        self.status.motor.step(0, 0)
        self.status.motor2.step(0, 0)
        return

    def exit(self):
        self.status.leds[0].off()
        self.status.leds[1].off()
        self.status.leds[2].off()
        return


class Moving(BaseAction):
    def __init__(self, status):
        super().__init__(status)
        return

    def guard(self):
        return (self.status.current_state == rc.States.HITTING and self.status.hit == False) or (
                self.status.current_state == rc.States.WAITING and False)

    def entry(self):
        return

    def run(self): 

        m_a1 = counter_to_angle_Long(self.status.encoder1_AB.counter()) # current motor angle m1 in degrees
        m_a2 = counter_to_angle_Short(self.status.encoder2_AB.counter()) # current motor angle m2 in degrees
        a_a1 = 180 - m_a1 # mirror the current m1 angle
        a_a2 = 360 - m_a2 # mirror the current m2 angle
     
        # calculate the desired motor angles m1 and m2 in degrees
        m1_set, m2_set = self.status.rki.compute_velocity_for_motors(self.status.rki_directions, a_a1, a_a2, 0.5)

        ref1 = 180 - m1_set
        ref2 = 360 - m2_set

        # directing the pwm_1&pwm_2 and directions for both motors 
        pwm_1 = self.status.pidcontroller.step(ref1, self.status.encoder1_AB.counter(), True)
        pwm_2 = self.status.pidcontroller_2.step(ref2, self.status.encoder2_AB.counter(), False)
        dir1 = 0
        dir2 = 0
        #determening the direction based of the pwm
        if pwm_1 > 0:
            dir1 = 0
        if pwm_1 < 0:
            dir1 = 1

        if pwm_2 > 0:
            dir2 = 0
        if pwm_2 < 0:
            dir2 = 1

        self.status.motor.step(dir1, pwm_1)
        self.status.motor2.step(dir2, pwm_2)
        return


class Hitting(BaseAction):

    def __init__(self, status):
        super().__init__(status)
        return

    def guard(self):
        return (self.status.current_state == rc.States.MOVING and self.status.hit == True)

    def entry(self):
        self.status.solenoid.off() 
        self.status.leds[2].on()
        self.robot_status.motor.step(0, 0)
        self.robot_status.motor2.step(0, 0)

    def run(self):
        pass

    def exit(self):
        self.status.solenoid.on()
        self.status.leds[2].off()
        return

from status import Status
import state_actions as actions
from biorobotics import Ticker
from robotkin import RKIModel
from robot_components import RobotComponents as rc
from state_actions import BaseAction

#The basis of the statemachine is based on the state machine explained by dr.ir. M. Vlutters in session 4

class StateMachine(object):
    def __init__(self, frequency):
        self.robot_status = Status(frequency)

        self.possible_actions = {
            rc.States.MOVING: actions.Moving(self.robot_status),
            rc.States.HITTING: actions.Hitting(self.robot_status),
            rc.States.WAITING: actions.Waiting(self.robot_status) # This state is never entered, so can be neglected 
        }                                                         # but it could be used in future designs

        self.ticker = Ticker(0, frequency, self.step)
        self.RKI_model = RKIModel()
        self.action = BaseAction(self.robot_status)
        return

    def step(self):
        """
        To be executed every time step
        """

        self.robot_status.update_sensors()

        for state, action in self.possible_actions.items():
            if action.guard():
                
                self.possible_actions[self.robot_status.current_state].exit()

                self.robot_status.current_state = state

                action.entry()

                break

        self.possible_actions[self.robot_status.current_state].run()
        print(actions.my_encoder_2,self.ticker)

        return

    def start(self):
        self.ticker.start()
        return

    def stop(self):
        self.ticker.stop()
       
        return
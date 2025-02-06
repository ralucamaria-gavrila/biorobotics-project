from biorobotics import AnalogIn
import math
from robot_components import RobotComponents as rc


class Butterworth:
    """
    General class for a Butterworth filter. Coefficients need to be given as an input.
    """
    def __init__(self, a1, a2, b0, b1, b2):
        self.a1 = a1
        self.a2 = a2
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.w1 = 0.0
        self.w2 = 0.0

    def filter(self, x):
        y = float(self.b0 * x + self.w1)
        self.w1 = float(self.b1 * x - self.a1 * y + self.w2)
        self.w2 = float(self.b2 * x - self.a2 * y)
        return y


class FilterEMG(object):
    def __init__(self, Pin, window_size, muscle):

        self.muscle_movement_mapping = {
            "RIGHT_BICEP": {
                "light_threshold": "RIGHT",
                "hard_threshold": "FORWARD"
            },
            "LEFT_BICEP": {
                "light_threshold": "LEFT",
                "hard_threshold": "BACKWARD"
            },
            "CALF": {
                "light_threshold": "HITTING",
                "hard_threshold": "HITTING"
            }
        }
        # 300Hz fs; 30Hz cut off Butterworth high pass
        self.gain_1 = rc.RobotSettings.Constants.EMG_GAIN_1
        # 300Hz fs; 48-52 band stop Butterworth
        self.gain_2 = rc.RobotSettings.Constants.EMG_GAIN_2

        self.pin = Pin

        # 300Hz fs; 30Hz cut off Butterworth high pass
        self.HP_300_30_1 = Butterworth(rc.RobotSettings.Constants.HP_A1,
                                       rc.RobotSettings.Constants.HP_A2,
                                       rc.RobotSettings.Constants.HP_B0,
                                       rc.RobotSettings.Constants.HP_B1,
                                       rc.RobotSettings.Constants.HP_B2)

        # 300Hz fs; 48-52 band stop Butterworth
        self.BS_300_50_2 = Butterworth(rc.RobotSettings.Constants.BS_A1,
                                       rc.RobotSettings.Constants.BS_A2, rc.RobotSettings.Constants.BS_B0,
                                       rc.RobotSettings.Constants.BS_B1, rc.RobotSettings.Constants.BS_B2)
        
        # set thresholds for light squeeze
        self.lower_bound_light_squeeze = rc.RobotSettings.Constants.LB_LIGHT_SQUEEZE
        self.high_bound_light_squeeze = rc.RobotSettings.Constants.HB_LIGHT_SQUEEZE

        # set thresholds for hard sqeeuze
        self.lower_bound_hard_squeeze = rc.RobotSettings.Constants.LB_HARD_SQUEEZE
        self.high_bound_hard_squeeze = rc.RobotSettings.Constants.HB_HARD_SQUEEZE

        # set thresholds for squeezing the calf 
        self.lower_bound_calf_squeeze = rc.RobotSettings.Constants.LB_CALF_SQUEEZE
        self.high_bound_calf_squeeze = rc.RobotSettings.Constants.HB_CALF_SQUEEZE

        self.min_value = 0
        self.max_value = 0
        self.window_size = window_size
        self.buffer = [0] * window_size
        self.normalize = -1
        self.muscle = muscle

    def check_threshold(self, normalized_value):
        if self.muscle == "CALF":
            if self.lower_bound_calf_squeeze < normalized_value < self.high_bound_calf_squeeze:
                # print(f"I am squeezing with {normalized_value} with {self.muscle}")
                return self.muscle_movement_mapping.get(self.muscle).get("hard_threshold")
            else:
                # print("I am in waiting state back from hitting")
                return rc.States.WAITING
        else:
            if normalized_value < self.lower_bound_light_squeeze:
                # print("I am in waiting state")
                return rc.States.WAITING
            if self.lower_bound_hard_squeeze < normalized_value < self.high_bound_hard_squeeze:
                # print(f"I am squeezing hard with {normalized_value} with {self.muscle}")
                return self.muscle_movement_mapping.get(self.muscle).get("hard_threshold")
            elif self.lower_bound_light_squeeze < normalized_value < self.high_bound_light_squeeze:
                # print(f"I am squeezing lightly with {normalized_value} with {self.muscle} ")
                return self.muscle_movement_mapping.get(self.muscle).get("light_threshold")

    def read_EMG(self):
        # EMG processing chain
        emg = AnalogIn(self.pin)

        rawvalue = emg.read()
        filterHP_value = (self.HP_300_30_1.filter(rawvalue)) * self.gain_1          #high-pass filter
        filterBS_value = (self.BS_300_50_2.filter(filterHP_value)) * self.gain_2    #bandstop filter
        absolute_value = abs(filterBS_value)

        # calculating RMS value 
        self.buffer.pop(0)
        self.buffer.append(absolute_value)
        square_sum = sum((sample ** 2 for sample in self.buffer))
        rms_value = math.sqrt(square_sum / len(self.buffer))

        # setting max and min RMS value in order to normalize the filtered EMG data
        self.max_value = max(rms_value, self.max_value)
        self.min_value = min(rms_value, self.min_value)

        # normalization of EMG data
        self.normalize = (rms_value - self.min_value) / (self.max_value - self.min_value)
        if self.normalize >= 1:
            self.normalize = 1
        if self.normalize <= 0:
            self.normalize = 0
        return self.check_threshold(self.normalize)

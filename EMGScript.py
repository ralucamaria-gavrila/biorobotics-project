from biorobotics import Ticker, SerialPC
from EMGproces import FilterEMG
from robot_components import RobotComponents as rc

# This is a test script of the EMG processing chain which was used to visualize the filtered data using Uscope. 
# This script is not being called upon anymore but implemented in the other scipts  

pc = SerialPC(3)

# filter the incoming EMG signals of the right bicep, left bicep and right calf
Right_biceps = FilterEMG(rc.RobotSettings.Pins.RIGHT_BICEP, 30, "RIGHT_BICEP")
Left_biceps = FilterEMG(rc.RobotSettings.Pins.LEFT_BICEP, 30, "LEFT_BICEP")
Right_calf = FilterEMG(rc.RobotSettings.Pins.CALF, 30, "CALF")


def loop():
    Left_biceps.read_EMG()
    Right_biceps.read_EMG()
    Right_calf.read_EMG()

    pc.set(0, Right_biceps.normalize)
    pc.set(1, Left_biceps.normalize)
    pc.set(2, Right_calf.normalize)
    pc.send()


ticker = Ticker(0, 300, loop, enable_gc=True)
ticker.start()

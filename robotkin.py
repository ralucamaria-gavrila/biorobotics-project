from ulab import numpy as np
import RKI_function as RKI
from robot_components import RobotComponents as rc


def velocity_multiplier(v_x, v_y, multiplier):
    """
    This function multiplies the direction with the input mulitplier. It enables to change the velocity of the end-effector. 
    """
    return v_x * multiplier, v_y * multiplier


def parameters(angle_from_motor1, angle_from_motor2):
    """
    Transforms the measured current motor angles from degrees to radians and calculates the corresponding virtual joint angles from motor angels. 
    """
    m1 = deg2rad(angle_from_motor1)  # measured current motor angles changed from degrees to radians
    m2 = deg2rad(angle_from_motor2)  # measured current motor angles changed from degrees to radians
    q1 = m1                         # virtual joint angle q1
    q2 = m2 - m1 - np.pi            # virtual joint angle q2
    return q1, q2, m1, m2


def rad_to_deg(rad):
    return rad * 180 / np.pi


def deg2rad(deg):
    return deg * np.pi / 180


class RKIModel(object):

    def __init__(self, frequency=300):
        # ROBOT PARAMETERS
        self.dt = 1 / frequency
        self.T1 = np.array([1, 0, 0])
        self.T2 = np.array([1, 0, -rc.RobotSettings.Constants.L2])
        self.He00 = np.eye(3)
        self.He00[0, 2] = rc.RobotSettings.Constants.L2 + rc.RobotSettings.Constants.L5     # x of endeffector in reference configuration
        self.He00[1, 2] = 0                                                                 # y of endeffector in reference configuration

    def compute_forward_kinematics(self, q1, q2):
        """
        Calculate the forward kinemetics using the current virtual joint angles q1 and q2
        """
        He0 = RKI.expT(RKI.tilde(self.T1) * q1)
        He0 = RKI.matrix_multiply(He0, RKI.expT(RKI.tilde(self.T2) * q2))
        He0 = RKI.matrix_multiply(He0, self.He00)
        pe0 = He0[:2, 2]
        return pe0

    def compute_inverse_dif_kinematics(self, rki_directions, q1, q2, velocity_mult):
        """
        Calculate the inverse differntial kinemetics using the desired end-effector direction (rki_direction), the current virtual joint angles q1 and q2 and velocity multiplier.
        """
        J = np.zeros((3, 2))
        J[:, 0] = np.array([1, 0, 0])
        J[:, 1] = np.array([1, rc.RobotSettings.Constants.L2 * np.sin(q1), -rc.RobotSettings.Constants.L2 * np.cos(q1)])
        Hf0 = np.eye(3)
        Hf0[:, 2] = np.array(
            [rc.RobotSettings.Constants.L2 * np.cos(q1) + rc.RobotSettings.Constants.L5 * np.cos(q1 + q2),
             rc.RobotSettings.Constants.L2 * np.sin(q1) + rc.RobotSettings.Constants.L5 * np.sin(q1 + q2),
             1])
        H0f = RKI.inverseH(Hf0)
        AdH0f = RKI.Adjoint(H0f)
        Jp = RKI.matrix_multiply(AdH0f, J)
        Jpp = Jp[1:, :] #pseudo-inverse


        v_x, v_y = rki_directions #desired end-effector velocity derived from moving state and decided by triggered threshold
        v_x, v_y = velocity_multiplier(v_x, v_y, velocity_mult)
        v = np.array([[v_x], [v_y]])
        dq_set = RKI.matrix_vector_multiply(np.linalg.inv(Jpp), v) # desired virtual joint velocity derived from desired end-effector velocity
        q1_set = q1 + dq_set[0] * self.dt  
        q2_set = q2 + dq_set[1] * self.dt  
        m1_set = q1_set                     # desired motor (m1) angle position to achieve desired end-effector velocity
        m2_set = q2_set + m1_set + np.pi    # desired motor (m2) angle position to achieve desired end-effector velocity

        return m1_set, m2_set

    def compute_velocity_for_motors(self, rki_directions, angle_from_motor1, angle_from_motor2, velocity_mult=1):
        """
        Compute the desired motor angle position based on the direction (desired end-effector velocity), 
        the current motor angles, and velocity multiplier. It returns the desired motor angle for both motors. 
        This needs to be feed into the controller.
        """
        q1, q2, m1, m2 = parameters(angle_from_motor1, angle_from_motor2)
        pe0 = self.compute_forward_kinematics(q1, q2)
        m1_set, m2_set = self.compute_inverse_dif_kinematics(rki_directions, q1, q2, velocity_mult)

        m1_set = rad_to_deg(m1_set)
        m2_set = rad_to_deg(m2_set)
        m1 = rad_to_deg(m1)
        m2 = rad_to_deg(m2)

        return m1_set, m2_set

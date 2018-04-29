
from math import *
import random
import numpy as np

import nelder_mead

# Plot imports - intentionally commented out to prevent import errors for grading
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import sympy as sym

# Note : The transform R->H is computed using DH tables, which then compute A matrices,
#        then the resultant matrices are multiplied to get the FK result

class Transform:
    def __init__(self, theta, d, a, alpha, q_i=0):
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
        self.q_i = q_i

    def get_A_matrix(self):
        theta = self.theta
        a = self.a
        alpha = self.alpha
        d = self.d
        return [ [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                 [0, sin(alpha), cos(alpha), d],
                 [0, 0, 0, 1] ]

    def __repr__(self):
        theta = self.theta
        a = self.a
        alpha = self.alpha
        d = self.d
        q_i = self.q_i
        return str(str(["c"+str(q_i), "-s"+str(q_i)+"*"+str(cos(alpha)), "s"+str(q_i)+"*"+str(sin(alpha)), str(a)+"*c"+str(q_i)]) + "\n" +
                   str(["s"+str(q_i), "c"+str(q_i)+"*"+str(cos(alpha)), "-c"+str(q_i)+"*"+str(sin(alpha)), str(a)+"*s"+str(q_i)]) + "\n" +
                   str(["0", str(sin(alpha)), str(cos(alpha)), str(d)]) + "\n" +
                   str(["0", "0", "0", "1"]) + "\n" )

class Arm:
    def __init__(self, joints):
        self.n_joints = len(joints)
        self.set_joint_angles(joints)

    def set_joint_angles(self, joints):
        assert len(joints) == self.n_joints
        self.joints = joints
        self._set_dh_transforms()

    def _set_dh_transforms(self):
        raise NotImplementedError, "The set_dh_transforms method must be implemented"
                                   " to match the desired manipulator geometry"

        """
        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

        Within this method, specify the geometry of the desired robot using
        Denavit-Hartenberg parameters as shown below
        Each row of the dh matrix corresponds to: [θ, d, r, α]
            θ : Angle about previous z, from old x to new x
            d : Offset along previous z to the common normal
            r : Length of the common normal (aka a, but if using this notation, do not confuse with α).
                Assuming a revolute joint, this is the radius about previous z.
            α : Angle about common normal, from old z axis to new z axis

        Note that the right hand rule for specifying axis is always used.

        EXAMPLE:

        self.dh = [ [radians(0), 100, 0, radians(-90)],
                    [radians(self.joints[0]), -98, 0, radians(90)],
                    [radians(self.joints[1]), 0, 105, radians(90)],
                    [radians(self.joints[2]), 15, 57.75+55.95, radians(-90)],
                    [0, -12.31, 0, 0] ]
        """

    def get_forward_kinematics(self):
        A = []
        R = None
        for dh in self.dh:
            trans = Transform(dh[0], dh[1], dh[2], dh[3])
            if R == None:
                R = trans.get_A_matrix()
            else:
                R = np.matmul(R,trans.get_A_matrix())
        return (R[0][3],R[1][3],R[2][3])

    def get_inverse_kinematics():
        # TODO : Use Nelder-mead as a generalized IK platform
        #        Bonus: use differential kinematics to guide the search
        pass

    def get_differential_kinematics():
        # TODO : Use sympy to compute derivatives and return jacobian
        pass

    def get_inverse_differential_kinematics():
        # TODO : invert the result of the last method
        pass

    def print_transforms(self):
        q_i = 0
        for dh in self.dh:
            trans = Transform(dh[0], dh[1], dh[2], dh[3], q_i = q_i)
            q_i += 1

    def plot_workspace(self,joint_angles):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for angles in joint_angles:
            self.set_joints(angles)
            p = self.get_forward_kinematics()
            ax.scatter(p[0], p[1], p[2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()


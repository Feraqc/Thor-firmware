#!/usr/bin/env python
"""
@author: Alexs A. - Of this file
@author: Robot by AngelM - Open Source 

"""

# all parameters are in SI units: m, radians, kg, kg.m2, N.m, N.m.s etc.

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from spatialmath import base


class ThorArm(DHRobot):
    """
    Class that models the open source Thor arm manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    `ThorArm()` is an object that models the opensource manipulator, and
    describes its kinematic and dynamic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.ThorArm()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. note::
        - SI units are used.
        - The model includes armature inertia and gear ratios.
        - The value of m1 is given as 0 here.  Armstrong found no value for it
          and it does not appear in the equation for tau1 after the
          substituion is made to inertia about link frame rather than COG
          frame.
        - Gravity load torque is the motor torque necessary to keep the joint
          static, and is thus -ve of the gravity caused torque.


    :references:
        - https://hackaday.io/project/12989-thor/log/43941-forward-kinematics
        

    """  # noqa

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from numpy import pi

            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        #base = 26.45 * inch  # from mounting surface to shoulder axis

        L = [
            RevoluteDH(
                d=0.2,  # link length (Dennavit-Hartenberg notation)
                a=0,  # link offset (Dennavit-Hartenberg notation)
                alpha=pi / 2,  # link twist (Dennavit-Hartenberg notation)
                #I=[0, 0.35, 0, 0, 0, 0],
                # inertia tensor of link with respect to
                # center of mass I = [L_xx, L_yy, L_zz,
                # L_xy, L_yz, L_xz]
                #r=[0, 0, 0],
                # distance of ith origin to center of mass [x,y,z]
                # in link reference frame
                #m=0,  # mass of link
                #Jm=200e-6,  # actuator inertia
                #G=-62.6111,  # gear ratio
                #B=1.48e-3,  # actuator viscous friction coefficient (measured
                # at the motor)
                #Tc=[0.395, -0.435],
                # actuator Coulomb friction coefficient for
                # direction [-,+] (measured at the motor)
                qlim=[-pi, pi],  # minimum and maximum joint angle
            ),
            RevoluteDH(
                offset = pi/2,
                d=0,
                a=0.16,
                alpha=zero,
               # I=[0.13, 0.524, 0.539, 0, 0, 0],
               # r=[-0.3638, 0.006, 0.2275],
               # m=17.4,
               # Jm=200e-6,
               # G=107.815,
               # B=0.817e-3,
               # Tc=[0.126, -0.071],
                qlim=[-np.pi/2, np.pi/2],  # qlim=[-45*deg, 225*deg]
            ),
            #RevoluteDH(
            #    offset=-np.pi/2,
            #    d=zero,
            #    a=zero,
            #    alpha=zero,
               # I=[0.066, 0.086, 0.0125, 0, 0, 0],
               # r=[-0.0203, -0.0141, 0.070],
               # m=4.8,
               # Jm=200e-6,
               # G=-53.7063,
               # B=1.38e-3,
               # Tc=[0.132, -0.105],
            #    qlim=[-pi/2, pi/2]
            #),
            RevoluteDH(
                offset=-np.pi/2,
                d=0,
                a=0,
                alpha=-pi / 2,
               # I=[1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0],
               # r=[0, 0.019, 0],
               # m=0.82,
               # Jm=33e-6,
               # G=76.0364,
               # B=71.2e-6,
               # Tc=[11.2e-3, -16.9e-3],
                qlim=[-pi/2, pi/2],  # qlim=[-110*deg, 170*deg]
            ),
            RevoluteDH(
                d=0.10,
                a=0,
                alpha=pi / 2,
               # I=[0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0],
               # r=[0, 0, 0],
               # m=0.34,
               # Jm=33e-6,
               # G=71.923,
               # B=82.6e-6,
               # Tc=[9.26e-3, -14.5e-3],
                qlim=[-100 * deg, 100 * deg],
            ),
            RevoluteDH(
                d=0,
                a=0,
                alpha=-pi/2,
               # I=[0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0],
               # r=[0, 0, 0.032],
               # m=0.09,
               # Jm=33e-6,
               # G=76.686,
               # B=36.7e-6,
               # Tc=[3.96e-3, -10.5e-3],
                qlim=[-pi, pi],
            ),
            RevoluteDH(),
        ]

        super().__init__(
            L,
            name="Thor",
            manufacturer="Open_Source",
            keywords=("dynamics", "symbolic", "mesh"),
            symbolic=symbolic,
            meshdir="meshes/THOR",
        )

        self.qr = np.zeros(6) 
        self.qz = np.array([0, -pi / 2, 0, 0, 0, 0])

        # nominal table top picking pose
        #self.qn = np.array([0, pi / 4, pi, 0, pi / 4, 0])

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        #self.addconfiguration("qn", self.qn)

        # straight and horizontal
        self.addconfiguration_attr("qs", np.array([0, 0, -pi / 2, 0, 0, 0]))

    def ikine_a(self, T, config="lun"):
        """
        Analytic inverse kinematic solution

        :param T: end-effector pose
        :type T: SE3
        :param config: arm configuration, defaults to "lun"
        :type config: str, optional
        :return: joint angle vector in radians
        :rtype: ndarray(6)

        ``robot.ikine_a(T, config)`` is the joint angle vector which achieves the
        end-effector pose ``T```.  The configuration string selects the specific
        solution and is a sting comprising the following letters:

        ======   ==============================================
        Letter   Meaning
        ======   ==============================================
        l        Choose the left-handed configuration
        r        Choose the right-handed configuration
        u        Choose the elbow up configuration
        d        Choose the elbow down configuration
        n        Choose the wrist not-flipped configuration
        f        Choose the wrist flipped configuration
        ======   ==============================================


        :reference:
            - Inverse kinematics for a PUMA 560,
              Paul and Zhang,
              The International Journal of Robotics Research,
              Vol. 5, No. 2, Summer 1986, p. 32-44

        :author: based on MATLAB code by Robert Biro with Gary Von McMurray,
            GTRI/ATRP/IIMB, Georgia Institute of Technology, 2/13/95

        """

        def ik3(robot, T, config="lun"):

            config = self.config_validate(config, ("lr", "ud", "nf"))

            # solve for the first three joints

            a2 = robot.links[1].a
            a3 = robot.links[2].a
            d1 = robot.links[0].d
            d3 = robot.links[2].d
            d4 = robot.links[3].d

            # The following parameters are extracted from the Homogeneous
            # Transformation as defined in equation 1, p. 34

            Px, Py, Pz = T.t
            Pz -= d1  # offset the pedestal height
            theta = np.zeros((3,))

            # Solve for theta[0]
            # r is defined in equation 38, p. 39.
            # theta[0] uses equations 40 and 41, p.39,
            # based on the configuration parameter n1

            r = np.sqrt(Px**2 + Py**2)
            if "r" in config:
                theta[0] = np.arctan2(Py, Px) + np.arcsin(d3 / r)
            elif "l" in config:
                theta[0] = np.arctan2(Py, Px) + np.pi - np.arcsin(d3 / r)
            else:
                raise ValueError("bad configuration string")

            # Solve for theta[1]
            # V114 is defined in equation 43, p.39.
            # r is defined in equation 47, p.39.
            # Psi is defined in equation 49, p.40.
            # theta[1] uses equations 50 and 51, p.40, based on the
            # configuration parameter n2
            if "u" in config:
                n2 = 1
            elif "d" in config:
                n2 = -1
            else:
                raise ValueError("bad configuration string")

            if "l" in config:
                n2 = -n2

            V114 = Px * np.cos(theta[0]) + Py * np.sin(theta[0])

            r = np.sqrt(V114**2 + Pz**2)

            with np.errstate(invalid="raise"):
                try:
                    Psi = np.arccos(
                        (a2**2 - d4**2 - a3**2 + V114**2 + Pz**2)
                        / (2.0 * a2 * r)
                    )
                except FloatingPointError:
                    return "Out of reach"

            theta[1] = np.arctan2(Pz, V114) + n2 * Psi

            # Solve for theta[2]
            # theta[2] uses equation 57, p. 40.
            num = np.cos(theta[1]) * V114 + np.sin(theta[1]) * Pz - a2
            den = np.cos(theta[1]) * Pz - np.sin(theta[1]) * V114
            theta[2] = np.arctan2(a3, d4) - np.arctan2(num, den)

            theta = base.angdiff(theta)

            return theta

            return self.ikine_6s(T, config, ik3)


if __name__ == "__main__":  # pragma nocover

    thor = ThorArm(symbolic=False)
    print(thor)
    #print(thor.dynamics())
    # T = thor.fkine(thor.qn)
    # print(thor.ikine_a(T, 'lu').q)
    # print(thor.ikine_a(T, 'ru').q)
    # print(thor.ikine_a(T, 'ld').q)
    # print(thor.ikine_a(T, 'rd').q)

    # thor.plot(thor.qz)

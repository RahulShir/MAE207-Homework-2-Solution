import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt

# For symbolic processing
import sympy
from sympy import symbols
from sympy import sin, cos, asin, acos, pi
from sympy.utilities.lambdify import lambdify
from sympy import Matrix


class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """

    #### Variables outside the init function are constants of the class
    # leg geometry
    l1 = 7  # NEED TO UPDATE units of cm
    l2 = 14.3  # NEED TO UPDATE units of cm
    l_base = 6  # NEED TO UPDATE units of cm

    # motor controller parameters
    encoder2angle = 360/(2048 * 4)
    angle2encoder = (2048 * 4)/360

    ### Methods
    # Classes are initiated with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables
    
##################################################-----MAKE FALSE-----######################################################## 
    def __init__(self, simulate = False):
        """
        This is the constructor for the leg class. Whenever you make a new leg
        this code will be called. We can optionally make a leg that will simulate
        the computations without needing to be connected to the ODrive
        """

        self.simulate = simulate

        # make the option to code without having the odrive connected
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  # easier handles to the motor commands
            self.m1 = self.drv.motor1

        else:
            self.drv = None

        # home angles
        self.joint_0_home = 0
        self.joint_1_home = 0

        # current positions
        (m0_pos, m1_pos) = self.get_joint_pos()
        self.joint_0_pos = m0_pos
        self.joint_1_pos = m1_pos

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        print('here2')
        self.J = self.compute_jacobian()
        print('Jacobian skeleton-function calculated!\nrun next section of code now-')
        #self.J_inv = self.J.pinv()

    def connect_to_controller(self):
        """
        Connects to the motor controller
        """
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###
    def get_joint_pos(self):
        """
        Get the current joint positions and store them in self.joint_0_pos and self.joint_1_pos in degrees.
        Also, return these positions using the return statement to terminate the function
        """
        # if simulating exit function
        if self.simulate == True:
            return(-1, -1)
            #return (self.joint_0_pos, self.joint_1_pos)
        else:
            self.joint_0_pos = self.m0.encoder.pll_pos*self.encoder2angle 
            self.joint_1_pos = self.m1.encoder.pll_pos*self.encoder2angle 
            return (self.joint_0_pos, self.joint_1_pos)


    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return
            #return(self.joint_0_home, self.joint_1_home)
        else:
            #self.joint_0_home = self.m0.encoder.pll_pos*self.encoder2angle 
            #self.joint_1_home = self.m1.encoder.pll_pos*self.encoder2angle 
            self.joint_0_home = 45
            self.joint_1_home = 180-45
            return(self.joint_0_home, self.joint_1_home)
        

    def set_joint_pos(self, theta_0, theta_1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            #m0.set_pos_setpoint(theta_0 + self.joint_0_home, vel0, curr0)
            #m1.set_pos_setpoint(theta_1 + self.joint_1_home, vel1, curr1)
            return
        else:
            #self.m0.set_pos_setpoint(theta_0*self.angle2encoder + self.joint_0_home*self.angle2encoder, vel0, curr0)
            #self.m1.set_pos_setpoint(theta_1*self.angle2encoder + self.joint_1_home*self.angle2encoder, vel1, curr1)
            self.m0.set_pos_setpoint(theta_0*self.angle2encoder + self.joint_0_home*self.angle2encoder, vel0, curr0)
            self.m1.set_pos_setpoint(theta_1*self.angle2encoder + self.joint_1_home*self.angle2encoder, vel1, curr1)
            return
        
        
    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            #m0.set_pos_setpoint(self.joint_0_home,vel0,curr0)
            #m1.set_pos_setpoint(self.joint_1_home,vel1,curr1)
            return
        else:
            self.m0.set_pos_setpoint(self.joint_0_home,0,0)
            self.m1.set_pos_setpoint(self.joint_1_home,0,0)
            return
        

    def set_foot_pos(self, x_target, y_target, theta0_current, theta1_current):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        (theta_0,theta_1) = self.inverse_kinematics(x_target, y_target, theta0_current, theta1_current)
        
        self.set_joint_pos(theta_0, theta_1, vel0=0, vel1=0, curr0=0, curr1=0)
        #time.sleep(0.2)
        return

    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        #ASSUMING STARTING POINT IS SAME AS HOME POSITION OF LEG
        theta0_current = self.joint_0_home 
        theta1_current = self.joint_1_home
        
        for k in range(len(tt)+1):
            x_target = xx[k]
            y_target = yy[k]
            #print(x_target)            
            self.set_foot_pos(x_target, y_target, theta0_current, theta1_current)
            (theta0_current, theta1_current) = self.get_joint_pos()
            print(k)
            self.draw_leg(ax=False)
        
        return

    
    
    
    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0, theta_1):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """        
        l1 = Leg.l1
        l2 = Leg.l2
        l_base = Leg.l_base
        
        d = (l_base**2 + l1**2 - 2*l_base*l1*math.cos(math.pi - math.pi*theta_0/180))**0.5
        bet = math.asin((l1*math.sin(math.pi - theta_0*math.pi/180))/d)
        
        
        aminus = (180/math.pi)*(math.acos((l1**2 + d**2 -2*l2**2 - 2*l1*d*math.cos((math.pi*theta_1/180) - bet))/(-2*l2**2)))
        aplus = 180
        
        alpha_0 = (aplus+aminus)/2
        alpha_1 = (aplus-aminus)/2
    
        return (alpha_0, alpha_1)
    

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """
        
        l1 = Leg.l1
        l2 = Leg.l2
        l_base = Leg.l_base
    

        # initiate the symbolic variables
        theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols(
            'theta0_sym theta1_sym alpha0_sym alpha1_sym', real=True)
        
        ##Calculate alpha values symbollically for use in differentiation
        d = (l_base**2 + l1**2 - 2*l_base*l1*cos(math.pi - math.pi*theta0_sym/180))**0.5
        bet = asin((l1*sin(math.pi - theta0_sym*math.pi/180))/d)
        
        aminus = (180/math.pi)*(acos((l1**2 + d**2 -2*l2**2 -2*l1*d*cos((math.pi*theta1_sym/180) - bet))/(-2*l2**2)))
        aplus = 180
        
        alpha0_sym = (aplus+aminus)/2
        #alpha0_sym.simplify()
        alpha1_sym = (aplus-aminus)/2
        #alpha1_sym.simplify()

        ##Calculate Jacobian symbollically (Choose a path along one of the legs to reach end point)
        x = l_base/2 + l1*cos(theta0_sym*math.pi/180) + l2*cos(alpha0_sym*math.pi/180) 
        #-l_base/2 + l1*cos(theta1_sym*pi/180) + l2*cos(alpha1_sym*pi/180)
        
        y = l1*sin(theta0_sym*math.pi/180) + l2*sin(alpha0_sym*math.pi/180) 
        # l1*sin(theta1_sym*pi/180) + l2*sin(alpha1_sym*pi/180)
        
        xypos = Matrix([[x] , [y]])
        sympy.simplify(xypos)
        
        
        J = xypos.jacobian([theta0_sym, theta1_sym])
        
        #J = []
        #for k in range(1):
            #J.append([])
            #J[k].append(xypos.diff(theta0_sym))
            #J[k].append(xypos.diff(theta1_sym))
            
        #J = Matrix(J)
        #sympy.simplify(J)
        #J.simplify()

        return J

    def inverse_kinematics(self, x_target, y_target, theta0_current, theta1_current):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """

        l1 = Leg.l1
        l2 = Leg.l2
        l_base = Leg.l_base
        
        theta_0 = theta0_current
        theta_1 = theta1_current
        
        eps = 0.0001
        delt = 0.001
        
        for k in range(10000):
            (alpha_0, alpha_1) = self.compute_internal_angles(theta_0, theta_1)
            
            x = l_base/2 + l1*math.cos(theta_0*math.pi/180) + l2*math.cos(alpha_0*math.pi/180) 
            y = l1*math.sin(theta_0*math.pi/180) + l2*math.sin(alpha_0*math.pi/180) 
            xypos = Matrix([[x] , [y]])
            
            xerror, yerror = (x_target - x), (y_target - y)
            
            if (xerror < eps) & (yerror < eps):
                print('target reached')
                break
                
            theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols(
            'theta0_sym theta1_sym alpha0_sym alpha1_sym', real=True)    
            J_current = self.J.subs({theta0_sym: theta_0, theta1_sym: theta_1, alpha0_sym: alpha_0, alpha1_sym: alpha_1}).evalf()
            #print(J_current)
            Jinv =J_current.pinv()
            
            Jmult = Jinv*xypos
            
            theta_0 = theta_0 + delt*Jmult[0]
            theta_1 = theta_1 + delt*Jmult[1]
            
            
        
        return (theta_0, theta_1)

    ###
    ### Visualization functions
    ###
    def draw_leg(self, ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """

        (theta2, theta1) = self.get_joint_pos()
        link1, link2, width = self.l1, self.l2, self.l_base
        theta2 = theta2*math.pi/180 + self.joint_0_home*math.pi/180
        theta1 = theta1*math.pi/180 + self.joint_1_home*math.pi/180

        alpha1, alpha2 = self.compute_internal_angles(theta1, theta2)
        alpha1 = theta1 + alpha1*math.pi/180 +math.pi
        alpha2 = theta2 + alpha2*math.pi/180 
        

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return (x, y)

        if ax == False:
            ax = plt.gca()
            ax.cla()

        ax.plot(-width / 2, 0, 'ok')
        ax.plot(width / 2, 0, 'ok')

        ax.plot([-width / 2, 0], [0, 0], 'k')
        ax.plot([width / 2, 0], [0, 0], 'k')

        ax.plot(-width / 2 + np.array([0, link1 * cos(theta1)]), [0, link1 * sin(theta1)], 'k')
        ax.plot(width / 2 + np.array([0, link1 * cos(theta2)]), [0, link1 * sin(theta2)], 'k')

        ax.plot(-width / 2 + link1 * cos(theta1) + np.array([0, link2 * cos(alpha1)]), \
                link1 * sin(theta1) + np.array([0, link2 * sin(alpha1)]), 'k');
        ax.plot(width / 2 + link1 * cos(theta2) + np.array([0, link2 * cos(alpha2)]), \
                np.array(link1 * sin(theta2) + np.array([0, link2 * sin(alpha2)])), 'k');

        ax.plot(width / 2 + link1 * cos(theta2) + link2 * cos(alpha2), \
                np.array(link1 * sin(theta2) + link2 * sin(alpha2)), 'ro');

        ax.axis([-20, 20, -20, 20])
        ax.invert_yaxis()
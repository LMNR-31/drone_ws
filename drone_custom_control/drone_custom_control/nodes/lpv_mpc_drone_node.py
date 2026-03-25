#!/usr/bin/env python3
'''
LICENSE AGREEMENT
Copyright of the original MPC code is owned by Mark Misin.
Adapted to ROS 2 + f450 parameters for FRL RoboCup Brasil 2025.
NO WARRANTY — use at your own risk.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from scipy.interpolate import CubicSpline

class SupportFilesDrone:
    def __init__(self):
        ''' Load the constants that do not change'''
        
        # Constants (F450)
        Ix=0.02375 # kg*m^2
        Iy=0.02375 # kg*m^2
        Iz=0.046875 # kg*m^2
        m=1.564 # kg
        g=9.81 # m/s^2
        max_thrust=40.18 # N
        Jtp=1.302*10**(-6) # N*m*s^2=kg*m^2
        Ts=0.1 # s

        # Matrix weights for the cost function (They must be diagonal)
        Q=np.matrix('10 0 0;0 10 0;0 0 10') # weights for outputs (all samples, except the last one)
        S=np.matrix('20 0 0;0 20 0;0 0 20') # weights for the final horizon period outputs
        R=np.matrix('10 0 0;0 10 0;0 0 10') # weights for inputs

        force_constant=0.00000012216
        torque_constant=0.07
        conv=(60/(2*np.pi))**2
        
        ct=force_constant*conv # N*s^2
        cq=torque_constant*ct # N*m*s^2
        l=0.225 # m

        controlled_states=3 # Number of attitude outputs: Phi, Theta, Psi
        
        hz=4 # horizon period
        innerDyn_length=4 # Number of inner control loop iterations

        # Complex poles
        px=np.array([-1.75+1.78j,-1.75-1.78j])
        py=np.array([-1.75+1.78j,-1.75-1.78j])
        pz=np.array([-4.0+3.5j,-4.0-3.5j])

        r=2
        f=0.025
        height_i=5
        height_f=25
        
        pos_x_y=0 # Default: 0. Make positive x and y longer for visual purposes (1-Yes, 0-No). It does not affect the dynamics of the UAV.
        sub_loop=5 # for animation purposes
        sim_version=1 # Can only be 1 or 2 - depending on that, it will show you different graphs in the animation
        
        # Drag force:
        drag_switch=0 # Must be either 0 or 1 (0 - drag force OFF, 1 - drag force ON)
        
        # Drag force coefficients [-]:
        C_D_u = 1.5
        C_D_v = 1.5
        C_D_w = 2.0
        
        # Drag force cross-section area [m^2]
        A_u=2*l*0.01+0.05**2
        A_v=2*l*0.01+0.05**2
        A_w=2*2*l*0.01+0.05**2
        
        # Air density
        rho=1.225 # [kg/m^3]
        
        # Choose the trajectory: only from 1-9
        trajectory=1

        self.constants = {
            'Ix': Ix, 'Iy': Iy, 'Iz': Iz, 'm': m, 'g': g, 'Jtp': Jtp, 'Ts': Ts,
            'Q': Q, 'S': S, 'R': R, 'ct': ct, 'cq': cq, 'l': l,
            'controlled_states': controlled_states, 'hz': hz, 'innerDyn_length': innerDyn_length,
            'px': px, 'py': py, 'pz': pz, 'r': r, 'f': f,
            'height_i': height_i, 'height_f': height_f, 'pos_x_y': pos_x_y,
            'sub_loop': sub_loop, 'sim_version': sim_version, 'drag_switch': drag_switch,
            'C_D_u': C_D_u, 'C_D_v': C_D_v, 'C_D_w': C_D_w, 'A_u': A_u, 'A_v': A_v, 'A_w': A_w,
            'rho': rho, 'trajectory': trajectory, 'max_thrust': max_thrust
        }

    def trajectory_generator(self,t):
        '''This method creates the trajectory for a drone to follow'''

        Ts=self.constants['Ts']
        innerDyn_length=self.constants['innerDyn_length']
        r=self.constants['r']
        f=self.constants['f']
        height_i=self.constants['height_i']
        height_f=self.constants['height_f']
        trajectory=self.constants['trajectory']
        d_height=height_f-height_i

        # Define the x, y, z dimensions for the drone trajectories
        alpha=2*np.pi*f*t

        if trajectory==1 or trajectory==2 or trajectory==3 or trajectory==4:
            # Trajectory 1
            x=r*np.cos(alpha)
            y=r*np.sin(alpha)
            z=height_i+d_height/(t[-1])*t

            x_dot=-r*np.sin(alpha)*2*np.pi*f
            y_dot=r*np.cos(alpha)*2*np.pi*f
            z_dot=d_height/(t[-1])*np.ones(len(t))

            x_dot_dot=-r*np.cos(alpha)*(2*np.pi*f)**2
            y_dot_dot=-r*np.sin(alpha)*(2*np.pi*f)**2
            z_dot_dot=0*np.ones(len(t))

            if trajectory==2:
                # Trajectory 2
                # Make sure you comment everything except Trajectory 1 and this bonus trajectory
                x[101:len(x)]=2*(t[101:len(t)]-t[100])/20+x[100]
                y[101:len(y)]=2*(t[101:len(t)]-t[100])/20+y[100]
                z[101:len(z)]=z[100]+d_height/t[-1]*(t[101:len(t)]-t[100])

                x_dot[101:len(x_dot)]=1/10*np.ones(len(t[101:len(t)]))
                y_dot[101:len(y_dot)]=1/10*np.ones(len(t[101:len(t)]))
                z_dot[101:len(z_dot*(t/20))]=d_height/(t[-1])*np.ones(len(t[101:len(t)]))

                x_dot_dot[101:len(x_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                y_dot_dot[101:len(y_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                z_dot_dot[101:len(z_dot_dot)]=0*np.ones(len(t[101:len(t)]))

            elif trajectory==3:
                # Trajectory 3
                # Make sure you comment everything except Trajectory 1 and this bonus trajectory
                x[101:len(x)]=2*(t[101:len(t)]-t[100])/20+x[100]
                y[101:len(y)]=2*(t[101:len(t)]-t[100])/20+y[100]
                z[101:len(z)]=z[100]+d_height/t[-1]*(t[101:len(t)]-t[100])**2

                x_dot[101:len(x_dot)]=1/10*np.ones(len(t[101:len(t)]))
                y_dot[101:len(y_dot)]=1/10*np.ones(len(t[101:len(t)]))
                z_dot[101:len(z_dot)]=2*d_height/(t[-1])*(t[101:len(t)]-t[100])

                x_dot_dot[101:len(x_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                y_dot_dot[101:len(y_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                z_dot_dot[101:len(z_dot_dot)]=2*d_height/t[-1]*np.ones(len(t[101:len(t)]))

            elif trajectory==4:
                # Trajectory 4
                # Make sure you comment everything except Trajectory 1 and this bonus trajectory
                x[101:len(x)]=2*(t[101:len(t)]-t[100])/20+x[100]
                y[101:len(y)]=2*(t[101:len(t)]-t[100])/20+y[100]
                z[101:len(z)]=z[100]+50*d_height/t[-1]*np.sin(0.1*(t[101:len(t)]-t[100]))

                x_dot[101:len(x_dot)]=1/10*np.ones(len(t[101:len(t)]))
                y_dot[101:len(y_dot)]=1/10*np.ones(len(t[101:len(t)]))
                z_dot[101:len(z_dot)]=5*d_height/t[-1]*np.cos(0.1*(t[101:len(t)]-t[100]))

                x_dot_dot[101:len(x_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                y_dot_dot[101:len(y_dot_dot)]=0*np.ones(len(t[101:len(t)]))
                z_dot_dot[101:len(z_dot_dot)]=-0.5*d_height/t[-1]*np.sin(0.1*(t[101:len(t)]-t[100]))

        elif trajectory==5 or trajectory==6:
            if trajectory==5:
                power=1
            else:
                power=2

            if power == 1:
                # Trajectory 5
                r_2=r/15
                x=(r_2*t**power+r)*np.cos(alpha)
                y=(r_2*t**power+r)*np.sin(alpha)
                z=height_i+d_height/t[-1]*t

                x_dot=r_2*np.cos(alpha)-(r_2*t+r)*np.sin(alpha)*2*np.pi*f
                y_dot=r_2*np.sin(alpha)+(r_2*t+r)*np.cos(alpha)*2*np.pi*f
                z_dot=d_height/(t[-1])*np.ones(len(t))

                x_dot_dot=-r_2*np.sin(alpha)*4*np.pi*f-(r_2*t+r)*np.cos(alpha)*(2*np.pi*f)**2
                y_dot_dot=r_2*np.cos(alpha)*4*np.pi*f-(r_2*t+r)*np.sin(alpha)*(2*np.pi*f)**2
                z_dot_dot=0*np.ones(len(t))
            else:
                # Trajectory 6
                r_2=r/500
                x=(r_2*t**power+r)*np.cos(alpha)
                y=(r_2*t**power+r)*np.sin(alpha)
                z=height_i+d_height/t[-1]*t

                x_dot=power*r_2*t**(power-1)*np.cos(alpha)-(r_2*t**(power)+r)*np.sin(alpha)*2*np.pi*f
                y_dot=power*r_2*t**(power-1)*np.sin(alpha)+(r_2*t**(power)+r)*np.cos(alpha)*2*np.pi*f
                z_dot=d_height/(t[-1])*np.ones(len(t))

                x_dot_dot=(power*(power-1)*r_2*t**(power-2)*np.cos(alpha)-power*r_2*t**(power-1)*np.sin(alpha)*2*np.pi*f)-(power*r_2*t**(power-1)*np.sin(alpha)*2*np.pi*f+(r_2*t**power+r_2)*np.cos(alpha)*(2*np.pi*f)**2)
                y_dot_dot=(power*(power-1)*r_2*t**(power-2)*np.sin(alpha)+power*r_2*t**(power-1)*np.cos(alpha)*2*np.pi*f)+(power*r_2*t**(power-1)*np.cos(alpha)*2*np.pi*f-(r_2*t**power+r_2)*np.sin(alpha)*(2*np.pi*f)**2)
                z_dot_dot=0*np.ones(len(t))

        elif trajectory==7:
            # Trajectory 7
            x=2*t/20+1
            y=2*t/20-2
            z=height_i+d_height/t[-1]*t

            x_dot=1/10*np.ones(len(t))
            y_dot=1/10*np.ones(len(t))
            z_dot=d_height/(t[-1])*np.ones(len(t))

            x_dot_dot=0*np.ones(len(t))
            y_dot_dot=0*np.ones(len(t))
            z_dot_dot=0*np.ones(len(t))

        elif trajectory==8:
            # Trajectory 8
            x=r/5*np.sin(alpha)+t/100
            y=t/100-1
            z=height_i+d_height/t[-1]*t

            x_dot=r/5*np.cos(alpha)*2*np.pi*f+1/100
            y_dot=1/100*np.ones(len(t))
            z_dot=d_height/(t[-1])*np.ones(len(t))

            x_dot_dot=-r/5*np.sin(alpha)*(2*np.pi*f)**2
            y_dot_dot=0*np.ones(len(t))
            z_dot_dot=0*np.ones(len(t))

        elif trajectory==9:
            # Trajectory 9
            wave_w=1
            x=r*np.cos(alpha)
            y=r*np.sin(alpha)
            z=height_i+7*d_height/t[-1]*np.sin(wave_w*t)

            x_dot=-r*np.sin(alpha)*2*np.pi*f
            y_dot=r*np.cos(alpha)*2*np.pi*f
            z_dot=7*d_height/(t[-1])*np.cos(wave_w*t)*wave_w

            x_dot_dot=-r*np.cos(alpha)*(2*np.pi*f)**2
            y_dot_dot=-r*np.sin(alpha)*(2*np.pi*f)**2
            z_dot_dot=-7*d_height/(t[-1])*np.sin(wave_w*t)*wave_w**2

        else:
            print("You only have 9 trajectories. Choose a number from 1 to 9")
            exit()

        dx = x[1:len(x)] - x[0:len(x)-1]
        dy = y[1:len(y)] - y[0:len(y)-1]
        dz = z[1:len(z)] - z[0:len(z)-1]
        dx = np.append(np.array(dx[0]), dx)
        dy = np.append(np.array(dy[0]), dy)
        dz = np.append(np.array(dz[0]), dz)

        psi = np.zeros(len(x))
        psiInt = psi
        psi[0] = np.arctan2(y[0], x[0]) + np.pi/2
        psi[1:len(psi)] = np.arctan2(dy[1:len(dy)], dx[1:len(dx)])

        dpsi = psi[1:len(psi)] - psi[0:len(psi)-1]
        psiInt[0] = psi[0]
        for i in range(1, len(psiInt)):
            if dpsi[i-1] < -np.pi:
                psiInt[i] = psiInt[i-1] + (dpsi[i-1] + 2*np.pi)
            elif dpsi[i-1] > np.pi:
                psiInt[i] = psiInt[i-1] + (dpsi[i-1] - 2*np.pi)
            else:
                psiInt[i] = psiInt[i-1] + dpsi[i-1]

        return x, x_dot, x_dot_dot, y, y_dot, y_dot_dot, z, z_dot, z_dot_dot, psiInt

    def pos_controller(self,X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,Psi_ref,states,max_vz_accel=3.0):
        '''This function is a position controller - it computes the necessary U1 for the open loop system, and phi & theta angles for the MPC controller'''

        # Load the constants
        m=self.constants['m']
        g=self.constants['g']
        px=self.constants['px']
        py=self.constants['py']
        pz=self.constants['pz']


        # Assign the states
        # States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        u = states[0]
        v = states[1]
        w = states[2]
        x = states[6]
        y = states[7]
        z = states[8]
        phi = states[9]
        theta = states[10]
        psi = states[11]

        # Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
        R_x=np.array([[1, 0, 0],[0, np.cos(phi), -np.sin(phi)],[0, np.sin(phi), np.cos(phi)]])
        R_y=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
        R_z=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
        R_matrix=np.matmul(R_z,np.matmul(R_y,R_x))
        pos_vel_body=np.array([[u],[v],[w]])
        pos_vel_fixed=np.matmul(R_matrix,pos_vel_body)
        x_dot=pos_vel_fixed[0]
        y_dot=pos_vel_fixed[1]
        z_dot=pos_vel_fixed[2]

        # Compute the errors
        ex=X_ref-x
        ex_dot=X_dot_ref-x_dot
        ey=Y_ref-y
        ey_dot=Y_dot_ref-y_dot
        ez=Z_ref-z
        ez_dot=Z_dot_ref-z_dot

        # Compute the feedback constants
        kx1=(px[0]-(px[0]+px[1])/2)**2-(px[0]+px[1])**2/4
        kx2=px[0]+px[1]
        kx1=kx1.real
        kx2=kx2.real

        ky1=(py[0]-(py[0]+py[1])/2)**2-(py[0]+py[1])**2/4
        ky2=py[0]+py[1]
        ky1=ky1.real
        ky2=ky2.real

        kz1=(pz[0]-(pz[0]+pz[1])/2)**2-(pz[0]+pz[1])**2/4
        kz2=pz[0]+pz[1]
        kz1=kz1.real
        kz2=kz2.real

        # Compute the values vx, vy, vz for the position controller
        ux=kx1*ex+kx2*ex_dot
        uy=ky1*ey+ky2*ey_dot
        uz=kz1*ez+kz2*ez_dot

        vx = X_dot_dot_ref-ux[0]
        vy = Y_dot_dot_ref-uy[0]
        vz = Z_dot_dot_ref-uz[0]

        # Clamp vertical acceleration demand to prevent thrust saturation
        vz = float(np.clip(vz, -max_vz_accel, max_vz_accel))

        # Compute phi, theta, U1
        a=vx/(vz+g)
        b=vy/(vz+g)
        c=np.cos(Psi_ref)
        d=np.sin(Psi_ref)
        tan_theta=a*c+b*d
        Theta_ref=np.arctan(tan_theta)

        if Psi_ref>=0:
            Psi_ref_singularity=Psi_ref-np.floor(abs(Psi_ref)/(2*np.pi))*2*np.pi
        else:
            Psi_ref_singularity=Psi_ref+np.floor(abs(Psi_ref)/(2*np.pi))*2*np.pi

        if ((np.abs(Psi_ref_singularity)<np.pi/4 or np.abs(Psi_ref_singularity)>7*np.pi/4) or (np.abs(Psi_ref_singularity)>3*np.pi/4 and np.abs(Psi_ref_singularity)<5*np.pi/4)):
            tan_phi=np.cos(Theta_ref)*(np.tan(Theta_ref)*d-b)/c
        else:
            tan_phi=np.cos(Theta_ref)*(a-np.tan(Theta_ref)*c)/d
        Phi_ref=np.arctan(tan_phi)
        U1=(vz+g)*m/(np.cos(Phi_ref)*np.cos(Theta_ref))

        return Phi_ref, Theta_ref, U1

    def LPV_cont_discrete(self, states, omega_total):
        '''This is an LPV model concerning the three rotational axis.'''

        # Get the necessary constants
        Ix=self.constants['Ix'] # kg*m^2
        Iy=self.constants['Iy'] # kg*m^2
        Iz=self.constants['Iz'] # kg*m^2
        Jtp=self.constants['Jtp'] #N*m*s^2=kg*m^2
        Ts=self.constants['Ts'] #s

        # Assign the states
        # States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        u=states[0]
        v=states[1]
        w=states[2]
        p=states[3]
        q=states[4]
        r=states[5]
        phi=states[9]
        theta=states[10]
        psi=states[11]

        # Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
        R_x=np.array([[1, 0, 0],[0, np.cos(phi), -np.sin(phi)],[0, np.sin(phi), np.cos(phi)]])
        R_y=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
        R_z=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
        R_matrix=np.matmul(R_z,np.matmul(R_y,R_x))
        pos_vel_body=np.array([[u],[v],[w]])
        pos_vel_fixed=np.matmul(R_matrix,pos_vel_body)
        x_dot=pos_vel_fixed[0]
        y_dot=pos_vel_fixed[1]
        z_dot=pos_vel_fixed[2]
        x_dot=x_dot[0]
        y_dot=y_dot[0]
        z_dot=z_dot[0]

        # To get phi_dot, theta_dot, psi_dot, you need the T matrix
        # Transformation matrix that relates p,q,r with phi_dot,theta_dot,psi_dot
        T_matrix=np.array([[1,np.sin(phi)*np.tan(theta),np.cos(phi)*np.tan(theta)],\
            [0,np.cos(phi),-np.sin(phi)],\
            [0,np.sin(phi)/np.cos(theta),np.cos(phi)/np.cos(theta)]])
        rot_vel_body=np.array([[p],[q],[r]])
        rot_vel_fixed=np.matmul(T_matrix,rot_vel_body)
        phi_dot=rot_vel_fixed[0]
        theta_dot=rot_vel_fixed[1]
        psi_dot=rot_vel_fixed[2]
        phi_dot=phi_dot[0]
        theta_dot=theta_dot[0]
        psi_dot=psi_dot[0]

        # Create the continuous LPV A, B, C, D matrices
        A01=1
        A13=-omega_total*Jtp/Ix
        A15=theta_dot*(Iy-Iz)/Ix
        A23=1
        A31=omega_total*Jtp/Iy
        A35=phi_dot*(Iz-Ix)/Iy
        A45=1
        A51=(theta_dot/2)*(Ix-Iy)/Iz
        A53=(phi_dot/2)*(Ix-Iy)/Iz

        A=np.zeros((6,6))
        B=np.zeros((6,3))
        C=np.zeros((3,6))
        D=0

        A[0,1]=A01
        A[1,3]=A13
        A[1,5]=A15
        A[2,3]=A23
        A[3,1]=A31
        A[3,5]=A35
        A[4,5]=A45
        A[5,1]=A51
        A[5,3]=A53

        B[1,0]=1/Ix
        B[3,1]=1/Iy
        B[5,2]=1/Iz

        C[0,0]=1
        C[1,2]=1
        C[2,4]=1

        D=np.zeros((3,3))

        # Discretize the system (Forward Euler)
        Ad=np.identity(np.size(A,1))+Ts*A
        Bd=Ts*B
        Cd=C
        Dd=D

        return Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot

    def mpc_simplification(self,Ad,Bd,Cd,Dd,hz):
        '''This function creates the compact matrices for Model Predictive Control'''
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex
        A_aug=np.concatenate((Ad,Bd),axis=1)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)
        D_aug=Dd


        Q=self.constants['Q']
        S=self.constants['S']
        R=self.constants['R']

        CQC=np.matmul(np.transpose(C_aug),Q)
        CQC=np.matmul(CQC,C_aug)

        CSC=np.matmul(np.transpose(C_aug),S)
        CSC=np.matmul(CSC,C_aug)

        QC=np.matmul(Q,C_aug)
        SC=np.matmul(S,C_aug)


        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R

            for j in range(0,hz):
                if j<=i:
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=np.matmul(np.linalg.matrix_power(A_aug,((i+1)-(j+1))),B_aug)

            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=np.linalg.matrix_power(A_aug,i+1)

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)

        return Hdb,Fdbt,Cdb,Adc

    def open_loop_new_states(self,states,omega_total,U1,U2,U3,U4):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        Ix=self.constants['Ix']
        Iy=self.constants['Iy']
        Iz=self.constants['Iz']
        m=self.constants['m']
        g=self.constants['g']
        Jtp=self.constants['Jtp']
        Ts=self.constants['Ts']

        # States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]
        current_states=states
        new_states=current_states
        u = current_states[0]
        v = current_states[1]
        w = current_states[2]
        p = current_states[3]
        q = current_states[4]
        r = current_states[5]
        x = current_states[6]
        y = current_states[7]
        z = current_states[8]
        phi = current_states[9]
        theta = current_states[10]
        psi = current_states[11]
        sub_loop=self.constants['sub_loop'] # Chop Ts into 5 pieces
        states_ani=np.zeros((sub_loop,6))
        U_ani=np.zeros((sub_loop,4))

        # Drag force:
        drag_switch=self.constants['drag_switch']
        C_D_u=self.constants['C_D_u']
        C_D_v=self.constants['C_D_v']
        C_D_w=self.constants['C_D_w']
        A_u=self.constants['A_u']
        A_v=self.constants['A_v']
        A_w=self.constants['A_w']
        rho=self.constants['rho']

        # Runge-Kutta method
        u_or=u
        v_or=v
        w_or=w
        p_or=p
        q_or=q
        r_or=r
        x_or=x
        y_or=y
        z_or=z
        phi_or=phi
        theta_or=theta
        psi_or=psi

        Ts_pos=2

        for j in range(0,4):

            if drag_switch==1:
                Fd_u=0.5*C_D_u*rho*u**2*A_u
                Fd_v=0.5*C_D_v*rho*v**2*A_v
                Fd_w=0.5*C_D_w*rho*w**2*A_w
            elif drag_switch==0:
                Fd_u=0
                Fd_v=0
                Fd_w=0
            else:
                print("drag_switch variable must be either 0 or 1 in the init function")
                exit()
            # print(u)
            # print(v)

            # Compute slopes k_x
            u_dot=(v*r-w*q)+g*np.sin(theta)-Fd_u/m
            v_dot=(w*p-u*r)-g*np.cos(theta)*np.sin(phi)-Fd_v/m
            w_dot=(u*q-v*p)-g*np.cos(theta)*np.cos(phi)+U1/m-Fd_w/m
            p_dot=q*r*(Iy-Iz)/Ix-Jtp/Ix*q*omega_total+U2/Ix
            q_dot=p*r*(Iz-Ix)/Iy+Jtp/Iy*p*omega_total+U3/Iy
            r_dot=p*q*(Ix-Iy)/Iz+U4/Iz

            # Get the states in the inertial frame
            # Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
            R_x=np.array([[1, 0, 0],[0, np.cos(phi), -np.sin(phi)],[0, np.sin(phi), np.cos(phi)]])
            R_y=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
            R_z=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
            R_matrix=np.matmul(R_z,np.matmul(R_y,R_x))

            pos_vel_body=np.array([[u],[v],[w]])
            pos_vel_fixed=np.matmul(R_matrix,pos_vel_body)
            x_dot=pos_vel_fixed[0]
            y_dot=pos_vel_fixed[1]
            z_dot=pos_vel_fixed[2]
            x_dot=x_dot[0]
            y_dot=y_dot[0]
            z_dot=z_dot[0]

            # To get phi_dot, theta_dot, psi_dot, you need the T matrix
            # Transformation matrix that relates p,q,r with phi_dot,theta_dot,psi_dot
            T_matrix=np.array([[1,np.sin(phi)*np.tan(theta),np.cos(phi)*np.tan(theta)],\
                [0,np.cos(phi),-np.sin(phi)],\
                [0,np.sin(phi)/np.cos(theta),np.cos(phi)/np.cos(theta)]])
            rot_vel_body=np.array([[p],[q],[r]])
            rot_vel_fixed=np.matmul(T_matrix,rot_vel_body)
            phi_dot=rot_vel_fixed[0]
            theta_dot=rot_vel_fixed[1]
            psi_dot=rot_vel_fixed[2]
            phi_dot=phi_dot[0]
            theta_dot=theta_dot[0]
            psi_dot=psi_dot[0]

            # Save the slopes:
            if j == 0:
                u_dot_k1=u_dot
                v_dot_k1=v_dot
                w_dot_k1=w_dot
                p_dot_k1=p_dot
                q_dot_k1=q_dot
                r_dot_k1=r_dot
                x_dot_k1=x_dot
                y_dot_k1=y_dot
                z_dot_k1=z_dot
                phi_dot_k1=phi_dot
                theta_dot_k1=theta_dot
                psi_dot_k1=psi_dot
            elif j == 1:
                u_dot_k2=u_dot
                v_dot_k2=v_dot
                w_dot_k2=w_dot
                p_dot_k2=p_dot
                q_dot_k2=q_dot
                r_dot_k2=r_dot
                x_dot_k2=x_dot
                y_dot_k2=y_dot
                z_dot_k2=z_dot
                phi_dot_k2=phi_dot
                theta_dot_k2=theta_dot
                psi_dot_k2=psi_dot
            elif j == 2:
                u_dot_k3=u_dot
                v_dot_k3=v_dot
                w_dot_k3=w_dot
                p_dot_k3=p_dot
                q_dot_k3=q_dot
                r_dot_k3=r_dot
                x_dot_k3=x_dot
                y_dot_k3=y_dot
                z_dot_k3=z_dot
                phi_dot_k3=phi_dot
                theta_dot_k3=theta_dot
                psi_dot_k3=psi_dot

                Ts_pos=1
            else:
                u_dot_k4=u_dot
                v_dot_k4=v_dot
                w_dot_k4=w_dot
                p_dot_k4=p_dot
                q_dot_k4=q_dot
                r_dot_k4=r_dot
                x_dot_k4=x_dot
                y_dot_k4=y_dot
                z_dot_k4=z_dot
                phi_dot_k4=phi_dot
                theta_dot_k4=theta_dot
                psi_dot_k4=psi_dot

            if j<3:
                # New states using k_x
                u=u_or+u_dot*Ts/Ts_pos
                v=v_or+v_dot*Ts/Ts_pos
                w=w_or+w_dot*Ts/Ts_pos
                p=p_or+p_dot*Ts/Ts_pos
                q=q_or+q_dot*Ts/Ts_pos
                r=r_or+r_dot*Ts/Ts_pos
                x=x_or+x_dot*Ts/Ts_pos
                y=y_or+y_dot*Ts/Ts_pos
                z=z_or+z_dot*Ts/Ts_pos
                phi=phi_or+phi_dot*Ts/Ts_pos
                theta=theta_or+theta_dot*Ts/Ts_pos
                psi=psi_or+psi_dot*Ts/Ts_pos
            else:
                # New states using average k_x
                u=u_or+1/6*(u_dot_k1+2*u_dot_k2+2*u_dot_k3+u_dot_k4)*Ts
                v=v_or+1/6*(v_dot_k1+2*v_dot_k2+2*v_dot_k3+v_dot_k4)*Ts
                w=w_or+1/6*(w_dot_k1+2*w_dot_k2+2*w_dot_k3+w_dot_k4)*Ts
                p=p_or+1/6*(p_dot_k1+2*p_dot_k2+2*p_dot_k3+p_dot_k4)*Ts
                q=q_or+1/6*(q_dot_k1+2*q_dot_k2+2*q_dot_k3+q_dot_k4)*Ts
                r=r_or+1/6*(r_dot_k1+2*r_dot_k2+2*r_dot_k3+r_dot_k4)*Ts
                x=x_or+1/6*(x_dot_k1+2*x_dot_k2+2*x_dot_k3+x_dot_k4)*Ts
                y=y_or+1/6*(y_dot_k1+2*y_dot_k2+2*y_dot_k3+y_dot_k4)*Ts
                z=z_or+1/6*(z_dot_k1+2*z_dot_k2+2*z_dot_k3+z_dot_k4)*Ts
                phi=phi_or+1/6*(phi_dot_k1+2*phi_dot_k2+2*phi_dot_k3+phi_dot_k4)*Ts
                theta=theta_or+1/6*(theta_dot_k1+2*theta_dot_k2+2*theta_dot_k3+theta_dot_k4)*Ts
                psi=psi_or+1/6*(psi_dot_k1+2*psi_dot_k2+2*psi_dot_k3+psi_dot_k4)*Ts

        for k in range(0,sub_loop):
            states_ani[k,0]=x_or+(x-x_or)/Ts*(k/(sub_loop-1))*Ts
            states_ani[k,1]=y_or+(y-y_or)/Ts*(k/(sub_loop-1))*Ts
            states_ani[k,2]=z_or+(z-z_or)/Ts*(k/(sub_loop-1))*Ts
            states_ani[k,3]=phi_or+(phi-phi_or)/Ts*(k/(sub_loop-1))*Ts
            states_ani[k,4]=theta_or+(theta-theta_or)/Ts*(k/(sub_loop-1))*Ts
            states_ani[k,5]=psi_or+(psi-psi_or)/Ts*(k/(sub_loop-1))*Ts

        U_ani[:,0]=U1
        U_ani[:,1]=U2
        U_ani[:,2]=U3
        U_ani[:,3]=U4

        # End of Runge-Kutta method

        # Take the last states
        new_states[0]=u
        new_states[1]=v
        new_states[2]=w
        new_states[3]=p
        new_states[4]=q
        new_states[5]=r

        new_states[6]=x
        new_states[7]=y
        new_states[8]=z
        new_states[9]=phi
        new_states[10]=theta
        new_states[11]=psi

        return new_states, states_ani, U_ani



# ==================== ROS2 NODE ====================
class LPVMPC_Drone(Node):
    """LPV-MPC drone controller with full flight FSM.

    States
    ------
    0 WAIT_WP        – idle, waiting for a waypoint / goal command
    1 TAKEOFF        – arm + OFFBOARD, climb to target altitude
    2 HOVER          – hold current goal position
    3 TRAJECTORY     – execute discrete-waypoint trajectory (type B)
    4 LANDING        – descend until landed, then branch on landing_mode
    5 STANDBY_GROUND – (landing_mode==0 only) remain OFFBOARD+ARM on ground
    """

    # ── FSM state constants ──────────────────────────────────────────────────
    _WAIT_WP        = 0
    _TAKEOFF        = 1
    _HOVER          = 2
    _TRAJECTORY     = 3
    _LANDING        = 4
    _STANDBY_GROUND = 5

    # Number of 100 Hz cycles to stream setpoints before requesting OFFBOARD
    _WARMUP_CYCLES = 200   # = 2 s

    def __init__(self):
        super().__init__('lpv_mpc_drone_node')

        # ── Declare & read parameters ────────────────────────────────────────
        self.declare_parameter('uav_name',              'uav1')
        self.declare_parameter('invert_attitude',       False)
        self.declare_parameter('enabled',               True)
        self.declare_parameter('override_active',       False)
        self.declare_parameter('landing_mode',          1)
        self.declare_parameter('hover_altitude',        2.0)
        self.declare_parameter('hover_altitude_margin', 0.3)
        self.declare_parameter('land_z_threshold',      0.3)
        self.declare_parameter('waypoint_duration',     5.0)
        self.declare_parameter('activation_timeout',    10.0)
        self.declare_parameter('landing_timeout',       5.0)
        self.declare_parameter('max_ref_speed_xy',      0.5)
        self.declare_parameter('max_ref_speed_z',       0.5)
        self.declare_parameter('use_velocity_body',     True)
        self.declare_parameter('max_vz_accel',             1.5)
        self.declare_parameter('max_tilt_rad',             0.25)
        self.declare_parameter('thrust_warn_threshold',    0.95)
        self.declare_parameter('min_takeoff_alt_rel',      0.8)
        self.declare_parameter('takeoff_complete_hold_time', 1.5)

        # ── Thrust saturation recovery ────────────────────────────────────────
        # When thrust stays at or above saturation_thrust_threshold for
        # saturation_recovery_cycles consecutive MPC steps (~0.1 s each),
        # the node enters a recovery phase for saturation_recovery_duration
        # MPC steps.  During recovery tilt is clamped tighter and XY speed is
        # reduced so the vehicle prioritises vertical stability.
        self.declare_parameter('saturation_recovery_enabled',         True)
        self.declare_parameter('saturation_thrust_threshold',         0.95)
        self.declare_parameter('saturation_recovery_cycles',          5)
        self.declare_parameter('saturation_recovery_duration',        10)
        self.declare_parameter('saturation_recovery_max_tilt_rad',    0.15)
        self.declare_parameter('saturation_recovery_max_ref_speed_xy', 0.2)
        self.declare_parameter('saturation_recovery_freeze_xy',       False)

        # When True, transition WAIT_WP -> HOVER automatically if the drone is
        # already armed+OFFBOARD (e.g. hover_supervisor handed off control).
        # Publishes hold-at-current-position attitude setpoints immediately,
        # which also triggers hover_supervisor's auto_disable_when_mpc_active.
        self.declare_parameter('auto_arm_handoff', True)

        uav = self.get_parameter('uav_name').value

        self.enabled            = bool(self.get_parameter('enabled').value)
        self.override_active    = bool(self.get_parameter('override_active').value)
        self.landing_mode       = int(self.get_parameter('landing_mode').value)
        self.hover_altitude     = float(self.get_parameter('hover_altitude').value)
        self.hover_alt_margin   = float(self.get_parameter('hover_altitude_margin').value)
        self.land_z_threshold   = float(self.get_parameter('land_z_threshold').value)
        self.waypoint_duration  = float(self.get_parameter('waypoint_duration').value)
        self.activation_timeout = float(self.get_parameter('activation_timeout').value)
        self.landing_timeout    = float(self.get_parameter('landing_timeout').value)
        self.max_ref_speed_xy   = float(self.get_parameter('max_ref_speed_xy').value)
        self.max_ref_speed_z    = float(self.get_parameter('max_ref_speed_z').value)
        self.use_velocity_body  = bool(self.get_parameter('use_velocity_body').value)
        self.max_vz_accel = float(self.get_parameter('max_vz_accel').value)
        self.max_tilt_rad = float(self.get_parameter('max_tilt_rad').value)
        self.thrust_warn_threshold = float(self.get_parameter('thrust_warn_threshold').value)
        self.min_takeoff_alt_rel = float(
            self.get_parameter('min_takeoff_alt_rel').value)
        self.takeoff_complete_hold_time = float(
            self.get_parameter('takeoff_complete_hold_time').value)

        self.saturation_recovery_enabled = bool(
            self.get_parameter('saturation_recovery_enabled').value)
        self.saturation_thrust_threshold = float(
            self.get_parameter('saturation_thrust_threshold').value)
        self.saturation_recovery_cycles = int(
            self.get_parameter('saturation_recovery_cycles').value)
        self.saturation_recovery_duration = int(
            self.get_parameter('saturation_recovery_duration').value)
        self.saturation_recovery_max_tilt_rad = float(
            self.get_parameter('saturation_recovery_max_tilt_rad').value)
        self.saturation_recovery_max_ref_speed_xy = float(
            self.get_parameter('saturation_recovery_max_ref_speed_xy').value)
        self.saturation_recovery_freeze_xy = bool(
            self.get_parameter('saturation_recovery_freeze_xy').value)
        self.auto_arm_handoff = bool(
            self.get_parameter('auto_arm_handoff').value)

        if self.landing_mode not in (0, 1):
            self.get_logger().warn(
                f'landing_mode={self.landing_mode} invalid; using 1 (DISARM)')
            self.landing_mode = 1

        self.add_on_set_parameters_callback(self._on_params)

        # ── MPC support (SupportFilesDrone — unchanged) ──────────────────────
        self.support         = SupportFilesDrone()
        self.hz              = self.support.constants['hz']
        self.innerDyn_length = self.support.constants['innerDyn_length']
        self.Ts              = self.support.constants['Ts']
        self.omega_total     = 0.0

        # Drone state vector [u, v, w, p, q, r, x, y, z, phi, theta, psi]
        self.states = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
        self.U1     = self.support.constants['m'] * self.support.constants['g']
        self.U2 = self.U3 = self.U4 = 0.0

        # ── FSM variables ────────────────────────────────────────────────────
        self.state_voo     = self._WAIT_WP
        self.odom_received = False
        self.mavros_state  = State()

        # OFFBOARD / ARM request tracking
        self.offboard_activated   = False
        self.activation_confirmed = False
        self.activation_time      = self.get_clock().now()
        self._last_offboard_req   = None   # rclpy.Time or None
        self._last_arm_req        = None
        self._req_min_interval    = 1.0    # s – minimum interval between repeat requests

        # Warmup counter (stream setpoints before requesting OFFBOARD)
        self._warmup_counter = 0

        # Waypoint / trajectory
        self.last_waypoint_goal   = None   # [x, y, z]
        self.trajectory_waypoints = []     # list of [x, y, z]
        self.trajectory_started   = False
        self.trajectory_start_time = None
        self.current_waypoint_idx  = 0

        # Landing
        self.landing_active         = False
        self.landing_start_time_set = False
        self.landing_start_time     = self.get_clock().now()
        # ENU: z is positive when airborne; ground-hold uses a small positive z
        # so the pos_controller targets a few cm of altitude.
        self.ground_hold            = [0.0, 0.0, max(0.01, self.land_z_threshold)]

        # Psi tracking (trajectory, unwrapped)
        self.last_psi_ref = 0.0

        # Takeoff counter (cycles after OFFBOARD+ARM confirmed)
        self.takeoff_counter = 0

        # Last published command (republished at 100 Hz between MPC steps)
        self._hold_cmd = None

        # velocity_body subscriber storage (body-frame linear velocity)
        self._vel_body = np.zeros(3)
        self._vel_body_received = False

        # Altitude reference in uav1/map frame (captures ground z on first odom)
        self._z0_map = None
        # True once armed (latches _z0_map; cleared when disarmed back in WAIT_WP)
        self._z0_map_locked = False
        # Track previous armed state to detect arming transitions
        self._prev_armed = False
        # Timestamp when takeoff completion conditions were first continuously met
        self._takeoff_cond_start: float = None  # seconds (ROS time)

        # State-5 recovery interval timer
        self._state5_recovery_time = self.get_clock().now()

        # Thrust saturation recovery counters
        # _sat_counter: consecutive MPC steps where thrust >= saturation_thrust_threshold
        # _sat_recovery_remaining: MPC steps remaining in the current recovery phase
        self._sat_counter = 0
        self._sat_recovery_remaining = 0

        # Decimation: run MPC every N cycles (N = Ts / 0.01 = 10)
        self._mpc_decimation = max(1, round(self.Ts / 0.01))
        self._cycle_count    = 0

        # ── Publishers ───────────────────────────────────────────────────────
        self.att_pub = self.create_publisher(
            AttitudeTarget, f'/{uav}/mavros/setpoint_raw/attitude', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, f'/{uav}/mavros/local_position/odom', self.odom_cb, qos)
        self.create_subscription(
            TwistStamped,
            f'/{uav}/mavros/local_position/velocity_body',
            self._vel_body_cb, qos)
        self.create_subscription(
            State, f'/{uav}/mavros/state', self._state_cb, 10)
        self.create_subscription(
            PoseArray, '/waypoints', self._waypoints_cb, 1)
        self.create_subscription(
            PoseStamped, '/waypoint_goal', self._waypoint_goal_cb, 1)

        # ── Service clients ──────────────────────────────────────────────────
        self._mode_client = self.create_client(SetMode, f'/{uav}/mavros/set_mode')
        self._arm_client  = self.create_client(CommandBool, f'/{uav}/mavros/cmd/arming')

        # ── Timer: 100 Hz ────────────────────────────────────────────────────
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info(
            f'LPV-MPC FSM node started | uav={uav} '
            f'landing_mode={self.landing_mode} '
            f'hover_alt={self.hover_altitude:.1f}m '
            f'land_z_thr={self.land_z_threshold:.2f}m '
            f'warmup={self._WARMUP_CYCLES} cycles'
        )

    # ── ENU convention helpers ───────────────────────────────────────────────
    @staticmethod
    def _z_up_to_enu(z_up: float) -> float:
        """Convert positive-UP altitude to ENU z (positive when airborne).

        Incoming waypoints use z as positive altitude (e.g. z=+2.0 → 2 m AGL).
        Internally the controller uses ENU where z is positive when airborne,
        matching what MAVROS local_position/odom reports in PX4 Gazebo Harmonic.
        """
        return abs(float(z_up))

    @staticmethod
    def _enu_to_alt(z_enu: float) -> float:
        """Altitude magnitude for logs (ENU z is already positive-up)."""
        return max(0.0, float(z_enu))

    def _alt_rel(self, z_enu: float = None) -> float:
        """Altitude relative to captured ground reference (z0 in map frame).

        Parameters
        ----------
        z_enu : float, optional
            ENU z value to convert.  Defaults to ``states[8]`` (current altitude).

        Returns
        -------
        float
            ``z_enu - _z0_map`` when a ground reference is available,
            otherwise ``z_enu`` unchanged.
        """
        if z_enu is None:
            z_enu = float(self.states[8])
        if self._z0_map is not None:
            return float(z_enu) - self._z0_map
        return float(z_enu)

    # ── Dynamic parameter callback ───────────────────────────────────────────
    def _on_params(self, params):
        result = SetParametersResult(successful=True)
        for p in params:
            if p.name == 'enabled':
                old = self.enabled
                self.enabled = bool(p.value)
                if old != self.enabled:
                    self.get_logger().info(
                        f'enabled: {old} -> {self.enabled}')
            elif p.name == 'override_active':
                old = self.override_active
                self.override_active = bool(p.value)
                if not old and self.override_active:
                    self.get_logger().warn(
                        'override_active -> true: FSM frozen, publishing hold setpoint')
                elif old and not self.override_active:
                    self.get_logger().warn(
                        'override_active -> false: normal operation resumed')
            elif p.name == 'landing_mode':
                val = int(p.value)
                if val not in (0, 1):
                    result.successful = False
                    result.reason = 'landing_mode must be 0 (standby) or 1 (DISARM)'
                    return result
                self.landing_mode = val
                self.get_logger().info(
                    f'landing_mode -> {val} '
                    f'({"standby on ground" if val == 0 else "DISARM"})')
            elif p.name == 'max_vz_accel':
                self.max_vz_accel = float(p.value)
                self.get_logger().info(f'max_vz_accel -> {self.max_vz_accel:.2f} m/s²')
            elif p.name == 'max_tilt_rad':
                self.max_tilt_rad = float(p.value)
                self.get_logger().info(f'max_tilt_rad -> {self.max_tilt_rad:.3f} rad')
            elif p.name == 'thrust_warn_threshold':
                self.thrust_warn_threshold = float(p.value)
                self.get_logger().info(
                    f'thrust_warn_threshold -> {self.thrust_warn_threshold:.3f}')
            elif p.name == 'saturation_recovery_enabled':
                self.saturation_recovery_enabled = bool(p.value)
                self.get_logger().info(
                    f'saturation_recovery_enabled -> {self.saturation_recovery_enabled}')
            elif p.name == 'saturation_thrust_threshold':
                self.saturation_thrust_threshold = float(p.value)
                self.get_logger().info(
                    f'saturation_thrust_threshold -> {self.saturation_thrust_threshold:.3f}')
            elif p.name == 'saturation_recovery_cycles':
                self.saturation_recovery_cycles = int(p.value)
                self.get_logger().info(
                    f'saturation_recovery_cycles -> {self.saturation_recovery_cycles}')
            elif p.name == 'saturation_recovery_duration':
                self.saturation_recovery_duration = int(p.value)
                self.get_logger().info(
                    f'saturation_recovery_duration -> {self.saturation_recovery_duration}')
            elif p.name == 'saturation_recovery_max_tilt_rad':
                self.saturation_recovery_max_tilt_rad = float(p.value)
                self.get_logger().info(
                    f'saturation_recovery_max_tilt_rad -> '
                    f'{self.saturation_recovery_max_tilt_rad:.3f} rad')
            elif p.name == 'saturation_recovery_max_ref_speed_xy':
                self.saturation_recovery_max_ref_speed_xy = float(p.value)
                self.get_logger().info(
                    f'saturation_recovery_max_ref_speed_xy -> '
                    f'{self.saturation_recovery_max_ref_speed_xy:.2f} m/s')
            elif p.name == 'saturation_recovery_freeze_xy':
                self.saturation_recovery_freeze_xy = bool(p.value)
                self.get_logger().info(
                    f'saturation_recovery_freeze_xy -> {self.saturation_recovery_freeze_xy}')
            elif p.name == 'auto_arm_handoff':
                self.auto_arm_handoff = bool(p.value)
                self.get_logger().info(
                    f'auto_arm_handoff -> {self.auto_arm_handoff}')
        return result

    # ── Subscribers callbacks ────────────────────────────────────────────────
    def _state_cb(self, msg):
        prev = self._prev_armed
        self.mavros_state = msg
        armed_now = msg.armed

        # Detect disarm: if was armed and now disarmed while in WAIT_WP, unlock z0
        if prev and not armed_now and self.state_voo == self._WAIT_WP:
            self.get_logger().info(
                f'Disarmed in WAIT_WP -> unlocking _z0_map '
                f'(was_locked={self._z0_map_locked} state={self.state_voo} '
                f'armed={armed_now} z0={self._z0_map})')
            self._z0_map_locked = False

        # Detect first arm transition: lock _z0_map once vehicle arms
        if not prev and armed_now and not self._z0_map_locked:
            self._z0_map_locked = True
            self.get_logger().info(
                f'Armed! Latching _z0_map={self._z0_map} '
                f'(state={self.state_voo} armed={armed_now})')

        self._prev_armed = armed_now

    def _vel_body_cb(self, msg):
        """Cache the latest body-frame linear velocity from MAVROS."""
        self._vel_body = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z], dtype=float)
        self._vel_body_received = True

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        q = msg.pose.pose.orientation
        qw = np.array([q.x, q.y, q.z, q.w])
        if qw[3] < 0:
            qw = -qw
        phi   = np.arctan2(
            2*(qw[3]*qw[0] + qw[1]*qw[2]),
            1 - 2*(qw[0]**2 + qw[1]**2))
        theta = np.arcsin(np.clip(2*(qw[3]*qw[1] - qw[2]*qw[0]), -1.0, 1.0))
        psi   = np.arctan2(
            2*(qw[3]*qw[2] + qw[0]*qw[1]),
            1 - 2*(qw[1]**2 + qw[2]**2))

        # Capture ground reference: never update when locked (vehicle is armed)
        if self._z0_map is None:
            self._z0_map = float(p.z)
            self.get_logger().warn(
                f'_z0_map initial capture: z0={self._z0_map:.4f} '
                f'state={self.state_voo} armed={self.mavros_state.armed}')
        elif (not self._z0_map_locked
              and self.state_voo == self._WAIT_WP
              and not self.mavros_state.armed):
            old = self._z0_map
            self._z0_map = float(p.z)
            if abs(self._z0_map - old) > 0.01:
                self.get_logger().warn(
                    f'_z0_map recaptured: {old:.4f} -> {self._z0_map:.4f} '
                    f'state={self.state_voo} armed={self.mavros_state.armed}')

        # Select u,v,w source:
        # Preferred: MAVROS velocity_body (already in body frame, no conversion needed)
        # Fallback:  rotate odom twist.linear (world frame) into body frame
        if self.use_velocity_body and self._vel_body_received:
            u_b, v_b, w_b = self._vel_body
            vel_source = 'velocity_body'
        else:
            cp, sp = np.cos(phi), np.sin(phi)
            ct, st = np.cos(theta), np.sin(theta)
            cs, ss = np.cos(psi), np.sin(psi)
            R_x = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
            R_y = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
            R_z = np.array([[cs, -ss, 0], [ss, cs, 0], [0, 0, 1]])
            R_body_to_world = np.matmul(R_z, np.matmul(R_y, R_x))
            v_world = np.array([v.x, v.y, v.z])
            u_b, v_b, w_b = np.matmul(R_body_to_world.T, v_world)
            vel_source = 'odom_twist(world->body)'

        self.states = np.array([
            u_b, v_b, w_b,
            w.x, w.y, w.z,
            p.x, p.y, p.z,
            phi, theta, psi])
        self.odom_received = True

        # Throttled log: frame convention + velocity source (every ~10 s)
        self.get_logger().info(
            f"odom frames: frame_id={msg.header.frame_id!r} "
            f"child_frame_id={msg.child_frame_id!r} "
            f"vel_src={vel_source}",
            throttle_duration_sec=10.0)

    def _waypoints_cb(self, msg):
        """Handle PoseArray: 1 pose = takeoff/land, 2+ = trajectory."""
        if len(msg.poses) < 1:
            self.get_logger().warn('Empty waypoints message, ignoring')
            return

        # Incoming z is positive-UP altitude; kept raw for landing detection.
        last_z = msg.poses[-1].position.z

        # ── Single waypoint ──────────────────────────────────────────────────
        if len(msg.poses) == 1:
            if last_z < self.land_z_threshold:
                # Landing command
                if self.state_voo == self._STANDBY_GROUND:
                    self.get_logger().info(
                        'Landing command ignored: already on ground (state 5)',
                        throttle_duration_sec=5.0)
                    return
                self.get_logger().warn(
                    f'LANDING commanded via waypoints! alt={last_z:.2f}m')
                # ENU: small positive z keeps the drone at a few cm of altitude
                self.ground_hold = [
                    msg.poses[0].position.x,
                    msg.poses[0].position.y,
                    max(0.01, self.land_z_threshold)]
                self._initiate_landing()
            else:
                # Takeoff / hover command – store altitude as ENU z
                z_enu = self._z_up_to_enu(msg.poses[0].position.z)
                self.get_logger().info(
                    f'TAKEOFF/HOVER waypoint: '
                    f'X={msg.poses[0].position.x:.2f} '
                    f'Y={msg.poses[0].position.y:.2f} '
                    f'alt={last_z:.2f}m (z_enu={z_enu:.2f})')
                self.last_waypoint_goal = [
                    msg.poses[0].position.x,
                    msg.poses[0].position.y,
                    z_enu]
                self.trajectory_waypoints.clear()
                self.trajectory_started = False
                self.current_waypoint_idx = 0
                self._initiate_takeoff()
            return

        # ── Trajectory (2+ waypoints) ────────────────────────────────────────
        if self.state_voo in (self._LANDING, self._STANDBY_GROUND):
            self.get_logger().warn(
                f'Ignoring trajectory while in state {self.state_voo} '
                f'(landing/standby ground)')
            return

        # Store each waypoint's altitude (positive-UP) as ENU z
        self.trajectory_waypoints = [
            [p.position.x, p.position.y, self._z_up_to_enu(p.position.z)]
            for p in msg.poses]
        self.trajectory_started   = False
        self.current_waypoint_idx = 0

        self.get_logger().info(
            f'Trajectory stored: {len(self.trajectory_waypoints)} waypoints '
            f'({self.waypoint_duration:.1f}s each)')
        for i, wp in enumerate(self.trajectory_waypoints):
            self.get_logger().info(
                f'  WP[{i}]: X={wp[0]:.2f} Y={wp[1]:.2f} '
                f'z_enu={wp[2]:.2f} (alt={self._enu_to_alt(wp[2]):.2f}m)')

        if self.state_voo == self._HOVER:
            self.state_voo = self._TRAJECTORY
            self.get_logger().info('HOVER -> TRAJECTORY (trajectory activated)')
        else:
            self.get_logger().info(
                f'Trajectory stored; will activate when HOVER is reached '
                f'(current state={self.state_voo})')

    def _waypoint_goal_cb(self, msg):
        """Handle PoseStamped single goal."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Incoming z is positive-UP altitude; store as ENU z for internal use.
        z_up = msg.pose.position.z
        z_enu = self._z_up_to_enu(z_up)

        # Landing detection while in flight (compare raw altitude)
        if self.state_voo in (self._HOVER, self._TRAJECTORY):
            if z_up < self.land_z_threshold:
                self.get_logger().warn(
                    f'LANDING via goal! alt={z_up:.2f}m')
                # ENU: small positive z keeps pos_controller at a few cm altitude
                self.ground_hold = [x, y, max(0.01, self.land_z_threshold)]
                self._initiate_landing()
                return

        # Ignore new goals during active landing
        if self.state_voo == self._LANDING:
            if not self.mavros_state.armed:
                # Drone disarmed: ready for new cycle
                self.get_logger().info('Drone disarmed during landing -> WAIT_WP')
                self.offboard_activated   = False
                self.activation_confirmed = False
                self.state_voo            = self._WAIT_WP
                self.takeoff_counter      = 0
                self.landing_active       = False
            return

        # State 5: accept flight waypoints to resume
        if self.state_voo == self._STANDBY_GROUND:
            if z_up >= self.land_z_threshold:
                self.get_logger().info(
                    f'[State 5] Flight waypoint received, resuming: '
                    f'X={x:.2f} Y={y:.2f} alt={z_up:.2f}m (z_enu={z_enu:.2f})')
                self.last_waypoint_goal   = [x, y, z_enu]
                self.trajectory_waypoints.clear()
                self.trajectory_started   = False
                self.current_waypoint_idx = 0
                self.landing_active       = False
                self._initiate_takeoff()
            else:
                self.get_logger().info(
                    '[State 5] Landing waypoint ignored (already on ground)',
                    throttle_duration_sec=5.0)
            return

        # State 3 (trajectory): store as override setpoint (ENU z)
        if self.state_voo == self._TRAJECTORY:
            self.last_waypoint_goal = [x, y, z_enu]
            return

        # Normal: update goal (store ENU z)
        self.last_waypoint_goal = [x, y, z_enu]
        self.get_logger().info(
            f'Waypoint goal updated: X={x:.2f} Y={y:.2f} '
            f'alt={z_up:.2f}m (z_enu={z_enu:.2f})')

    # ── FSM helpers ──────────────────────────────────────────────────────────
    def _initiate_landing(self):
        """Transition to LANDING state."""
        self.landing_active         = True
        self.landing_start_time_set = False
        self.state_voo              = self._LANDING
        self.get_logger().warn('-> LANDING state')

    def _initiate_takeoff(self):
        """Transition to TAKEOFF state, setting up warmup / activation."""
        if self.last_waypoint_goal is None:
            self.get_logger().warn('_initiate_takeoff: no waypoint goal set')
            return

        self.landing_active      = False
        self.state_voo           = self._TAKEOFF
        self.takeoff_counter     = 0
        self._takeoff_cond_start = None

        # If already OFFBOARD+ARM (e.g. from state 5), skip warmup
        if (self.mavros_state.armed
                and self.mavros_state.mode == 'OFFBOARD'):
            self._warmup_counter      = self._WARMUP_CYCLES
            self.offboard_activated   = True
            self.activation_confirmed = True
            self.get_logger().info(
                f'TAKEOFF: already OFFBOARD+ARM – skipping warmup '
                f'-> goal={self.last_waypoint_goal}')
        else:
            self._warmup_counter      = 0
            self.offboard_activated   = False
            self.activation_confirmed = False
            self.get_logger().info(
                f'TAKEOFF: warmup phase ({self._WARMUP_CYCLES} cycles) '
                f'-> goal={self.last_waypoint_goal}')

    # ── MAVROS service helpers ────────────────────────────────────────────────
    def _request_offboard(self):
        """Rate-limited async request for OFFBOARD mode."""
        now = self.get_clock().now()
        if self._last_offboard_req is not None:
            elapsed = (now - self._last_offboard_req).nanoseconds / 1e9
            if elapsed < self._req_min_interval:
                return
        if not self._mode_client.service_is_ready():
            self.get_logger().warn('set_mode service not ready')
            return
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self._mode_client.call_async(req)
        self._last_offboard_req = now
        self.get_logger().info('OFFBOARD mode requested')

    def _request_arm(self):
        """Rate-limited async ARM request."""
        now = self.get_clock().now()
        if self._last_arm_req is not None:
            elapsed = (now - self._last_arm_req).nanoseconds / 1e9
            if elapsed < self._req_min_interval:
                return
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('arming service not ready')
            return
        req = CommandBool.Request()
        req.value = True
        self._arm_client.call_async(req)
        self._last_arm_req = now
        self.get_logger().info('ARM requested')

    def _request_disarm(self):
        """Async DISARM request."""
        if not self._arm_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self._arm_client.call_async(req)
        self.get_logger().info('DISARM requested')

    # ── Reference computation ─────────────────────────────────────────────────
    def _goal_ref(self, gx, gy, gz):
        """Return MPC reference tuple for holding/approaching a fixed goal.

        During thrust saturation recovery the XY speed limit is tightened and,
        when ``saturation_recovery_freeze_xy`` is true, the XY goal is frozen to
        the current position so no lateral acceleration is demanded.

        Returns
        -------
        (X_ref, Xd, Xdd, Y_ref, Yd, Ydd, Z_ref, Zd, Zdd, psi_ref)
        """
        x_now = self.states[6]
        y_now = self.states[7]

        in_recovery = (self.saturation_recovery_enabled
                       and self._sat_recovery_remaining > 0)

        # Freeze XY goal to current position during recovery if configured
        if in_recovery and self.saturation_recovery_freeze_xy:
            gx, gy = x_now, y_now

        dx = gx - x_now
        dy = gy - y_now
        dz = gz - self.states[8]

        dist_xy = float(np.sqrt(dx**2 + dy**2))
        xy_speed_limit = (self.saturation_recovery_max_ref_speed_xy
                          if in_recovery else self.max_ref_speed_xy)
        if dist_xy > 0.01:
            vx = (dx / dist_xy) * min(dist_xy, xy_speed_limit)
            vy = (dy / dist_xy) * min(dist_xy, xy_speed_limit)
        else:
            vx = vy = 0.0

        vz = float(np.clip(dz, -self.max_ref_speed_z, self.max_ref_speed_z))

        return (gx, vx, 0.0, gy, vy, 0.0, gz, vz, 0.0, self.last_psi_ref)

    def _trajectory_ref(self, now):
        """Return MPC reference tuple for trajectory execution (type B).

        Selects the current discrete waypoint based on elapsed time, computes
        velocity toward it (saturated), and updates the unwrapped psi reference.

        Returns None if trajectory is invalid.
        """
        if not self.trajectory_waypoints:
            self.get_logger().warn('_trajectory_ref: no waypoints, -> HOVER')
            self.state_voo = self._HOVER
            return None

        # Initialise timing on first call
        if not self.trajectory_started:
            self.trajectory_start_time = now
            self.trajectory_started    = True
            self.current_waypoint_idx  = 0
            self.get_logger().info(
                f'Trajectory started: {len(self.trajectory_waypoints)} WPs, '
                f'{self.waypoint_duration:.1f}s each')

        elapsed = (now - self.trajectory_start_time).nanoseconds / 1e9
        total   = self.waypoint_duration * len(self.trajectory_waypoints)

        idx = min(
            int(elapsed / self.waypoint_duration),
            len(self.trajectory_waypoints) - 1)

        if idx < 0 or idx >= len(self.trajectory_waypoints):
            self.get_logger().error(
                f'Waypoint index {idx} out of range, -> HOVER')
            self.state_voo = self._HOVER
            return None

        self.current_waypoint_idx = idx
        wp = self.trajectory_waypoints[idx]

        x_now = self.states[6]
        y_now = self.states[7]

        dx = wp[0] - x_now
        dy = wp[1] - y_now
        dz = wp[2] - self.states[8]

        dist_xy = float(np.sqrt(dx**2 + dy**2))
        in_recovery = (self.saturation_recovery_enabled
                       and self._sat_recovery_remaining > 0)
        xy_speed_limit = (self.saturation_recovery_max_ref_speed_xy
                          if in_recovery else self.max_ref_speed_xy)
        if dist_xy > 0.01:
            vx = (dx / dist_xy) * min(dist_xy, xy_speed_limit)
            vy = (dy / dist_xy) * min(dist_xy, xy_speed_limit)
        else:
            vx = vy = 0.0

        vz = float(np.clip(dz, -self.max_ref_speed_z, self.max_ref_speed_z))

        # Unwrapped psi reference from XY velocity direction
        speed_xy = float(np.sqrt(vx**2 + vy**2))
        if speed_xy > 0.05:
            psi_new = float(np.arctan2(vy, vx))
            dpsi = psi_new - self.last_psi_ref
            # Wrap dpsi to (-pi, pi]
            dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
            self.last_psi_ref += dpsi

        # Progress log (every 500 cycles ≈ 5 s)
        if self._cycle_count % 500 == 0:
            pct = (elapsed / total * 100.0) if total > 0 else 0.0
            self.get_logger().info(
                f'TRAJECTORY WP[{idx}/{len(self.trajectory_waypoints)-1}] '
                f'X={wp[0]:.2f} Y={wp[1]:.2f} z_enu={wp[2]:.2f} '
                f'z_enu_real={self.states[8]:.2f} '
                f'alt_rel={self._alt_rel():.2f}m {pct:.1f}%')

        # Check completion
        if elapsed >= total:
            self.get_logger().info('Trajectory complete -> HOVER')
            self.state_voo        = self._HOVER
            self.trajectory_started = False

        return (wp[0], vx, 0.0, wp[1], vy, 0.0, wp[2], vz, 0.0, self.last_psi_ref)

    def _compute_reference(self, now):
        """Return reference tuple for the current FSM state, or None."""
        sv = self.state_voo

        if sv == self._WAIT_WP:
            return None

        if sv in (self._TAKEOFF, self._HOVER):
            if self.last_waypoint_goal is None:
                return None
            return self._goal_ref(*self.last_waypoint_goal)

        if sv == self._TRAJECTORY:
            return self._trajectory_ref(now)

        if sv == self._LANDING:
            if self.landing_mode == 0:
                return self._goal_ref(*self.ground_hold)
            return None   # Modo B: no setpoint during landing

        if sv == self._STANDBY_GROUND:
            return self._goal_ref(*self.ground_hold)

        return None

    # ── MPC step ─────────────────────────────────────────────────────────────
    def _run_mpc_step(self, ref):
        """Run one MPC computation.

        Parameters
        ----------
        ref : tuple
            (X_ref, Xd, Xdd, Y_ref, Yd, Ydd, Z_ref, Zd, Zdd, psi_ref)
            Z_ref/Zd/Zdd are in ENU (positive when airborne).

        Returns
        -------
        AttitudeTarget or None on error.
        """
        X_ref, Xd, Xdd, Y_ref, Yd, Ydd, Z_ref, Zd, Zdd, psi_ref = ref

        # Determine tilt limit: tighter during saturation recovery
        in_recovery = (self.saturation_recovery_enabled
                       and self._sat_recovery_remaining > 0)
        tilt_limit = (self.saturation_recovery_max_tilt_rad
                      if in_recovery else self.max_tilt_rad)

        # Internal convention is ENU (z up). pos_controller expects ENU too.
        # No z-axis sign conversion needed.
        states_enu = self.states

        try:
            phi_ref, theta_ref, U1_new = self.support.pos_controller(
                X_ref, Xd, Xdd,
                Y_ref, Yd, Ydd,
                Z_ref, Zd, Zdd,
                psi_ref, states_enu, max_vz_accel=self.max_vz_accel)
        except Exception as exc:
            self.get_logger().warn(f'pos_controller error: {exc}')
            return None

        U1_new = float(U1_new)

        if self.get_parameter('invert_attitude').value:
            phi_ref   = -phi_ref
            theta_ref = -theta_ref

        phi_ref   = float(np.clip(phi_ref,   -tilt_limit, tilt_limit))
        theta_ref = float(np.clip(theta_ref, -tilt_limit, tilt_limit))
        psi_ref   = float(psi_ref)

        ref_signals  = np.array([phi_ref, theta_ref, psi_ref])
        stacked_ref  = np.tile(ref_signals, self.hz)

        for _ in range(self.innerDyn_length):
            try:
                Ad, Bd, Cd, Dd, *_ = self.support.LPV_cont_discrete(
                    self.states, self.omega_total)
                Hdb, Fdbt, _, _ = self.support.mpc_simplification(
                    Ad, Bd, Cd, Dd, self.hz)
            except Exception as exc:
                self.get_logger().warn(f'LPV/MPC error: {exc}')
                return None

            x_aug_t  = np.array([
                self.states[9],  self.states[3],
                self.states[10], self.states[4],
                self.states[11], self.states[5],
                self.U2, self.U3, self.U4])
            extended = np.concatenate((x_aug_t, stacked_ref))
            ft       = extended @ Fdbt
            du       = -np.linalg.inv(Hdb) @ ft.reshape(-1, 1)

            self.U2 = float(np.clip(self.U2 + du[0, 0], -1.0, 1.0))
            self.U3 = float(np.clip(self.U3 + du[1, 0], -1.0, 1.0))
            self.U4 = float(np.clip(self.U4 + du[2, 0], -1.0, 1.0))

        # Convert torques to desired angular-rate commands
        Ix = self.support.constants['Ix']
        Iy = self.support.constants['Iy']
        Iz = self.support.constants['Iz']
        dt = self.Ts

        p_des = float(self.states[3] + (self.U2 / Ix) * dt)
        q_des = float(self.states[4] + (self.U3 / Iy) * dt)
        r_des = float(self.states[5] + (self.U4 / Iz) * dt)

        att = AttitudeTarget()
        att.type_mask        = AttitudeTarget.IGNORE_ATTITUDE
        att.thrust           = float(np.clip(
            U1_new / self.support.constants['max_thrust'], 0.0, 1.0))
        att.body_rate.x      = p_des
        att.body_rate.y      = q_des
        att.body_rate.z      = r_des
        att.orientation.w    = 1.0
        att.orientation.x    = 0.0
        att.orientation.y    = 0.0
        att.orientation.z    = 0.0

        if att.thrust >= self.thrust_warn_threshold:
            self.get_logger().warn(
                f'THRUST SATURATION: thrust={att.thrust:.3f} U1={U1_new:.1f}N '
                f'z_enu={self.states[8]:.2f} alt_rel={self._alt_rel():.2f}m '
                f'Z_ref={Z_ref:.2f} phi_ref={phi_ref:.3f}rad '
                f'theta_ref={theta_ref:.3f}rad',
                throttle_duration_sec=2.0)

        # ── Saturation recovery state machine ─────────────────────────────────
        # Counts consecutive MPC steps (≈0.1 s each at 10 Hz) where thrust
        # exceeds saturation_thrust_threshold, then enters a recovery phase.
        if self.saturation_recovery_enabled:
            if att.thrust >= self.saturation_thrust_threshold:
                self._sat_counter += 1
            else:
                self._sat_counter = 0

            if self._sat_recovery_remaining > 0:
                self._sat_recovery_remaining -= 1
                if self._sat_recovery_remaining == 0:
                    self.get_logger().warn(
                        'SATURATION RECOVERY: exiting recovery; '
                        'normal tilt/XY limits restored')
            elif self._sat_counter >= self.saturation_recovery_cycles:
                self._sat_recovery_remaining = self.saturation_recovery_duration
                self._sat_counter = 0
                self.get_logger().warn(
                    f'SATURATION RECOVERY: entering recovery for '
                    f'{self.saturation_recovery_duration} MPC steps '
                    f'(tilt_limit={self.saturation_recovery_max_tilt_rad:.2f}rad '
                    f'xy_speed={self.saturation_recovery_max_ref_speed_xy:.2f}m/s'
                    f'{" freeze_xy=true" if self.saturation_recovery_freeze_xy else ""})')

        return att

    def _publish_attitude(self, att, now):
        """Stamp and publish an AttitudeTarget message."""
        att.header.stamp = now.to_msg()
        self.att_pub.publish(att)

    def _run_and_publish(self, ref, now):
        """Run MPC (decimated) and publish, caching result for interim cycles."""
        if ref is None:
            return

        if self._cycle_count % self._mpc_decimation == 0:
            att = self._run_mpc_step(ref)
            if att is not None:
                self._hold_cmd = att

        if self._hold_cmd is not None:
            self._publish_attitude(self._hold_cmd, now)

    # ── FSM per-state handlers ───────────────────────────────────────────────
    def _fsm_step(self, now):
        """Dispatch to the appropriate per-state handler."""
        sv = self.state_voo
        if sv == self._WAIT_WP:
            self._state_wait_wp()
        elif sv == self._TAKEOFF:
            self._state_takeoff(now)
        elif sv == self._HOVER:
            self._state_hover(now)
        elif sv == self._TRAJECTORY:
            self._state_trajectory(now)
        elif sv == self._LANDING:
            self._state_landing(now)
        elif sv == self._STANDBY_GROUND:
            self._state_standby_ground(now)

    def _state_wait_wp(self):
        # Auto-handoff: when the drone is already armed+OFFBOARD (e.g. handed
        # off from hover_supervisor_node), latch current position and enter
        # HOVER so the MPC publishes attitude setpoints immediately.
        # This prevents a gap in setpoints (which would drop OFFBOARD mode)
        # and triggers hover_supervisor's auto_disable_when_mpc_active.
        if (self.auto_arm_handoff
                and self.odom_received
                and self.mavros_state.armed
                and self.mavros_state.mode == 'OFFBOARD'):
            # states[6]=x, states[7]=y, states[8]=z (ENU)
            cur_x, cur_y, cur_z = (
                float(self.states[6]),
                float(self.states[7]),
                float(self.states[8]),
            )
            self.last_waypoint_goal = [cur_x, cur_y, cur_z]
            self.offboard_activated   = True
            self.activation_confirmed = True
            self._warmup_counter      = self._WARMUP_CYCLES
            self.state_voo            = self._HOVER
            self.get_logger().info(
                f'WAIT_WP: armed+OFFBOARD detected – latching current position '
                f'({cur_x:.3f},{cur_y:.3f},{cur_z:.3f}) and entering HOVER '
                f'(auto_arm_handoff from hover_supervisor_node)')
            return

        if self._cycle_count % 1000 == 0:
            self.get_logger().info(
                'WAIT_WP: waiting for waypoint command',
                throttle_duration_sec=10.0)

    def _state_takeoff(self, now):
        """State 1: warmup -> OFFBOARD+ARM -> climb -> HOVER."""

        # ── Phase A: warmup ──────────────────────────────────────────────────
        if self._warmup_counter < self._WARMUP_CYCLES:
            self._warmup_counter += 1
            if self._warmup_counter == 1:
                self.get_logger().info('TAKEOFF: warmup streaming started')
            # Stream a safe setpoint at current position
            ref = self._goal_ref(
                self.states[6], self.states[7], self.states[8])
            self._run_and_publish(ref, now)
            return

        # ── Phase B: request OFFBOARD + ARM ─────────────────────────────────
        if not self.offboard_activated:
            self.get_logger().info('TAKEOFF: requesting OFFBOARD+ARM')
            self._request_offboard()
            self._request_arm()
            self.offboard_activated = True
            self.activation_time    = now
            self.takeoff_counter    = 0
            return

        # ── Phase C: wait for FCU confirmation ───────────────────────────────
        if not self.activation_confirmed:
            if (self.mavros_state.armed
                    and self.mavros_state.mode == 'OFFBOARD'):
                self.activation_confirmed = True
                self.takeoff_counter      = 0
                self.get_logger().info(
                    'OFFBOARD+ARM confirmed! Starting climb...')
            elif (now - self.activation_time).nanoseconds / 1e9 > self.activation_timeout:
                self.get_logger().warn(
                    f'OFFBOARD+ARM timeout ({self.activation_timeout:.0f}s)! '
                    f'armed={self.mavros_state.armed} '
                    f'mode={self.mavros_state.mode}  – retrying')
                self.offboard_activated   = False
                self.activation_confirmed = False
                self.takeoff_counter      = 0
                return
            else:
                self.get_logger().info(
                    f'Waiting OFFBOARD+ARM... '
                    f'armed={self.mavros_state.armed} '
                    f'mode={self.mavros_state.mode}',
                    throttle_duration_sec=2.0)
                # Keep streaming during wait
                ref = self._goal_ref(
                    self.states[6], self.states[7], self.states[8])
                self._run_and_publish(ref, now)
                return

        # ── Phase D: publish takeoff reference, check arrival ────────────────
        goal = self.last_waypoint_goal
        if goal is None:
            return

        ref = self._goal_ref(goal[0], goal[1], goal[2])
        self._run_and_publish(ref, now)
        self.takeoff_counter += 1

        if self.takeoff_counter == 1:
            self.get_logger().info(
                f'Climbing to alt={self._enu_to_alt(goal[2]):.1f}m '
                f'(z_enu={goal[2]:.2f})  '
                f'({goal[0]:.2f}, {goal[1]:.2f})')

        alt_rel    = self._alt_rel()
        goal_alt_rel = self._alt_rel(goal[2])

        # Condition A: within hover_alt_margin of goal
        cond_a = alt_rel >= goal_alt_rel - self.hover_alt_margin
        # Condition B: minimum safe altitude reached
        cond_b = alt_rel >= self.min_takeoff_alt_rel

        # Throttled debug log every 2 s (200 cycles at 100 Hz); 1 s otherwise
        if self.takeoff_counter % 100 == 0:
            base_msg = (
                f'Takeoff: alt_target={self._enu_to_alt(goal[2]):.1f}m  '
                f'alt_rel={alt_rel:.2f}m  '
                f'z_enu={self.states[8]:.2f}m  '
                f't={self.takeoff_counter / 100:.1f}s')
            if self.takeoff_counter % 200 == 0:
                base_msg += (
                    f'  goal_alt_rel={goal_alt_rel:.2f}m'
                    f'  _z0_map={self._z0_map}'
                    f'  condA={cond_a} condB={cond_b}')
            self.get_logger().info(base_msg)

        # Arrival check: Condition C – A and B must both hold continuously
        # for takeoff_complete_hold_time seconds before switching to HOVER.
        now_sec = now.nanoseconds / 1e9
        if cond_a and cond_b:
            if self._takeoff_cond_start is None:
                self._takeoff_cond_start = now_sec
            held = now_sec - self._takeoff_cond_start
            if held >= self.takeoff_complete_hold_time:
                self.get_logger().info(
                    f'Takeoff complete! '
                    f'alt_rel={alt_rel:.2f}m '
                    f'(held {held:.2f}s) -> HOVER')
                self.takeoff_counter     = 0
                self._takeoff_cond_start = None
                self.state_voo           = self._HOVER
                # Immediately activate pending trajectory
                if self.trajectory_waypoints:
                    self.get_logger().info(
                        'Pending trajectory found -> TRAJECTORY')
                    self.state_voo = self._TRAJECTORY
        else:
            # Reset hold timer if conditions are not continuously met
            if self._takeoff_cond_start is not None:
                self.get_logger().info(
                    f'Takeoff hold reset: alt_rel={alt_rel:.2f}m '
                    f'condA={cond_a} condB={cond_b}')
            self._takeoff_cond_start = None

    def _state_hover(self, now):
        """State 2: hold goal position, wait for trajectory."""
        if self.last_waypoint_goal is None:
            return

        ref = self._goal_ref(*self.last_waypoint_goal)
        self._run_and_publish(ref, now)

        if self._cycle_count % 500 == 0:
            gx, gy, gz = self.last_waypoint_goal
            alt_rel = self._alt_rel()
            self.get_logger().info(
                f'HOVER goal=({gx:.2f},{gy:.2f}) '
                f'alt_target={self._enu_to_alt(gz):.2f}m z_enu_ref={gz:.2f} '
                f'z_enu={self.states[8]:.2f} alt_rel={alt_rel:.2f}m',
                throttle_duration_sec=10.0)

        # Activate trajectory when one arrives
        if self.trajectory_waypoints and not self.trajectory_started:
            self.get_logger().info('HOVER -> TRAJECTORY')
            self.state_voo = self._TRAJECTORY

    def _state_trajectory(self, now):
        """State 3: discrete-waypoint trajectory execution."""
        # Landing detection (relative altitude: near 0 when on ground)
        alt_rel = self._alt_rel()
        if alt_rel < self.land_z_threshold:
            self.get_logger().warn(
                f'Landing detected during trajectory! '
                f'z_enu={self.states[8]:.2f}m alt_rel={alt_rel:.2f}m')
            if self.last_waypoint_goal is not None:
                self.ground_hold = [
                    self.last_waypoint_goal[0],
                    self.last_waypoint_goal[1],
                    max(0.01, self.land_z_threshold)]
            else:
                self.ground_hold = [
                    self.states[6], self.states[7],
                    max(0.01, self.land_z_threshold)]
            self._initiate_landing()
            return

        ref = self._trajectory_ref(now)
        self._run_and_publish(ref, now)

    def _state_landing(self, now):
        """State 4: wait for landing timeout, then branch on landing_mode."""
        if self._cycle_count % 500 == 0:
            self.get_logger().info(
                'LANDING: waiting for touchdown',
                throttle_duration_sec=10.0)

        if self.landing_active:
            if not self.landing_start_time_set:
                self.landing_start_time     = now
                self.landing_start_time_set = True
                self.get_logger().info(
                    f'Landing timer started ({self.landing_timeout:.0f}s)')

            elapsed = (now - self.landing_start_time).nanoseconds / 1e9

            if elapsed > self.landing_timeout:
                if self.landing_mode == 0:
                    # ── Modo A: standby on ground ────────────────────────────
                    self.landing_active         = False
                    self.landing_start_time_set = False
                    self.takeoff_counter        = 0
                    self.trajectory_waypoints.clear()
                    self.trajectory_started     = False
                    self.current_waypoint_idx   = 0
                    # ENU: positive z keeps pos_controller at a few cm altitude
                    self.ground_hold[2]         = max(0.01, self.land_z_threshold)
                    self._state5_recovery_time  = now
                    self.state_voo              = self._STANDBY_GROUND
                    self.get_logger().warn(
                        'LANDING -> STANDBY_GROUND (Modo A: keeping OFFBOARD+ARM)')
                else:
                    # ── Modo B: disarm, reset to WAIT_WP ────────────────────
                    self._request_disarm()
                    self.landing_active         = False
                    self.landing_start_time_set = False
                    self.offboard_activated     = False
                    self.activation_confirmed   = False
                    self.takeoff_counter        = 0
                    self.trajectory_waypoints.clear()
                    self.trajectory_started     = False
                    self.current_waypoint_idx   = 0
                    self.last_waypoint_goal     = None
                    self.state_voo              = self._WAIT_WP
                    self.get_logger().warn(
                        'LANDING -> WAIT_WP (Modo B: DISARM requested)')
                return

        # Modo A: publish ground-hold setpoint to keep OFFBOARD alive
        if self.landing_mode == 0:
            ref = self._goal_ref(*self.ground_hold)
            self._run_and_publish(ref, now)

    def _state_standby_ground(self, now):
        """State 5 (Modo A): hold on ground OFFBOARD+ARM, wait for flight goal."""
        # Check OFFBOARD+ARM; attempt recovery if lost
        if (not self.mavros_state.armed
                or self.mavros_state.mode != 'OFFBOARD'):
            elapsed = (now - self._state5_recovery_time).nanoseconds / 1e9
            if elapsed > self.activation_timeout:
                self.get_logger().warn(
                    f'State 5: not OFFBOARD+ARM – retrying '
                    f'(armed={self.mavros_state.armed} '
                    f'mode={self.mavros_state.mode})')
                self._request_offboard()
                self._request_arm()
                self._state5_recovery_time = now
            return

        ref = self._goal_ref(*self.ground_hold)
        self._run_and_publish(ref, now)

        if self._cycle_count % 500 == 0:
            gx, gy, gz = self.ground_hold
            self.get_logger().info(
                f'STANDBY_GROUND hold=({gx:.2f},{gy:.2f}) '
                f'z_enu={gz:.3f} alt={self._enu_to_alt(gz):.3f}m',
                throttle_duration_sec=10.0)

    # ── Main control loop (100 Hz) ───────────────────────────────────────────
    def control_loop(self):
        now = self.get_clock().now()
        self._cycle_count += 1

        # Gate 1: controller disabled
        if not self.enabled:
            self.get_logger().info(
                'Controller disabled (enabled=false)',
                throttle_duration_sec=5.0)
            return

        # Gate 2: external override – freeze FSM but keep setpoint stream alive
        if self.override_active:
            if self._hold_cmd is not None:
                self._publish_attitude(self._hold_cmd, now)
            self.get_logger().info(
                'Override active (override_active=true): FSM frozen',
                throttle_duration_sec=5.0)
            return

        # Gate 3: odometry not yet received
        if not self.odom_received:
            return

        # Run FSM
        self._fsm_step(now)

        # Periodic diagnostic log
        if self._cycle_count % 1000 == 0:
            state_names = ['WAIT_WP', 'TAKEOFF', 'HOVER',
                           'TRAJECTORY', 'LANDING', 'STANDBY_GROUND']
            sname = state_names[self.state_voo] if 0 <= self.state_voo <= 5 else '?'
            ref_pos = self.last_waypoint_goal or [0.0, 0.0, 0.0]
            self.get_logger().info(
                f'FSM={sname}({self.state_voo}) '
                f'pos=({self.states[6]:.2f},{self.states[7]:.2f}) '
                f'z_enu={self.states[8]:.2f} alt_rel={self._alt_rel():.2f}m '
                f'goal=({ref_pos[0]:.2f},{ref_pos[1]:.2f}) '
                f'z_enu_ref={ref_pos[2]:.2f} '
                f'armed={self.mavros_state.armed} '
                f'mode={self.mavros_state.mode}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LPVMPC_Drone()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

##THIS SCRIPT IS TO SIMULATE A 3D QUADCOPTER USING MATPLOTLIB, BASED ON PD CONTROL SYSTEM
##NOTE: QUIT THE SIMULATION BY PRESSING 'q'


## IMPORTING REQUIRED MODULES
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

from parameters import *
from controller import *


## DO YOU WANT ANIMATION?
animate = 0 # True | False

## FETCHING AND DECLARING VARIABLES FROM PARAMETERS MODULE
gravity=param.gravity
mass=param.mass
Ixx=param.Ixx
Iyy=param.Iyy


## INITIAL AND CONDITTIONS
initialStates=[0,0,0,0,0,0,0,0,0,0] # [x,y,z,theta,phi,xVel,yVel,zVel,thetaVel,phiVel]
desiredStates=[1,1,1,0,0,0,0,0,0]


## DECLARING TIME LIMIT AND NUMBER OF TIME POINTS
finalTime=10
timeSteps=30*finalTime+1

## TIME POINTS ARRAY
time=np.linspace(0,finalTime,timeSteps)


## FUNCTION THAT RETURNS dx/dt
def model(z,t,u1,u2,u3):
    """
    Purpose:
    ---
    Takes a list of initial conditions with some timpoints and some arguments which are the input required to the quadcopter.
    Using them in relation to the model of a quadcopter, the simplified differential equations are generated.

Q    Input Arguments:
    ---
    z, t, u1, u2, u3 :   [ list ], [ list ], [ int ], [ int ], [ int ]
            z : initial condition
            t : time points
            u1 : force input for movement in z direction
            u2 : moment input for movement in x direction
            u3 : moment input for movement in y direction
    
    Returns:
    ---
    `dxdt` :  [ list ]
            differential of all the initial states
    """
    
    x1=z[0]     # Intial Condition for First FODE 
    x2=z[1]     # Initial condition for Second FODE
    x3=z[2]     # Initial condition for Third FODE, and so on
    x4=z[3]     
    x5=z[4]     
    x6=z[5]     
    x7=z[6]
    x8=z[7]
    x9=z[8]
    x10=z[9]
    
    dx1dt=x6                # First FODE
    dx2dt=x7                # Second FODE 
    dx3dt=x8                # Third FODE, and so on
    dx4dt=x9
    dx5dt=x10
    dx6dt=-gravity*x4       
    dx7dt=-gravity*x5
    dx8dt=-gravity+u1/mass  
    dx9dt=u2/Iyy
    dx10dt=u3/Ixx            
    
    dxdt = [dx1dt,dx2dt,dx3dt,dx4dt,dx5dt,dx6dt,dx7dt,dx8dt,dx9dt,dx10dt]
    return dxdt


## LISTS TO STORE SOLUTION
x1=np.zeros(timeSteps)
x2=np.zeros(timeSteps)
x3=np.zeros(timeSteps)
x4=np.zeros(timeSteps)
x5=np.zeros(timeSteps)
x6=np.zeros(timeSteps)
x7=np.zeros(timeSteps)
x8=np.zeros(timeSteps)
x9=np.zeros(timeSteps)
x10=np.zeros(timeSteps)

u1=np.zeros(timeSteps)
u2=np.zeros(timeSteps)
u3=np.zeros(timeSteps)

thetaC=np.zeros(timeSteps)
phiC=np.zeros(timeSteps)

## DECLARING THE LIMIT OF THE MOMENT AS PER COPTER'S ABILITY
uMax=0.1
uMin=-0.1

## RECORDING THE INITIAL CONDITIONS
x1[0]=initialStates[0]   # x1 or x
x2[0]=initialStates[1]   # x2 or y
x3[0]=initialStates[2]   # x3 or z
x4[0]=initialStates[3]   # x4 or theta
x5[0]=initialStates[4]   # x5 or phi
x6[0]=initialStates[5]   # x6 or xdot
x7[0]=initialStates[6]   # x1 or ydot
x8[0]=initialStates[7]   # x1 or zdot
x9[0]=initialStates[8]   # x1 or thetadot
x10[0]=initialStates[9]  # x1 or phidot


## SETTING THE PLOT FOR MATPLOTLIB
fig1 = plt.figure("2D graphs",figsize=(6,6))
fig2 = plt.figure("3D Graph", figsize=(6,6))

plt.ion()
plt.show()

## SOLVING ODE
for i in range(1,timeSteps):
    """
    Purpose:
    ---
    This loop helps in simulating the copter wrt time.
    Steps followed:
                    1. Get the inputs from the controller by passing initial and desired states to it.
                    2. Pass them to odeint and to the model along with initial states and time points.
                    3. Store the new returned conditions as initial conditions and plot the required variables(for eg.,'x','y','z','thetaC','phiC','time',etc).

    """
    
    ## SPAN FOR NEXT TIME STEP
    timeSpan=[time[i-1],time[i]]

    ## STEP-1 EXECUTION #####################################################################################################################################################
    
    u=controller.quadControls(initialStates, desiredStates)
    
    u1[i]=round(u[0],2)
    u2[i]=round(u[1],3)
    u3[i]=round(u[2],3)
    thetaC[i]=round(u[3],3)
    phiC[i]=round(u[4],3)
    
    # SETTING LIMIT TO MOMENT INPUTS FOR X AND Y AXIS
    if u2[i]>uMax:
        u2[i]=uMax
    elif u2[i]<uMin:
        u2[i]=uMin
    if u3[i]>uMax:
        u3[i]=uMax
    elif u3[i]<uMin:
        u3[i]=uMin
        
    ## STEP-2 EXECUTION #####################################################################################################################################################
        
    z=odeint(model,initialStates,timeSpan,args=(u1[i],u2[i],u3[i]))
    #print("Solution ", z)
    
    if(z[1][2]<0):
        z[1][2]=0
        z[1][7]=0

    ## STEP-3 EXECUTION #####################################################################################################################################################

    # STORING THE SOLUTION FOR PLOTTING
    x1[i]=z[1][0]
    x2[i]=z[1][1]
    x3[i]=z[1][2]
    x4[i]=z[1][3]
    x5[i]=z[1][4]
    x6[i]=z[1][5]
    x7[i]=z[1][6]
    x8[i]=z[1][7]
    x9[i]=z[1][8]
    x10[i]=z[1][9]

    # NEXT INITIAL CONDITIONS
    initialStates=z[1]
    
    fig1.clf()
    fig2.clf()

    # PLOTTING X,Y,Z POSITION SEPERATELY WRT TIME
    ax1=fig1.add_subplot(211,label = 'post')
    ax1.plot(time[0:i],x1[0:i],'g--',label='x')
    ax1.plot(time[0:i],x2[0:i],'r:',label='y')
    ax1.plot(time[0:i],x3[0:i],'b:',label='z')
    ax1.set_ylabel('position')
    ax1.set_xlabel('time')
    ax1.legend(loc='best')

    # PLOTTING ALL THE INPUTS(u1,u2,u3,phiC,thetaC) WRT TIME
    ax2=fig1.add_subplot(212, label = 'input')
    ax2.plot(time[0:i],u1[0:i],'k:',label='u1(t)')
    ax2.plot(time[0:i],u2[0:i],'g:',label='u2(t)')
    ax2.plot(time[0:i],u3[0:i],'m:',label='u3(t)')
    ax2.plot(time[0:i],phiC[0:i],'b-.',label='phiC(t)')
    ax2.plot(time[0:i],thetaC[0:i],'c-.',label='thetaC(t)')
    ax2.set_ylabel('input')
    ax2.set_xlabel('time')
    ax2.legend(loc='best')
    

    # PLOTTING 3D TRAJECTORY OF QUADCOPTER
    axes = fig2.add_subplot(1,1,1,projection='3d')
    if animate:
        axes.set_xlim3d(initialStates[1]-2,desiredStates[1]+2)
        axes.set_ylim3d(initialStates[0]-2,desiredStates[0]+2)
        axes.set_zlim3d(initialStates[2]-2,desiredStates[2]+2)
        axes.text(x2[i],x1[i],x3[i], "-O-", size = 18, color = 'red')
        axes.text(desiredStates[1],desiredStates[0],desiredStates[2],  'stop', size=11, zorder=4, color='b')
        axes.plot(0,time[0],0, label = 'quadcopter', color = 'red')
    else:    
        axes.plot(x2[0:i],x1[0:i], x3[0:i],'o-', label = 'quadcopter', color = 'red')
    axes.set_xlabel('y')
    axes.set_ylabel('x')
    axes.set_zlabel('z')
    axes.legend(loc='best')
    
    # ADDING ANNOTATIONS AT START AND STOP POINT
    axes.text(x2[0],x1[0],x3[0],  'start', size=11, zorder=1, color='b')
    diff = np.array([x2[i],x1[i],x3[i]]) - np.array([desiredStates[1],desiredStates[0],desiredStates[2]])
    if ((sum(diff)/3) < 0.00015) and ((sum(diff)/3) > -0.00015) :
        axes.text(desiredStates[1],desiredStates[0],desiredStates[2],  'stop', size=11, zorder=4, color='b')

       
    plt.pause(0.1)

## THIS SCRIPT IS A PD CONTROL PROGRAM FOR PURPOSE OF SIMULATING A 3D QUADCOPTER, P.S. CAN BE USED IN OTHER APPLICATIONS


from parameters import *



class controller:
        
    def quadControls(z, zd):
        """
        Purpose:
        ---
        Takes two list of initial conditions and desired conditions.
        Based on that it calculates the input the copter must have to reach the desired state.
        The values are manipulated by PD control which has predefined kP and kD values.
        These values must be adjusted properly in order to tune our controller and have a fast and smooth simulation

        Input Arguments:
        ---
        z, zd :   [ list ], [ list ]
                z  : current states: 1x10 current state [x,y,z,theta,phi,xVel,yVel,zVel,thetaVel,phiVel]
                zd : desired states: 1x8  desired state [xPosDes,yPosDes,zPosDes,xVelDes,yVelDes,zVelDes,xAccDes,yAccDes,zAccDes]
                
        
        Returns:
        ---
        force, moment1, moment2, thetaC, phiC :  [ int ]*5
                input value for quadcopter, thi=ese will be served as arguments to the model function
        """

        ## CURRENT STATE
        xPos=z[0]
        yPos=z[1]
        zPos=z[2]
        theta=z[3]
        phi=z[4]
        xVel=z[5]
        yVel=z[6]
        zVel=z[7]
        thetaVel=z[8]
        phiVel=z[9]

        ## DESIRED STATE
        xPosDes=zd[0]
        yPosDes=zd[1]
        zPosDes=zd[2]
        xVelDes=zd[3]
        yVelDes=zd[4]
        zVelDes=zd[5]
        xAccDes=zd[6]
        yAccDes=zd[7]
        zAccDes=zd[8]

        ## DECLARING KP AND KD VALUES FOR X,Y,Z AND THETA,PHI
        #  KP DEPENDS ON THE MAGNITUDE OF ERROR LEFT i.e. COEFF OF PROPOTIONALITY
        #  KD DEPENDS ON THE RATE OF CHANGE OF ERROR
        kpX=40;            
        kdX=9;
        
        kpY=40;            
        kdY=9;            

        kpZ=65.0;           
        kdZ=10.9;           

        kpTheta=25;           
        kdTheta=11;
        
        kpPhi=25;           
        kdPhi=11;            

        thetaC=-(xAccDes+kpX*(xPosDes-xPos)+kdX*(xVelDes-xVel))/param.gravity
        thetaCVel=-(kpX*(xVelDes-xVel)+kdX*(xAccDes+param.gravity*theta))/param.gravity
        
        phiC=-(yAccDes+kpY*(yPosDes-yPos)+kdY*(yVelDes-yVel))/param.gravity
        phiCVel=-(kpY*(yVelDes-yVel)+kdY*(yAccDes+param.gravity*phi))/param.gravity

        ## CALCULATING INPUTS TO RETURN TO FUNCTION
        force=param.mass*(param.gravity+zAccDes+kpZ*(zPosDes-zPos)+kdZ*(zVelDes-zVel))
        moment1=param.Iyy*(kpTheta*(thetaC-theta)+kdTheta*(thetaCVel-thetaVel))
        moment2=param.Ixx*(kpPhi*(phiC-phi)+kdPhi*(phiCVel-phiVel))


        return force, moment1, moment2, thetaC, phiC

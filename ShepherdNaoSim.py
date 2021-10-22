#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import sys
#Appending the system path to the NAOqi site-packages location on my system
sys.path.append('/home/zachary/Downloads/pyNaoqi/lib/python2.7/site-packages')
import qi
import argparse

import almath
import math
import time

import numpy as np
#import tensorflow as tf
import datetime
import pandas as pd
import csv
import math
import matplotlib.pyplot as plt
from Environment_Strombom import Environment
global coordMatrixClock 


def main(session):
    
    # Get the services ALMotion & ALRobotPosture.
    useSensorValues = False
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    #motion_service.moveTo(5,-5,0) Use this to move the robot to a predefined position before starting a sim 

    #Getting initial robot position
    initRobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
    initRobotPositionMatrix = np.matrix([[initRobotPosition.x],[initRobotPosition.y]])
    #Convert initial robot position to global coordinates
    coordMatrixCounterClock = np.matrix([[np.cos(math.pi/2), -np.sin(math.pi/2)], [np.sin(math.pi/2), np.cos(math.pi/2)]])
    NaoGlobal = np.matmul(coordMatrixCounterClock,initRobotPositionMatrix)
    NaoGlobalx =NaoGlobal[0,0]
    NaoGlobaly =NaoGlobal[1,0]
    NaoGlobal = np.matrix([NaoGlobalx,NaoGlobaly])
    #motion_service.stopMove() uncomment to make NAO stop previous motion before beginning a new run
    global coordMatrixClock 
    coordMatrixClock = np.matrix([[np.cos(math.pi/2), np.sin(math.pi/2)], [-np.sin(math.pi/2), np.cos(math.pi/2)]])

    """
    This code is required for controlling the sheep and shepherd in Hungs Strombom algorithm
    """
    NumberOfSheep = 3 #no. agents in the shepherding sim
    N_EPISODE = 27 #how many trials you want to run
    max_steps = 2000 #steps desired per trial
    
    NeighbourhoodSize = int(NumberOfSheep/2)
    NumberOfShepherds = 1 #leave this at one unless integrating with more robot shepherds
    case = 1 #1: Driving ; 0: Collecting
    #Creating the new environment
    env = Environment(NumberOfSheep,NeighbourhoodSize,NumberOfShepherds,case)
    #done,s_t,case = env.reset(NaoGlobal)
    """
    Ending the code required to initalise shepherding sim
    """

    #Initalize the arms for better walk stability   
    leftArmEnable  = True
    rightArmEnable = True
    motion_service.setMoveArmsEnabled(leftArmEnable, rightArmEnable)

    # Go to rest position
    #motion_service.rest()
    # Wake up robot
    #motion_service.wakeUp()

    # Send robot to Stand Init
    posture_service.goToPosture("StandInit", 0.5)

    # Initialize the move
    motion_service.moveInit()

    #Clock setup to track the total time to run a simulation trial
    t_start = time.time()#Start the clock
    t_end = float('inf')#float('inf')##We want to do do this forever so set = inf
    t = 0 ##set current time to 0 before loop begins

    # Create some text files to write data (Uncomment to save data)
    #NaoPos = open('NaoPosition.txt', 'w')
    #ShepherdPos = open('ShepherdPos.txt', 'w')
    #TimeFile = open('Time.txt', 'w')
    #SpeedFile = open('Speed.txt', 'w')
    
    #Main Control Loop (CHANGE ROTATION OF COORDINATE SYSTEM BASED ON NAO LOCAL HEADING IN VICON)
 
    #Create the header for the CSV file to save data to
    now = datetime.datetime.now()
    header = ["Ep","Step","ShepherdX","ShepherdY","ShepherdXvel","ShepherdYvel","Sheep1x","Sheep1y","Sheep2x","Sheep2y","Sheep3x","Sheep3y","SheepGCMx","SheepGCMy","Furthestx","FurthestY","SubGoalX","SubGoalY","time"]
    filename = "NAOShepherding-25_2.csv"
    with open(filename, 'w') as f:
        pd.DataFrame(columns = header).to_csv(f,encoding='utf-8', index=False, header = True)
    t=time.time()-t_start

    """
    This code is required for controlling the sheep and shepherd in Hungs Strombom algorithm
    """
    for episode_i in range(N_EPISODE):
        startpoint = np.matrix([2,5])
        done,s_t,case = env.reset(startpoint)
        reset = 0
        while reset == 0:
            RobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
            real_action=0
            setpoint = 1
            NaoGlobal, DtoPoint = NaoMove(2,5,motion_service,RobotPosition,real_action,setpoint)
            RobotPositionMatrix = np.matrix([[RobotPosition.x],[RobotPosition.y]])
            CurrentNaoPos = np.matmul(coordMatrixCounterClock,RobotPositionMatrix)
            if DtoPoint<0.01:
                reset =1
                t=time.time()-t_start
            """
            Code required for strombom sim concludes
            """

        # number of timesteps
        for step in range(max_steps):
            reset = 0
            RobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
            RobotPositionMatrix = np.matrix([[RobotPosition.x],[RobotPosition.y]])
            CurrentNaoPos = np.matmul(coordMatrixCounterClock,RobotPositionMatrix)
            a_t,subgoal,driving = env.Strombom_action(case)
            ##Getting Next Shepherd Position and Velocity
            real_action, ShepherdMatrix = env.stepRealAction(a_t,CurrentNaoPos)


            #Define Desired Global Position (x right y up)
            posx =ShepherdMatrix[0,0] #r*np.cos(w*t)Current desired x global position (m)
            posy =ShepherdMatrix[0,1]#r*np.sin(2*w*t)/2 #Current desired y globalposition (m)

            #Transform these Global Coordinates (x right y up) into GlobalNAO (x up y left) by rotating around z axis clockwise relative to NAO global thy
            coordMatrixClock = np.matrix([[np.cos(math.pi/2), np.sin(math.pi/2)], [-np.sin(math.pi/2), np.cos(math.pi/2)]])
            PosGlobal= np.matrix([[posx],[posy]])
            PosNaoGlobal = np.matmul(coordMatrixClock,PosGlobal)

            NaoGlobal = NaoMove(posx,posy,motion_service,RobotPosition,real_action,reset)

            #print motion_service.getMoveConfig("Max")
            #time.sleep(Sdes)
            time.sleep(0.05)
            RobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
            RobotPositionMatrix = np.matrix([[RobotPosition.x],[RobotPosition.y]])
            CurrentNaoPos1 = np.matmul(coordMatrixCounterClock,RobotPositionMatrix)
            s_t1, done,ShepherdMatrix,SheepMatrix,SheepFurthest, case, infor = env.step(a_t,CurrentNaoPos1)
            

            t=time.time()-t_start

            #Save the data to a csv file
            save_data = np.hstack([episode_i+1,step+1,posx,posy,real_action[0,0],real_action[0,1],infor[0],infor[1],infor[2],infor[3],infor[4],infor[5],infor[6],infor[7],infor[8],infor[9],infor[10],infor[11],done,t]).reshape(1,20)
            with open(filename, 'a') as f:
                pd.DataFrame(save_data).to_csv(f,encoding='utf-8', index=False, header = False)

            #View the simulation figure in real-time
            env.view(NaoGlobal,subgoal,driving,CurrentNaoPos)

            #If the COM is within 0.2x0.2m from target, end the sim episode/trial
            if (infor[6]<0.2 and infor[7]<0.2):
                break
              
##NAO Controller
def NaoMove(posx,posy,motion_service,RobotPosition,real_action,setpoint):
    #Walk Parameters
    f =  0.01#frequency of walk
    r = 0.75 #radius of turn
    w = 2*math.pi*f
    Kp = 0.30 #Position Gain
    Kthy = 0.50 #Turn Rate Gain
    useSensorValues = True                

    #Define NAOs current position Global NAO (x up y left) Simulation Purposes
    
    Nao = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
    RobotGlobalThy = math.atan2(RobotPosition.y,RobotPosition.x) #NAO Thy Relative to his Global Axis
    NaoGlobalPos= np.matrix([[RobotPosition.x],[RobotPosition.y]]) #NAO Position Relative to his Global Axis
    
    
    #Transform the GlobalNAO Position Axis (x up y left) into GLOBAL by making Axis (x right y up), rotaion around z axis counter-clockwise relative to NAO global thy
    coordMatrixCounterClock = np.matrix([[np.cos(math.pi/2), -np.sin(math.pi/2)], [np.sin(math.pi/2), np.cos(math.pi/2)]])
    NaoGlobal = np.matmul(coordMatrixCounterClock,NaoGlobalPos)
    NaoGlobalx =NaoGlobal[0,0]
    NaoGlobaly =NaoGlobal[1,0]
    
    #Setting Global Velocity
    if setpoint == 0:
        xGdesDeriv = real_action[0,0]#-r*w*np.sin(w*t)#a_t[0,0]#-r*w*np.sin(w*t) #Derivative of desired x global (m/s)
        yGdesDeriv = real_action[0,1]#r*w*np.cos(w*t)#a_t[0,1]#r*w*np.cos(w*t)#+2*r*w*np.cos(2*w*t)/2 #Derivative of desired y global (m/s)
    else:
        xGdesDeriv = 0
        yGdesDeriv = 0
    #print "Velocity Main= ", xGdesDeriv, yGdesDeriv

    #Get NAOs current Local heading from the Sim (Use VICON GLOBAL Heading with Vicon)
    cTheta = RobotPosition.theta #Grab NAOs Heading
    
    #Create matrix with Global difference in position (x right y up)
    PosErrorMat2 = np.matrix([[posx-NaoGlobalx],[posy-NaoGlobaly]])

    #Transform Global Difference (x right y up) to NAO Global (x up y left)
    PosErrorMatNao = np.matmul(coordMatrixClock,PosErrorMat2)

    #Create the desired velocity vector in Global (x right y up)
    VdesGlobal = np.matrix([[xGdesDeriv],[yGdesDeriv]])

    #Create the desired velocity vector in Global NAO (x up y left)
    VdesGlobalNAO = np.matmul(coordMatrixClock,VdesGlobal)
    #print "Velocity NAO",VdesGlobalNAO

    #Create Vc command Velocity in NAO Global (x up y left)
    Vcc = VdesGlobalNAO+Kp*(PosErrorMatNao)
    Vccx = Vcc[0,0]
    Vccy = Vcc[1,0]

    #Compute the Desired Global Thy
    ThyDes=(math.atan2(Vccy,Vccx))

    #Compute Desired turn angle error
    thyAng3 = ThyDes-cTheta

    #Deal with the heading flip around the unit circle
    if thyAng3>np.pi:
        finalHeading = thyAng3-(2*np.pi)
    elif thyAng3<-np.pi:
        finalHeading = thyAng3+(2*np.pi)
    else:
        finalHeading = thyAng3
            
    #Compute Desired turn rate
    turnRate = Kthy*finalHeading

    #Compute Desired Forward Speed
    vcxsquare = np.square(Vccx)
    vcysquare = np.square(Vccy)
    Sdes = math.sqrt(vcxsquare+vcysquare)

    #If sDes (Forward Speed) is greater than 0.08m/s then set it to 0.08m/s as this is approximately NAOs fastest forward movement speed (10.5cms/s actually)
    if Sdes>0.08:
        Sdes=0.08
    
    #Check if turn rate is greater than 0.53rad/s in both directions (NAO Max Turn Rate)
    if turnRate>0.5:
        turnRate=0.5
    if turnRate<-0.5:
        turnRate=-0.5
    #SpeedFile.write("%s\n" % np.column_stack((Sdes,t)))
    dFromNextPoint = np.square(PosErrorMat2[0,0]+PosErrorMat2[1,0])

    #Limit speed of robot if it is close
    #if dFromNextPoint>0.05:
    #    motion_service.move(Sdes,0,turnRate)
    #else:
    #    Sdes=Sdes*0.5
    #    motion_service.move(Sdes,0,turnRate)
    #print "Forward Speed = ",Sdes

    ##Set Robot Movement Speed and Turn Rate
    motion_service.move(Sdes,0,turnRate)

    return(NaoGlobal,dFromNextPoint) 
        
def __init__(
        self,position
):
    self.position = position
     


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'")##127.0.0.1 or 192.168.1.22
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
            "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)


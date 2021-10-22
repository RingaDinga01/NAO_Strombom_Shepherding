# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 21:04:50 2019

@author: Lab User
"""

import numpy as np

def Shepherds(PaddockLength,
              SheepMatrix,
              ShepherdMatrix,
              NumberOfShepherds,
              ShepherdStep,
              SheepRadius,
              NoiseLevel,
              Action,
              safeDist, 
              SheepGlobalCentreOfMass,
              SubGoal,
              Flag=1):

    ''' INPUTS
    
    % PaddockLength             = Width and Height of the environment 
    % TargetCoordinate          = Coordinate of the target
    % SheepMatrix               = Sheep position matrix
    % ShepherdMatrix            = Shepherd position matrix
    % NumberOfShepherds         = Number of shepherds
    % ShepherdStep              = Displacement per time step
    % ShepherdRadius            = Relative strength of repulsion from other shepherds
    % NoiseLevel                = Relative strength of angular noise
    
    % NeighbourhoodSize         = Number of neighbors sheep
    % SheepRadius               = Radius of sheep
    % ShepherdRadius            = Radius of shepherd
    
    % OUTPUT
    % ShepherdUpdatedMatrix     = Updated shepherd object population matrix'''
  

    ShepherdUpdatedMatrix = ShepherdMatrix
    
    # 1 shepherd case:
    RealAction = Action
    
    for TheShepherd in range(0,NumberOfShepherds): # Go through every shepherd object.         
                
        ### Find Direction to move quickly behind the sheep flock
        Prey = SheepMatrix[:,0:2]
        Predator = ShepherdMatrix[TheShepherd,0:2]
        ShepherdToSheep = Prey - Predator
        QuartersShepherdToSheep = 3 - (1*(ShepherdToSheep[:,0]>0)+(2) * (ShepherdToSheep[:,1]>0))
        Q1 = np.sum(QuartersShepherdToSheep == 0)
        Q2 = np.sum(QuartersShepherdToSheep == 1)
        # Q3 = np.sum(QuartersShepherdToSheep == 3)
        Q4 = np.sum(QuartersShepherdToSheep == 2)
        # Flag = 0
        # ShepherdToTarget = TargetCoordinate - Predator;
        # QTarget = 3 - (1*(ShepherdToTarget(1)>0)+(2) * (ShepherdToTarget(2)>0));
        # QTarget = 0 for Q1, 1 for Q2, 3 for Q3, and 2 for Q4
        Shepherd_CurPos = np.zeros(2)#ShepherdMatrix[TheShepherd,0:2]
        Shepherd_CurPos[0]=ShepherdMatrix[0,0]
        Shepherd_CurPos[1]=ShepherdMatrix[0,1]
        Shepherd_NextPos = np.zeros(2)
        Shepherd_NextPos[0] = ShepherdMatrix[0,0]
        Shepherd_NextPos[1] = ShepherdMatrix[0,1]
        if Flag == 0:
            if (Q1 > 0 or Q4 > 0):
                Shepherd_NextPos[0] = ShepherdMatrix[TheShepherd,0] + ShepherdStep
                ShepherdUpdatePos = checkNotDisturbingSheep(safeDist,Shepherd_CurPos,Shepherd_NextPos,                                                        
                                                        SheepGlobalCentreOfMass, SubGoal,ShepherdStep)
               
                ShepherdUpdatedMatrix[TheShepherd,0] = ShepherdUpdatePos[0]
                ShepherdUpdatedMatrix[TheShepherd,1] = ShepherdUpdatePos[1]

                RealAction[0,0] = ShepherdStep
                RealAction[0,1] = 0
            else:
                if Q2 > 0:
                    Shepherd_NextPos[1] = ShepherdMatrix[TheShepherd,1] + ShepherdStep
                    ShepherdUpdatedMatrix[TheShepherd,0:2] = checkNotDisturbingSheep(safeDist,Shepherd_CurPos,Shepherd_NextPos,                                                        
                                                                SheepGlobalCentreOfMass, SubGoal,ShepherdStep)
                    RealAction[0,0] = 0
                    RealAction[0,1] = ShepherdStep
                else:
                    Flag = 1
        
        ### If the shepherds are behind the sheep flock, run to subgoals
        else:
	    '''
            if (Action == 0): # Up         
                NormalisedDirection = np.array([0,1])
                 
            if (Action == 1): # Down                
                NormalisedDirection = np.array([0,-1])
                                    
            if (Action == 2): # Right
                NormalisedDirection = np.array([1,0])
                                   
            if (Action == 3): # Left
                NormalisedDirection = np.array([-1,0])'''
                  
            ShepherdDistanceToSheep = np.sqrt((SheepMatrix[:,0]-ShepherdMatrix[TheShepherd,0])**2 + 
                                              (SheepMatrix[:,1]-ShepherdMatrix[TheShepherd,1])**2)    

            if np.min(ShepherdDistanceToSheep) < 3*SheepRadius: # if shepherd distance to any sheep < 3 ra then shepherd does not move
                RandomNoise = NoiseLevel*np.clip(np.random.randn(2),-1,1)
                Shepherd_NextPos = ShepherdMatrix[TheShepherd,0:2] + RandomNoise #NoiseLevel*np.clip(np.random.randn(2),-1,1)
                ShepherdUpdatedMatrix[TheShepherd,0:2] = checkNotDisturbingSheep(safeDist,Shepherd_CurPos,Shepherd_NextPos,                                                        
                                                        SheepGlobalCentreOfMass, SubGoal,ShepherdStep)
                
                RealAction = Action*0 + RandomNoise
            else:
                
                RandomNoise = NoiseLevel*np.clip(np.random.randn(2),-1,1)
                Shepherd_NextPos = ShepherdMatrix[TheShepherd,0:2] + Action[0]*ShepherdStep + RandomNoise
                                                         #NoiseLevel*np.clip(np.random.randn(2),-1,1))
                ShepherdUpdatedMatrix[TheShepherd,0:2] = checkNotDisturbingSheep(safeDist,Shepherd_CurPos,Shepherd_NextPos,                                                        
                                                        SheepGlobalCentreOfMass, SubGoal,ShepherdStep)
                RealAction = Action*0 + RandomNoise
                
                #print "Action", Action
                #print "Shepherd Matrix", ShepherdMatrix[TheShepherd,0:2]
                #print "Shepherd Update" ,ShepherdUpdatedMatrix
                #print "Shepherd Next", Shepherd_NextPos
                #print "Shepherd Current" , Shepherd_CurPos
                RealAction[0,0] = ShepherdUpdatedMatrix[0,0] - Shepherd_CurPos[0]
                RealAction[0,1] = ShepherdUpdatedMatrix[0,1] - Shepherd_CurPos[1]  
                NormOfAction = np.sqrt(RealAction[0,0]**2+RealAction[0,1]**2)
                RealAction = RealAction/NormOfAction
                #print "NormofAction" ,NormOfAction
                #print "Real Action Else", RealAction
                RealAction = RealAction*ShepherdStep #+ RandomNoise#NoiseLevel*np.clip(np.random.randn(2),-1,1)
               
                #if Shepherd_NextPos[0]!=ShepherdUpdatedMatrix[0,0] and Shepherd_NextPos[1]!=ShepherdUpdatedMatrix[0,1]:
                    #print "In Circle", RealAction
                #if Shepherd_NextPos[0]==ShepherdUpdatedMatrix[0,0] and Shepherd_NextPos[1]==ShepherdUpdatedMatrix[0,1]:
                    #print "Out Circle", RealAction

               
        
   

                    
        ## Limit the movement inside the paddock:
              
        if (ShepherdUpdatedMatrix[TheShepherd,0] < -1.4):
            ShepherdUpdatedMatrix[TheShepherd,0] = -1.4
            RealAction[0,0] = 0
            RealAction[0,1] = 0
              
        if (ShepherdUpdatedMatrix[TheShepherd,1] < -1.4):
            ShepherdUpdatedMatrix[TheShepherd,1] = -1.4
            RealAction[0,0] = 0
            RealAction[0,1] = 0
                
        if (ShepherdUpdatedMatrix[TheShepherd,0] > PaddockLength+1.4):
            ShepherdUpdatedMatrix[TheShepherd,0] = PaddockLength+1.4
            RealAction[0,0] = 0
            RealAction[0,1] = 0
              
        if (ShepherdUpdatedMatrix[TheShepherd,1] > PaddockLength+1.4):
            ShepherdUpdatedMatrix[TheShepherd,1] = PaddockLength+1.4
            RealAction[0,0] = 0
            RealAction[0,1] = 0
        
    return (ShepherdUpdatedMatrix,Flag,RealAction)

def checkNotDisturbingSheep(safeDist, Shepherd_CurPos, Shepherd_NextPos,SheepGlobalCentreOfMass, SubGoal,ShepherdStep):
        #calculate the following to be used
        SP = Shepherd_CurPos
        LCM = SheepGlobalCentreOfMass
        TP = Shepherd_NextPos
        theta_step = 0.2
        TP_theta = np.arctan2(TP[1] - LCM[1], TP[0] - LCM[0])
        Pcd = SubGoal
        Pcd_theta = np.arctan2(Pcd[1]- LCM[1], Pcd[0] - LCM[0])
        #print "SP", SP
        #print "TP", TP
        
        sheepDogNextDistance = np.sqrt((SP[0] - LCM[0])**2 + (SP[1] - LCM[1])**2) # sheepDog->position_t1.dist(sheepDog->Lambda_t);
        sheepDog_UpdatePos = TP
        #print sheepDogNextDistance
        if ((sheepDogNextDistance < safeDist+0.1) or (sheepDogNextDistance < safeDist-0.1)) and np.abs(TP_theta - Pcd_theta) > 2 * theta_step:
            #print "circle"
            #print "safeDist = ", safeDist
            r = np.minimum(safeDist, sheepDogNextDistance + ShepherdStep)
           
            #Define the closest points to SP and TP on the circle
            SP_theta = np.arctan2(SP[1] - LCM[1], SP[0] - LCM[0])
            SP_cx = r * np.cos(SP_theta) + LCM[0]
            SP_cy = r * np.sin(SP_theta) + LCM[1]

            TP_cx = r * np.cos(TP_theta) + LCM[0]
            TP_cy = r * np.sin(TP_theta) + LCM[1]

            #Identify the next step until reaching the target point
            NP_x = SP_cx
            NP_y = SP_cy

            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            #check if clockwise is shorter or counterclockwise
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            #1. first augment the negative values
            if (SP_theta < 0):
                SP_theta = SP_theta + 2 * np.pi

            if ((TP_theta) < 0):
                TP_theta = TP_theta + 2 * np.pi


            #2. Decide on clockwise or counterclockwise based on distance between
            #Starting point SP and target point TP
            NP_theta = SP_theta
            #NP refers to Next point.Identify its theta first, then estimat its x, y values

            if (0 < (SP_theta - TP_theta) and (SP_theta - TP_theta) < np.pi):
                #clockwise is shorter                

                while (NP_theta > TP_theta):
                    NP_theta = NP_theta - theta_step
                    NP_cx = r * np.cos(NP_theta) + LCM[0]
                    NP_cy = r * np.sin(NP_theta) + LCM[1]

            elif ((SP_theta - TP_theta) > np.pi):

                #counterclockwise is shorter
                while (NP_theta < TP_theta + 2 * np.pi):
                    NP_theta = NP_theta + theta_step
                    NP_cx = r * np.cos(NP_theta) + LCM[0]
                    NP_cy = r * np.sin(NP_theta) + LCM[1]

            elif ((SP_theta - TP_theta) < 0):

                #counterclockwise is shorter               
                while (NP_theta < TP_theta):
                    NP_theta = NP_theta + theta_step
                    NP_cx = r * np.cos(NP_theta) + LCM[0]
                    NP_cy = r * np.sin(NP_theta) + LCM[1]

            sheepDog_UpdatePos = [NP_cx, NP_cy]
        return sheepDog_UpdatePos



      
          
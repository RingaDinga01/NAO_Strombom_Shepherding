# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 12:05:22 2019

@author: TALabUser
"""


import numpy as np
import math

def create_Env(PaddockLength,NumberOfShepherds,NumberOfSheep,MaximumSheepDistanceToGlobalCentreOfMass,Radius_subgoal,SheepRadius,case,initRobotPositionMatrix):
    
    """Create the sheep matrix, shepherd matrix and target"""
    TargetCoordinate = np.array([0,0]) # Target coordinates
    # Spawn a random global center of mass near the center of the field    
    SheepMatrix = np.zeros([NumberOfSheep,5]) # initial population of sheep **A matrix of size OBJECTSx5
    #create the shepherd matrix
    ShepherdMatrix = np.zeros([NumberOfShepherds,5])##np.zeros([NumberOfShepherds,5]) Make it random if you like # initial population of shepherds **A matrix of size OBJECTSx5   
    ShepherdMatrix[0,0]=initRobotPositionMatrix[0,0]
    ShepherdMatrix[0,1]=initRobotPositionMatrix[0,1]
    #print ShepherdMatrix
    # Initialize first sheep
   
    SheepIndex = 1
   
    # Case 1: Finding driving point: All sheep are collected. Dog is far from driving point
    #Initialise Sheep within MaximumSheepDistanceToGlobalCentreOfMass
    if case == 1:
        SheepMatrix[0,[0,1]] = [2,3]#np.random.rand(1,2)*PaddockLength*3/4
        SheepMatrix[1,[0,1]] = SheepMatrix[0,[0,1]]
        SheepMatrix[1,[0]] = SheepMatrix[1,[0]] +2#+ 1
        SheepMatrix[1,[1]] = SheepMatrix[1,[1]] -1#- 3
        SheepMatrix[2,[0,1]] = SheepMatrix[0,[0,1]]
        SheepMatrix[2,[0]] = SheepMatrix[2,[0]] +2.2#+ 1.5
        SheepMatrix[2,[1]] = SheepMatrix[2,[1]] -1#- 3
       
        GCM = np.sum(SheepMatrix[:,[0,1]],0)/(SheepIndex+1)
     
        #SheepMatrix[0,[0,1]] = (np.random.rand(1,2)*PaddockLength*3/4)
        #while SheepIndex < NumberOfSheep:
        #    SheepMatrix[SheepIndex,[0,1]] = np.random.rand(1,2)*PaddockLength*3/4
        #    GCM = np.sum(SheepMatrix[:,[0,1]],0)/(SheepIndex+1)
        #    if cal_dist(SheepMatrix[SheepIndex,[0,1]],GCM) <= (MaximumSheepDistanceToGlobalCentreOfMass/2):
        #        SheepIndex += 1
        
        #DrivingPoint = cal_subgoal_behindcenter(NumberOfSheep,GCM,TargetCoordinate,SheepRadius)
        #Initialise Shepherd far from the driving point
	
        #ShepherdIndex = 0
        #while ShepherdIndex < NumberOfShepherds:
        #    ShepherdMatrix[ShepherdIndex,[0,1]] = np.random.rand(1,2)*PaddockLength 
        #    dist_pd_GCM = cal_dist(ShepherdMatrix[ShepherdIndex,[0,1]],GCM)
	        #d = cal_dist(ShepherdMatrix[ShepherdIndex,[0,1]],DrivingPoint)
	    
        #    if (dist_pd_GCM > MaximumSheepDistanceToGlobalCentreOfMass and dist_pd_GCM < 2*MaximumSheepDistanceToGlobalCentreOfMass and cal_dist(ShepherdMatrix[ShepherdIndex,[0,1]],DrivingPoint) > Radius_subgoal and ShepherdMatrix[ShepherdIndex,0] > GCM[0] and ShepherdMatrix[ShepherdIndex,1] > GCM[1]):
        #        ShepherdIndex += 1
    	
        
    # Case 2: Finding collecting point: one sheep is not collected. Dog is far from collecting point
    # Initialise Sheep within MaximumSheepDistanceToGlobalCentreOfMass
    if case == 2:
        SheepMatrix[0,0:2] = np.random.rand(1,2)*PaddockLength*3/4
        
        while SheepIndex < NumberOfSheep-1:
            SheepMatrix[SheepIndex,[0,1]] = np.random.rand(1,2)*PaddockLength*3/4
            GCM = np.sum(SheepMatrix[:,[0,1]],0)/(SheepIndex+1)
            if cal_dist(SheepMatrix[SheepIndex,[0,1]],GCM) <= (MaximumSheepDistanceToGlobalCentreOfMass/2):               
                SheepIndex += 1
        
        while SheepIndex < NumberOfSheep:
            SheepMatrix[SheepIndex,[0,1]] = np.random.rand(1,2)*PaddockLength*3/4
            GCM = np.sum(SheepMatrix[:,[0,1]],0)/(SheepIndex+1)
            if cal_dist(SheepMatrix[SheepIndex,[0,1]],GCM) > MaximumSheepDistanceToGlobalCentreOfMass:                
                SheepIndex += 1
        
        CollectingPoint = cal_subgoal_behindfurthest(GCM,SheepMatrix,TargetCoordinate,SheepRadius)   
        #Initialise Shepherd far from the collecting point
        ShepherdIndex = 0
        while ShepherdIndex < NumberOfShepherds:
            ShepherdMatrix[ShepherdIndex,[0,1]] = np.random.rand(1,2)*PaddockLength
            dist_pd_GCM = cal_dist(ShepherdMatrix[ShepherdIndex,[0,1]],GCM)            
            if (dist_pd_GCM > MaximumSheepDistanceToGlobalCentreOfMass and dist_pd_GCM < 2*MaximumSheepDistanceToGlobalCentreOfMass and cal_dist(ShepherdMatrix[ShepherdIndex,[0,1]],CollectingPoint) > Radius_subgoal and ShepherdMatrix[ShepherdIndex,0] > GCM[0] and ShepherdMatrix[ShepherdIndex,1] > GCM[1]):
                ShepherdIndex += 1
 
   
    #Initialise Sheep Initial Directions Angle [-pi,pi]
    SheepMatrix[:,2] = np.pi - np.random.rand(len(SheepMatrix[:,2]))*2*np.pi #1 - because just having one column

    #Add the index of each sheep into the matrix
    SheepMatrix[:,4] = np.arange(0,len(SheepMatrix[:,4]),1)
    #Initialise Shepherds Initial Directions Angle [-pi,pi]
    ShepherdMatrix[:,2]= np.pi - np.random.rand(NumberOfShepherds)*2*np.pi

    #Add the index of each shepherd into the matrix
    ShepherdMatrix[:,4] = np.arange(0,len(ShepherdMatrix[:,4]),1)

  
    return list([SheepMatrix,ShepherdMatrix,TargetCoordinate])


def cal_dist(v1,v2):
    
    distance = np.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)
    return distance

def cal_subgoal_behindcenter(NumberOfSheep,SheepGlobalCentreOfMass,TargetCoordinate,SheepRadius):
      DirectionFromTargetToGlobalCentreOfMass = np.array([SheepGlobalCentreOfMass[0]-TargetCoordinate[0], SheepGlobalCentreOfMass[1]-TargetCoordinate[1]])
      NormOfDirectionFromTargetToGlobalCentreOfMass = np.sqrt(DirectionFromTargetToGlobalCentreOfMass[0]**2 + DirectionFromTargetToGlobalCentreOfMass[1]**2)
      NormalisedDirectionFromTargetToGlobalCentreOfMass = DirectionFromTargetToGlobalCentreOfMass / NormOfDirectionFromTargetToGlobalCentreOfMass
      PositionBehindCenterFromTarget = NormalisedDirectionFromTargetToGlobalCentreOfMass * (NormOfDirectionFromTargetToGlobalCentreOfMass + SheepRadius * (NumberOfSheep**(2/3)))

      return PositionBehindCenterFromTarget         
  
def cal_subgoal_behindfurthest(SheepGlobalCentreOfMass,SheepMatrix,TargetCoordinate,SheepRadius):
      DirectionFromTargetToGlobalCentreOfMass = np.array([SheepGlobalCentreOfMass[0]-TargetCoordinate[0], SheepGlobalCentreOfMass[1]-TargetCoordinate[1]])   
      CS_FS_x = SheepMatrix[-1,0] - SheepGlobalCentreOfMass[0] # CS to FS x
      CS_FS_y = SheepMatrix[-1,1] - SheepGlobalCentreOfMass[1] # CS to FS y
  
      DirectionFromGlobalCentreOfMassToFurthestSheep = np.array([CS_FS_x, CS_FS_y])
      NormOfDirectionFromGlobalCentreOfMassToFurthestSheep = np.sqrt(DirectionFromGlobalCentreOfMassToFurthestSheep[0]**2 + DirectionFromGlobalCentreOfMassToFurthestSheep[1]**2)
      NormalisedDirectionFromGlobalCentreOfMassToFurthestSheep = DirectionFromGlobalCentreOfMassToFurthestSheep / NormOfDirectionFromGlobalCentreOfMassToFurthestSheep
      DirectionFromGlobalCentreOfMassToPositionBehindFurthestSheep = NormalisedDirectionFromGlobalCentreOfMassToFurthestSheep*(NormOfDirectionFromGlobalCentreOfMassToFurthestSheep + SheepRadius)
      PositionBehindFurthestSheep = DirectionFromTargetToGlobalCentreOfMass + DirectionFromGlobalCentreOfMassToPositionBehindFurthestSheep
      return PositionBehindFurthestSheep
  
    
    
    

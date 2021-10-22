# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 20:19:43 2019

@author: Lab User
"""

from SheepCalculations import SheepCalculations
import numpy as np

def Sheeps(PaddockLength,SheepMatrix,NeighbourhoodSize,
           ShepherdMatrix,SheepRadius,SheepSensingOfShepherdRadius,
           SheepStep,WeightOfInertia,WeightRepellFromOtherSheep,
           WeightAttractionToLocalCentreOfMassOfnNeighbours,
           WeightRepellFromShepherd,NoiseLevel):

  

# INPUTS
# SheepMatrix = Sheep position matrix
# NeighbourhoodSize = Number of neighbors sheep
# ShepherdMatrix = Shepherd position matrix
# SheepRadius = Radius of sheep
# ShepherdRadius = Radius of shepherd
# SheepStep = Displacement per time step

# OUTPUT
# UpdatedSheepMatrix = Updated sheep object population matrix P
    
    NumberOfSheep = len(SheepMatrix)
   
    UpdatedSheepMatrix = np.zeros([NumberOfSheep, 5]) # For simultaneous update
    UpdatedSheepMatrix_nonoise = np.zeros([NumberOfSheep, 5]) # For simultaneous update
    CMAL_ALL = None
    for i in range(0,NumberOfSheep): # Go through every sheep object.

        CMAL=SheepCalculations(i,SheepMatrix,NeighbourhoodSize,ShepherdMatrix,SheepRadius,SheepSensingOfShepherdRadius) 
    
        RepulsionDirectionFromOtherSheep    = CMAL[0,:]	# Direction of repulsion from other sheep
        
        RepulsionDirectionFromShepherds     = CMAL[1,:]	# Direction of repulsion from shepherds
       
        AttractionDirectionToOtherSheep     = CMAL[2,:]	# Direction of attraction to other sheep.
        
        DistanceToShepherds                 = CMAL[3,0]	# Distance to shepherds.
        
        #NumberOfNearestNeighbours           = CMAL(3,1)	# Number of nearest
        #neighbours - Not Used in This Function
        
        if CMAL_ALL is None:
            CMAL_ALL = CMAL.reshape([1,8])
        else:
            CMAL_ALL = np.vstack([CMAL_ALL,CMAL.reshape([1,8])])
                                 
        PreviousDirection = np.array([np.cos(SheepMatrix[i,2]),np.sin(SheepMatrix[i,2])]) # Direction in the previous time step
         
        if (DistanceToShepherds < SheepSensingOfShepherdRadius):                     # If reacting to the shepherd
        
             #Calculate new direction
             NewDirection=WeightOfInertia*PreviousDirection+WeightRepellFromOtherSheep*RepulsionDirectionFromOtherSheep+WeightRepellFromShepherd*RepulsionDirectionFromShepherds+WeightAttractionToLocalCentreOfMassOfnNeighbours*AttractionDirectionToOtherSheep+NoiseLevel*np.clip(np.random.randn(2),-1,1) # New direction of sheep i
        
             
             if (NewDirection[0] == 0 and NewDirection[1] == 0):
               NormalisedNewDirection = np.array([0,0])
             else:
               NormalisedNewDirection=(NewDirection/np.sqrt(NewDirection[0]**2+NewDirection[1]**2)) #Normalized direction of sheep i
            
             # Update position
             UpdatedSheepMatrix[i,0:2]=SheepMatrix[i,0:2]+SheepStep*NormalisedNewDirection[0:2]# + NoiseLevel*np.clip(np.random.randn(2),-1,1) 
             UpdatedSheepMatrix_nonoise[i,0:2]=SheepMatrix[i,0:2]+SheepStep*NormalisedNewDirection[0:2]
             # New directional angle
             UpdatedSheepMatrix[i,2]=np.arctan2(NormalisedNewDirection[1],NormalisedNewDirection[0])
	           #UpdatedSheepMatrix_nonoise[i,2]=np.arctan2(NormalisedNewDirection[1],NormalisedNewDirection[0])
             
             # Preserve the index of the object
             UpdatedSheepMatrix[i,4]=SheepMatrix[i,4]
        
        else: # If not reacting to the shepherd, add in small random movement  
        
              UpdatedSheepMatrix[i,0:2]=SheepMatrix[i,0:2]+ NoiseLevel*np.clip(np.random.randn(2),-1,1)
              UpdatedSheepMatrix[i,2:5]=SheepMatrix[i,2:5]

              UpdatedSheepMatrix_nonoise[i,0:2]=SheepMatrix[i,0:2]#+ NoiseLevel*np.clip(np.random.randn(2),-1,1)
              UpdatedSheepMatrix_nonoise[i,2:5]=SheepMatrix[i,2:5]
        
       
        if (UpdatedSheepMatrix[i,0] < 0 ):
          UpdatedSheepMatrix[i,0] = 0
	  UpdatedSheepMatrix_nonoise[i,0] = 0
          
           
        if (UpdatedSheepMatrix[i,1] < 0 ):
          UpdatedSheepMatrix[i,1] = 0
          UpdatedSheepMatrix_nonoise[i,1] = 0
           
        if( UpdatedSheepMatrix[i,0] > PaddockLength ):
          UpdatedSheepMatrix[i,0] = PaddockLength
          UpdatedSheepMatrix_nonoise[i,0] = PaddockLength
           
        if (UpdatedSheepMatrix[i,1] > PaddockLength ):
          UpdatedSheepMatrix[i,1] = PaddockLength
	  UpdatedSheepMatrix_nonoise[i,1] = PaddockLength
        
           
    
    return(CMAL_ALL,UpdatedSheepMatrix)
    

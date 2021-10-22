# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 14:28:27 2019

@author: Lab User
"""
import numpy as np

def SheepCalculations(i,SheepPosition,NumberOfSheepNeighbours,ShepherdPosition,SheepRadius,ShepherdRadius):

  

        # INPUTS
    # i=Index of sheep being calculated
    # SheepPosition=Sheep position matrix
    # NumberOfSheepNeighbours=Number of neighbours
    # ShepherdPosition=Position matrix of shepherds
    # SheepRadius=Radius of sheep
    # ShepherdRadius=Radius of shepherd
    
    # OUTPUTS
    # RepulsionSheepDirection=Direction of repulsion from other sheep
    # ShepherdRepulsionDirection=Direction of repulsion from shepherds
    # AttractionDirectionToOtherSheep=Direction of attraction to other sheep.
    # DistanceToShepherd=Distance to shepherds.
    # NumberOfNearestNeighbour=Number of nearest neighbours
    
    NumberOfSheep = SheepPosition.shape[0]
    NumberOfShepherd = ShepherdPosition.shape[0]
    SheepNeighbourhoodMatrix = np.zeros([NumberOfSheep,2])
    ShepherdNeighbourhoodMatrix = np.zeros([NumberOfShepherd,2])
    NumberOfSheepCloseToASheep = 0
    NumberOfShepherdCloseToASheep = 0
    CummulativeSheepToASheepDirection = np.array([0,0])
    CummulativeShepherdToSheepDirection = np.array([0,0])

    ## Repulsion vectors from shepherds
    for k in range(0,NumberOfShepherd):
    
      IndividualSheepDistanceToShepherd = np.sqrt((SheepPosition[i,0]-ShepherdPosition[k,0])**2+
                                                  (SheepPosition[i,1]-ShepherdPosition[k,1])**2) # distance from shepherd
      if (IndividualSheepDistanceToShepherd<ShepherdRadius): # If shepherd within rShepherd        
        ShepherdNeighbourhoodMatrix[NumberOfShepherdCloseToASheep,:] = np.array([ShepherdPosition[k,0],ShepherdPosition[k,1]])
        NumberOfShepherdCloseToASheep = NumberOfShepherdCloseToASheep+1
        
      if ( k == 0 ):
        MinimumDistanceToShepherd = IndividualSheepDistanceToShepherd
      
      else:
        if (MinimumDistanceToShepherd > IndividualSheepDistanceToShepherd): 
          MinimumDistanceToShepherd = IndividualSheepDistanceToShepherd

    
    ShepherdNeighbourhoodMatrix=ShepherdNeighbourhoodMatrix[0:NumberOfShepherdCloseToASheep,:] # restrict size to that needed
    
    if(NumberOfShepherdCloseToASheep > 0):
    
        for k in range(0,NumberOfShepherdCloseToASheep): 
            ShepherdToSheepDirectionVector = np.array([SheepPosition[i,0]-ShepherdNeighbourhoodMatrix[k,0],SheepPosition[i,1]-ShepherdNeighbourhoodMatrix[k,1]])
            
            NormOfShepherdToSheepDirectionVector = np.sqrt(ShepherdToSheepDirectionVector[0]**2+ShepherdToSheepDirectionVector[1]**2)
            
            if (NormOfShepherdToSheepDirectionVector == 0):
                NormalisedShepherdToSheepDirectionVector = np.array([0,0])
            else:
                NormalisedShepherdToSheepDirectionVector = ShepherdToSheepDirectionVector/NormOfShepherdToSheepDirectionVector
            
            CummulativeShepherdToSheepDirection = CummulativeShepherdToSheepDirection+NormalisedShepherdToSheepDirectionVector
        
        NormOfCummulativeShepherdToSheepDirection = np.sqrt(CummulativeShepherdToSheepDirection[0]**2+CummulativeShepherdToSheepDirection[1]**2)
        if (NormOfCummulativeShepherdToSheepDirection == 0):
          RepulsionFromShepherdDirection = np.array([0,0])
        else:
          RepulsionFromShepherdDirection = CummulativeShepherdToSheepDirection/NormOfCummulativeShepherdToSheepDirection
    
    else:
      RepulsionFromShepherdDirection= np.array([0,0])


    ## Repulsion vectors from other sheep
    for j in range(0,NumberOfSheep):
    
      if (j!=i):
        if (((SheepPosition[i,0]-SheepPosition[j,0])**2 + (SheepPosition[i,1]- SheepPosition[j,1])**2) < (SheepRadius**2)): #If sheep j within dist Rsheep from sheep i
    
          SheepNeighbourhoodMatrix[NumberOfSheepCloseToASheep,:] = np.array([SheepPosition[j,0], SheepPosition[j,1]])
          NumberOfSheepCloseToASheep=NumberOfSheepCloseToASheep+1
  
   
    SheepNeighbourhoodMatrix = SheepNeighbourhoodMatrix[0:NumberOfSheepCloseToASheep,:]
    
    if (NumberOfSheepCloseToASheep > 0):
    
      for j in range(0,NumberOfSheepCloseToASheep): #For each neighbour too close
                
        SheepToASheepDirectionVector = np.array([SheepPosition[i,0]- SheepNeighbourhoodMatrix[j,0], SheepPosition[i,1]- SheepNeighbourhoodMatrix[j,1]])
                
        NormOfSheepToASheepDirectionVector = np.sqrt(SheepToASheepDirectionVector[0]**2+SheepToASheepDirectionVector[1]**2)
        
        if (NormOfSheepToASheepDirectionVector == 0):
          NormalisedSheepToASheepDirectionVector = np.array([0,0])
        else:
          NormalisedSheepToASheepDirectionVector = SheepToASheepDirectionVector/NormOfSheepToASheepDirectionVector #Relative strength is inversly proportional to distance.
        CummulativeSheepToASheepDirection = CummulativeSheepToASheepDirection + NormalisedSheepToASheepDirectionVector #Add upp all repulsion vectors
        
      
      NormOfCummulativeSheepToSheepDirection = np.sqrt(CummulativeSheepToASheepDirection[0]**2+CummulativeSheepToASheepDirection[1]**2)
      
      if (NormOfCummulativeSheepToSheepDirection == 0):
        RepulsionFromSheepDirection = np.array([0,0])
      else:
        RepulsionFromSheepDirection = CummulativeSheepToASheepDirection/NormOfCummulativeSheepToSheepDirection
        
    else:
      RepulsionFromSheepDirection = np.array([0,0]) #if no neighbour no new direction.


    ## The sheep attraction to the closest N neighbours
    
    #First we need to exclud the sheep of interest that is, Sheep i
    
    if ((i > 0) and (i < NumberOfSheep-1)):
      #SheepPosition
      SheepPositionExcludingi = np.vstack([SheepPosition[0:(i-1),:], SheepPosition[(i):NumberOfSheep,:]])
    else:
        if (i == 0):
          SheepPositionExcludingi = SheepPosition[1:NumberOfSheep,:]
        else:
          SheepPositionExcludingi = SheepPosition[0:(NumberOfSheep-1),:]
      
      
  
    DistanceBetweenAllSheepAndTheSheep = np.vstack([np.sqrt((SheepPositionExcludingi[:,0]-SheepPosition[i,0])**2+ (SheepPositionExcludingi[:,1]-SheepPosition[i,1])**2),SheepPositionExcludingi[:,4]]).T # distance and index number of all sheep
    ClosestSheepToTheSheep = (DistanceBetweenAllSheepAndTheSheep[DistanceBetweenAllSheepAndTheSheep[:,0].argsort()])
    
    AveragePosition = np.mean(SheepPosition[ClosestSheepToTheSheep[0:NumberOfSheepNeighbours,1].astype(int),0:2],axis=0)
    UnnormalisedAttractionDirectionToOtherSheep = AveragePosition - SheepPosition[i,0:2]
    NormUnnormalisedAttractionDirectionToOtherSheep = np.sqrt(UnnormalisedAttractionDirectionToOtherSheep[0]**2+UnnormalisedAttractionDirectionToOtherSheep[1]**2)
    
    if (NormUnnormalisedAttractionDirectionToOtherSheep == 0):
      AttractionDirectionToOtherSheep = np.array([0,0])
    else:
      AttractionDirectionToOtherSheep = UnnormalisedAttractionDirectionToOtherSheep / NormUnnormalisedAttractionDirectionToOtherSheep
    
    CMAL=np.vstack([RepulsionFromSheepDirection,RepulsionFromShepherdDirection,AttractionDirectionToOtherSheep,np.array([MinimumDistanceToShepherd,NumberOfSheepCloseToASheep])])
      
    return CMAL

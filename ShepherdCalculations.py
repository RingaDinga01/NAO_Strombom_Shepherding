self.SheepSensingOfShepherdRadius# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 20:43:29 2019

@author: Lab User
"""

import numpy as np

def ShepherdCalculations(TheShepherd,ShepherdRadius,ShepherdMatrix):

  

# INPUTS
# TheShepherd = Index of shepherd under consideration
# ShepherdRadius = shepherd neighbourhood
# ShepherdMatrix = Position matrix of shepherds

# OUTPUTS
# Repulsion Vector of Shepherds 

    NumberOfShepherd = ShepherdMatrix.shape[0]
    #ShepherdNeighbourhoodMatrix = np.zeros([NumberOfShepherd,2])
    #NumberOfShepherdCloseToASheep = 0
    ShepherdRepulsionDirection = np.array([0,0])
    
    ## Repulsion vectors from shepherds
    for k in range(0,NumberOfShepherd):
    
          if (k != TheShepherd):
          
              TheShepherdPosition = ShepherdMatrix[TheShepherd,0:2]
              OtherShepherdPosition = ShepherdMatrix[k,0:2]
              DirectionToOther = TheShepherdPosition - OtherShepherdPosition
              DistanceToOther = np.linalg.norm(DirectionToOther)
         
              if (DistanceToOther < ShepherdRadius):
              
                  NormalisedDirectionToOther = DirectionToOther / DistanceToOther
                  ShepherdRepulsionDirection = ShepherdRepulsionDirection + NormalisedDirectionToOther
              
          
    
    
    if ((ShepherdRepulsionDirection[0] != 0) or (ShepherdRepulsionDirection[1] != 0)):
      ShepherdRepulsionDirection = ShepherdRepulsionDirection / np.linalg.norm(ShepherdRepulsionDirection)
    
    CMAL = ShepherdRepulsionDirection
      
    return CMAL
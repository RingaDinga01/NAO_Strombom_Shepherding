# -*- coding: utf-8 -*-
"""
Created on Thu Dec 19 11:54:22 2019

@author: z5095790
"""

import numpy as np

def findFurthestSheep(SheepMatrix,SheepGlobalCentreOfMass,NumberOfShepherds,MaximumSheepDistanceToGlobalCentreOfMass):
    
    ''' INPUTS

    % SheepMatrix               = Sheep position matrix
    % NumberOfShepherds         = Number of shepherds
    
    % OUTPUT
    % IndexOfFurthestSheep     = Index of Furthest Sheep matrix with the number of selected sheep equal to the number of shepherd
    % AreFurthestSheepCollected to identify if there are still outlier sheep'''
    NumberOfSheep = len(SheepMatrix)
    SheepDistanceToGlobalCentreOfMass = np.sqrt((SheepMatrix[:,0]-SheepGlobalCentreOfMass[0])**2+(SheepMatrix[:,1]-SheepGlobalCentreOfMass[1])**2)
    
    # Find and sort the distance between LCM of the sheep and every sheep
    GCMDistanceToSheepWithIndex = np.zeros([NumberOfSheep,2])
    GCMDistanceToSheepWithIndex[:,0] = SheepDistanceToGlobalCentreOfMass
    GCMDistanceToSheepWithIndex[:,1] = SheepMatrix[:,4]
    SortedGCMDistanceToSheepWithIndex = GCMDistanceToSheepWithIndex[GCMDistanceToSheepWithIndex[:,0].argsort()[::-1]]
    # Find the index of the furthest sheep to LCM
    if NumberOfShepherds > 1:
        IndexOfFurthestSheep = SortedGCMDistanceToSheepWithIndex[0:NumberOfShepherds,1].astype(int)
    else:
        IndexOfFurthestSheep = SortedGCMDistanceToSheepWithIndex[0,1].astype(int)
    # find the collection status of the furthest sheep
    AreFurthestSheepCollected = 0
    
    if SortedGCMDistanceToSheepWithIndex[0,0] <= MaximumSheepDistanceToGlobalCentreOfMass:
        AreFurthestSheepCollected = 1
    
    return(IndexOfFurthestSheep,AreFurthestSheepCollected)

def findDistanceToShepherd(ShepherdMatrix, SheepMatrix):
    NumberOfSheep = len(SheepMatrix)
    SheepDistanceToShepherd = np.sqrt((SheepMatrix[:,0]-ShepherdMatrix[0,0])**2+(SheepMatrix[:,1]-ShepherdMatrix[0,1])**2)
    
    # Find and sort the distance between Shepherd and every sheep
    ShepherdDistanceToSheepWithIndex = np.zeros([NumberOfSheep,2])
    ShepherdDistanceToSheepWithIndex[:,0] = SheepDistanceToShepherd
    ShepherdDistanceToSheepWithIndex[:,1] = SheepMatrix[:,4]
    SortedShepherdDistanceToSheepWithIndex = ShepherdDistanceToSheepWithIndex[ShepherdDistanceToSheepWithIndex[:,0].argsort()[::-1]]
    # Find the index of the furthest sheep to Shepherd
    NumberOfShepherds2 = len(ShepherdMatrix)
    if NumberOfShepherds2 > 1:
        IndexOfFurthestSheep = SortedShepherdDistanceToSheepWithIndex[0:NumberOfShepherds2,1].astype(int)
    else:
        IndexOfFurthestSheep = SortedShepherdDistanceToSheepWithIndex[0,1].astype(int)
    # find the collection status of the furthest sheep 
    FurthestSheepToShepherdDistance = SortedShepherdDistanceToSheepWithIndex[IndexOfFurthestSheep]
    #print "furthest", FurthestSheepToShepherdDistance[0]
    return(FurthestSheepToShepherdDistance[0])

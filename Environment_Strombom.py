# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 13:28:32 2019

@author: Lab User
"""

import numpy as np
from Sheeps import Sheeps
from Shepherds import Shepherds
import Sub_Env2
from findFurthestSheep import findFurthestSheep
from findFurthestSheep import findDistanceToShepherd
import matplotlib.pyplot as plt



class Environment: 
    
    def __init__(
            self,
            NumberOfSheep,
            NeighbourhoodSize,
            NumberOfShepherds,
            case,
            SheepRadius = None,
            SheepSensingOfShepherdRadius = None,
            PaddockLength = None,
            WeightRepellFromOtherSheep = None,# p_0
            WeightAttractionToLocalCentreOfMassOfnNeighbours = None, # c
            WeightRepellFromShepherd = None,# p_s
            WeightOfInertia = None, # h
            NoiseLevel = None, #e
            SheepStep = None, # delta
            ShepherdStep = None, # delta_s,
            StopWhenSheepGlobalCentreOfMassDistanceToTargetIs = None,
            Radius_subgoal = None,
            DogViolatingDistanceToSheep = None,
            
        
            #Previous distance used to calculate reward
            Pre_dist_PD_SubGoal = None,
            iReset = None,
            
            SheepMatrix = None,# initial population of sheep **A matrix of size OBJECTSx5
            #create the shepherd matrix
            ShepherdMatrix= None, # initial population of shepherds **A matrix of size OBJECTSx5
            TargetCoordinate = None, # Target coordinates
                 
    ):
        
      self.NumberOfSheep = NumberOfSheep
      self.NeighbourhoodSize = NeighbourhoodSize
      self.NumberOfShepherds = NumberOfShepherds
      self.case = case
      # Constants
      self.PaddockLength=7
      self.SheepRadius = 0.175
      self.SheepSensingOfShepherdRadius = 1
      self.WeightRepellFromOtherSheep = 0.093 # p_0
      self.WeightAttractionToLocalCentreOfMassOfnNeighbours = 0.050#0.050 # c
      self.WeightRepellFromShepherd = 0.0467# p_s
      self.WeightOfInertia = 0.023 # h
      self.NoiseLevelShepherd = 0.01 #set noise level for shepherd to 0 
      self.NoiseLevelSheep = 0.01 #Set noise level to sheep so they can move
      self.SheepStep = 0.02# delta
      self.ShepherdStep = 0.08 # delta_s
      ## Other Parameters Not Specified in the algorithm
      # StopWhenSheepGlobalCentreOfMassDistanceToTargetIs = 10
      # This parameter depends on the size of the herd
      self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs = 0.467
      self.Radius_subgoal = 0.6
      self.ShepherdRadius  = 1.4
      # self.DogViolatingDistanceToSheep = 3
      # Because the original algorithm is designed for a single shepherd, we need
      # to add a parameter for repulsion among shepherds
      # ShepherdRadius            = Relative strength of repulsion from other shepherds
      # self.ShepherdRadius            = 10 # = 65 in the IJCNN paper with 300x300m environment
      ## Initialise Sheep in the top right quarter of the paddock
      self.CMAL_ALL = None        
      self.SheepGlobalCentreOfMass = None
      self.ShepherdGlobalCentreOfMass = None
      self.Flag = 0
      self.IndexOfFurthestSheep = None
      self.Pre_dist_PD_SubGoal = None
      self.iReset = None      
      self.MaximumSheepDistanceToGlobalCentreOfMass = self.WeightRepellFromOtherSheep * (self.NumberOfSheep**(2/float(3)))+0.2 
      

      #Adding to move in circle
      self.R1 = self.MaximumSheepDistanceToGlobalCentreOfMass 
      self.R2 = 0.40 
      self.R3 = 0.40 
      self.SubGoal = None
      self.sheepDog_t1_2LCM_MinRadius = None

    #Method used to reset the environment and initialise Shepherd/Sheep
    def reset(self,initRobotPositionMatrix):
      
      self.Flag = 0
      self.SheepMatrix, self.ShepherdMatrix, self.TargetCoordinate = Sub_Env2.create_Env(self.PaddockLength,
                                                                                         self.NumberOfShepherds,
                                                                                         self.NumberOfSheep,
                                                                                         self.MaximumSheepDistanceToGlobalCentreOfMass,
                                                                                         self.Radius_subgoal,
                                                                                         self.SheepRadius,
                                                                                         self.case,initRobotPositionMatrix)
      self.CMAL_ALL = np.zeros([self.NumberOfSheep,8])

      #Calculating Initial State
      self.SheepGlobalCentreOfMass= np.array([np.mean(self.SheepMatrix[:,0]),np.mean(self.SheepMatrix[:,1])]) # GCM of sheep objects
      self.ShepherdGlobalCentreOfMass=np.array([np.mean(self.ShepherdMatrix[:,0]),np.mean(self.ShepherdMatrix[:,1])]) # GCM of shepherd objects
  
      self.IndexOfFurthestSheep, AreFurthestSheepCollected = findFurthestSheep(self.SheepMatrix,
                                                                          self.SheepGlobalCentreOfMass,
                                                                          self.NumberOfShepherds,
                                                                          self.MaximumSheepDistanceToGlobalCentreOfMass)     
      InitialState = self.cal_State()
      
      


      #Reset iReset, Previous distances is NULL
      #Previous distance used to calculate reward
      self.Pre_dist_PD_SubGoal = None
      self.iReset = 1
      Terminate = 0
      
      #print("Environment reset.")
      
      return(Terminate,InitialState,AreFurthestSheepCollected) 


    #Additional step function used to get NAOs actual position from the simulation 
    def stepRealAction(self,action,CurrentNaoPos):
      #Distance from the CM to furthest sheep
      furthestDist = np.sqrt((self.SheepGlobalCentreOfMass[0]-self.SheepMatrix[self.IndexOfFurthestSheep,0])**2+
                                  (self.SheepGlobalCentreOfMass[1]-self.SheepMatrix[self.IndexOfFurthestSheep,1])**2) 
      #print furthestDist
      #print self.R1
      self.sheepDog_t1_2LCM_MinRadius = np.maximum(furthestDist, self.R1) + self.R2 + self.R3; #outter (3rd) circle radius
      self.ShepherdMatrix[0,0]=CurrentNaoPos[0,0]
      self.ShepherdMatrix[0,1]=CurrentNaoPos[1,0]
      self.ShepherdMatrix, self.Flag, RealAction  = Shepherds(self.PaddockLength,
                                                    self.SheepMatrix,
                                                    self.ShepherdMatrix,
                                                    self.NumberOfShepherds, 
                                                    self.ShepherdStep, 
                                                    self.SheepRadius,
                                                    self.NoiseLevelShepherd,
                                                    action,self.sheepDog_t1_2LCM_MinRadius,self.SheepGlobalCentreOfMass,self.SubGoal)

      return (RealAction, self.ShepherdMatrix)  
    
    #Step function used to calculate where NAO needs to move to next
    def step(self,action,NextNaoPos):
      #Distance from the CM to furthest sheep
      furthestDist = np.sqrt((self.SheepGlobalCentreOfMass[0]-self.SheepMatrix[self.IndexOfFurthestSheep,0])**2+
                                  (self.SheepGlobalCentreOfMass[1]-self.SheepMatrix[self.IndexOfFurthestSheep,1])**2) 
      #print furthestDist
      #print self.R1
      self.sheepDog_t1_2LCM_MinRadius = np.maximum(furthestDist, self.R1) + self.R2 + self.R3; #outter (3rd) circle radius
      self.ShepherdMatrix[0,0]=NextNaoPos[0,0]
      self.ShepherdMatrix[0,1]=NextNaoPos[1,0]
      #print "Velocity Step = ", RealAction
      #Performing action herding sheeps
      #self.ShepherdMatrix[0,0]=CurrentNaoPos[0,0]
      #self.ShepherdMatrix[0,1]=CurrentNaoPos[1,0]
      self.CMAL_ALL, self.SheepMatrix = Sheeps(self.PaddockLength,
                                                self.SheepMatrix,
                                                self.NeighbourhoodSize,
                                                self.ShepherdMatrix,
                                                self.SheepRadius,
                                                self.SheepSensingOfShepherdRadius,
                                                self.SheepStep,
                                                self.WeightOfInertia,
                                                self.WeightRepellFromOtherSheep,
                                                self.WeightAttractionToLocalCentreOfMassOfnNeighbours,self.WeightRepellFromShepherd,self.NoiseLevelSheep) # Update position of sheep
            
      # Recompute Sheep Centre of mass
      self.SheepGlobalCentreOfMass = np.array([np.mean(self.SheepMatrix[:,0]),np.mean(self.SheepMatrix[:,1])])  # GCM of sheep objects
      self.ShepherdGlobalCentreOfMass = np.array([np.mean(self.ShepherdMatrix[:,0]),np.mean(self.ShepherdMatrix[:,1])]) # GCM of shepherd objects

      self.IndexOfFurthestSheep, AreFurthestSheepCollected = findFurthestSheep(self.SheepMatrix,
                                                                          self.SheepGlobalCentreOfMass,
                                                                          self.NumberOfShepherds,
                                                                          self.MaximumSheepDistanceToGlobalCentreOfMass)
      #Checking terminate
      Terminate, SubGoalPosition =  self.check_terminate()    

      #Calculating Next State
      NextState = self.cal_State()
      infor = (self.SheepMatrix[0,[0]],self.SheepMatrix[0,[1]],self.SheepMatrix[1,[0]],self.SheepMatrix[1,[1]],self.SheepMatrix[2,[0]],self.SheepMatrix[2,[1]],self.SheepGlobalCentreOfMass[0],self.SheepGlobalCentreOfMass[1],
			self.SheepMatrix[self.IndexOfFurthestSheep,0],self.SheepMatrix[self.IndexOfFurthestSheep,1],SubGoalPosition[0],SubGoalPosition[1])
      return (NextState, Terminate,self.ShepherdMatrix,self.SheepMatrix, self.SheepMatrix[self.IndexOfFurthestSheep,:], AreFurthestSheepCollected, infor)   

    #Depending on whether NAO is determined to be driving or collection, the relevant subgoal calculation methods will be run
    def Strombom_action(self,driving):
        if driving == 1:
            self.SubGoal = self.cal_subgoal_behindcenter()
            #print "Subgoal Driving", self.SubGoal
        else:
            self.SubGoal = self.cal_subgoal_behindfurthest()
            #print "Subgoal Collecting", self.SubGoal
        action = self.SubGoal - self.ShepherdMatrix[:,0:2]        
        NormOfAction = np.sqrt(action[:,0]**2+action[:,1]**2)
        action = action/NormOfAction
        return(action,self.SubGoal,driving)
        
    
    def cal_State(self):
      #Calculating State
      # Position of Dog (PD), center of sheeps (CS), furthest sheep(FS), target (T)
      # Or vector of PD to CS, PD to FS, CS to T
      State=np.zeros(4)

      if self.case == 1: # driving
          State[0] = self.SheepGlobalCentreOfMass[0] - self.ShepherdMatrix[0,0] #PD to CS (x)
          State[1] = self.SheepGlobalCentreOfMass[1] - self.ShepherdMatrix[0,1] #PD to CS (y)
          State[2] = self.TargetCoordinate[0] - self.SheepGlobalCentreOfMass[0] #CS to T (x)
          State[3] = self.TargetCoordinate[0] - self.SheepGlobalCentreOfMass[1] #CS to T (y)
      else: # collecting
          State[0] = self.SheepGlobalCentreOfMass[0] - self.ShepherdMatrix[0,0] #PD to CS (x)
          State[1] = self.SheepGlobalCentreOfMass[1] - self.ShepherdMatrix[0,1] #PD to CS (y)          
          State[2] = self.SheepMatrix[self.IndexOfFurthestSheep,0] - self.SheepGlobalCentreOfMass[0] #CS to FS (x)
          State[3] = self.SheepMatrix[self.IndexOfFurthestSheep,1] - self.SheepGlobalCentreOfMass[1] #CS to FS (y)

      State = State/self.PaddockLength

      return State
        
    #Function used to terminate the algorithm when certain conditions are met such as the GCM being moved to within a pre-defined distance from the target location
    def check_terminate(self):
      ### Termination Conditions
      terminate = 0 #Not achieving the target
      if (self.case == 1):
              SubGoalPosition = self.cal_subgoal_behindcenter()
      else:
              SubGoalPosition = self.cal_subgoal_behindfurthest()
            
      dist_PD_SubGoal = np.sqrt((self.ShepherdMatrix[0,0]-SubGoalPosition[0])**2+
                                  (self.ShepherdMatrix[0,1]-SubGoalPosition[1])**2) # Distance GCM sheep to target
      if (dist_PD_SubGoal <= self.Radius_subgoal):
          terminate = 1 #Achieving the target       
        
      return (terminate, SubGoalPosition)

    #Calculates the subgoal direction vector and position when the Shepherd is deemed to be driving the herd because all sheep are aggregated
    def cal_subgoal_behindcenter(self):
      DirectionFromTargetToGlobalCentreOfMass = np.array([self.SheepGlobalCentreOfMass[0]-self.TargetCoordinate[0], self.SheepGlobalCentreOfMass[1]-self.TargetCoordinate[1]])
      NormOfDirectionFromTargetToGlobalCentreOfMass = np.sqrt(DirectionFromTargetToGlobalCentreOfMass[0]**2 + DirectionFromTargetToGlobalCentreOfMass[1]**2)
      NormalisedDirectionFromTargetToGlobalCentreOfMass = DirectionFromTargetToGlobalCentreOfMass / NormOfDirectionFromTargetToGlobalCentreOfMass
      DistanceToShepherdFromFurthestSheep = findDistanceToShepherd(self.ShepherdMatrix,self.SheepMatrix)
      PositionBehindCenterFromTarget = NormalisedDirectionFromTargetToGlobalCentreOfMass * (NormOfDirectionFromTargetToGlobalCentreOfMass + self.SheepSensingOfShepherdRadius-0.25)#* np.sqrt(self.NumberOfSheep)+1.5)#-0.1 works well
      #print "driving"
      return PositionBehindCenterFromTarget
     
    
    #Calculates the subgoal direction vector and position when the Shepherd is deemed to be collcting because one or more sheep have dispersed from the herd
    def cal_subgoal_behindfurthest(self):
      DirectionFromTargetToGlobalCentreOfMass = np.array([self.SheepGlobalCentreOfMass[0]-self.TargetCoordinate[0], self.SheepGlobalCentreOfMass[1]-self.TargetCoordinate[1]])   
      CS_FS_x = self.SheepMatrix[self.IndexOfFurthestSheep,0] - self.SheepGlobalCentreOfMass[0] # CS to FS x
      CS_FS_y = self.SheepMatrix[self.IndexOfFurthestSheep,1] - self.SheepGlobalCentreOfMass[1] # CS to FS y
  
      DirectionFromGlobalCentreOfMassToFurthestSheep = np.array([CS_FS_x, CS_FS_y])
      NormOfDirectionFromGlobalCentreOfMassToFurthestSheep = np.sqrt(DirectionFromGlobalCentreOfMassToFurthestSheep[0]**2 + DirectionFromGlobalCentreOfMassToFurthestSheep[1]**2)
      NormalisedDirectionFromGlobalCentreOfMassToFurthestSheep = DirectionFromGlobalCentreOfMassToFurthestSheep / NormOfDirectionFromGlobalCentreOfMassToFurthestSheep
      DirectionFromGlobalCentreOfMassToPositionBehindFurthestSheep = NormalisedDirectionFromGlobalCentreOfMassToFurthestSheep*(NormOfDirectionFromGlobalCentreOfMassToFurthestSheep + self.SheepSensingOfShepherdRadius-0.25)#-0.25 works good
      PositionBehindFurthestSheep = DirectionFromTargetToGlobalCentreOfMass + DirectionFromGlobalCentreOfMassToPositionBehindFurthestSheep
      #print "collecting"
      return PositionBehindFurthestSheep

    #This function is used to plot the simulation in real-time
    def view(self,RobotPosition,subgoal,driving,ShepherdCurrPos):
      # Plotting---------------------------------------------------------
       self.SheepGlobalCentreOfMass = np.array([np.mean(self.SheepMatrix[:,0]),np.mean(self.SheepMatrix[:,1])])  # GCM of sheep objects
       self.ShepherdGlobalCentreOfMass = np.array([np.mean(self.ShepherdMatrix[:,0]),np.mean(self.ShepherdMatrix[:,1])]) # GCM of shepherd objects

       fHandler = plt.figure(1)
       #fHandler.OuterPosition = [80 80 800 800];
       fHandler.Color = 'white'
       fHandler.MenuBar = 'none'
       fHandler.ToolBar = 'none'      
       fHandler.NumberTitle = 'off'
       ax = fHandler.gca()
       ax.cla() # clear things for fresh plot

        # change default range 
       ax.set_xlim((-self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs-0.5, self.PaddockLength+self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs+0.5))
       ax.set_ylim((-self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs-0.5, self.PaddockLength+self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs+0.5))
       #plt.hold(True)
       ax.plot(self.SheepMatrix[:,0],self.SheepMatrix[:,1],'k.',markersize=10) # plot sheeps 
       ax.plot(self.ShepherdMatrix[:,0],self.ShepherdMatrix[:,1],'b*',markersize=10)# plot shepherd
       ax.plot(self.SheepGlobalCentreOfMass[0],self.SheepGlobalCentreOfMass[1],'ro',markersize=10); # plot GCM of sheep
       #NaoGlobalx =RobotPosition[0,0]
       #NaoGlobaly =RobotPosition[1,0]
       #ax.plot(NaoGlobalx,NaoGlobaly,'r*',markersize=10); # plot NAO Pos
       #Plotting the subgoal for testing
       subgoalx =subgoal[0]
       subgoaly =subgoal[1]
       ax.plot(subgoalx,subgoaly,'bs',markersize=8)
       #plt.plot(ShepherdGlobalCentreOfMass(1,1),ShepherdGlobalCentreOfMass(1,2),'rd','markersize',10); % plot GCM of shepherds
       ax.plot(self.TargetCoordinate[0],self.TargetCoordinate[1],'gp',markersize=10) # plot target point
       
       circle=plt.Circle([self.TargetCoordinate[0],self.TargetCoordinate[1]],self.StopWhenSheepGlobalCentreOfMassDistanceToTargetIs,color='r')
       ax.add_artist(circle)
       if driving==1:
        circle2 = plt.Circle((self.SheepGlobalCentreOfMass[0],self.SheepGlobalCentreOfMass[1]), (self.SheepSensingOfShepherdRadius), color='b', fill=False)
        ax.add_artist(circle2)
       else:
         innerDrivingCircle = plt.Circle((self.SheepMatrix[self.IndexOfFurthestSheep,0], self.SheepMatrix[self.IndexOfFurthestSheep,1]), (self.SheepSensingOfShepherdRadius), color='m', fill=False)
         ax.add_artist(innerDrivingCircle)
       circle3 = plt.Circle((self.SheepGlobalCentreOfMass[0],self.SheepGlobalCentreOfMass[1]), (self.sheepDog_t1_2LCM_MinRadius), color='r', fill=False)
       ax.add_artist(circle3)
       drivingCircle = plt.Circle((self.SheepGlobalCentreOfMass[0],self.SheepGlobalCentreOfMass[1]), self.MaximumSheepDistanceToGlobalCentreOfMass, color='g', fill=False)
       ax.add_artist(drivingCircle)
       
       ax.plot([0,0],[0,self.PaddockLength],'b-')
       
       ax.plot([0,self.PaddockLength],[0,0],'b-')
       ax.plot([self.PaddockLength,0],[self.PaddockLength,self.PaddockLength],'b-')
       ax.plot([self.PaddockLength,self.PaddockLength],[self.PaddockLength,0],'b-')


       #Plot Next Shepherd Step
       ShepherdCurrPosx =ShepherdCurrPos[0,0]
       ShepherdCurrPosy =ShepherdCurrPos[1,0]
       #print ShepherdCurrPos
       ax.plot(ShepherdCurrPosx,ShepherdCurrPosy,'r*',markersize=10)
    ## Visualise Attraction Repulsion Directions
       #ax.quiver(self.SheepMatrix[:,0],self.SheepMatrix[:,1],self.CMAL_ALL[:,0],self.CMAL_ALL[:,1],color = 'r')# Direction of repulsion from other sheep
       ax.quiver(self.SheepMatrix[:,0],self.SheepMatrix[:,1],self.CMAL_ALL[:,2],self.CMAL_ALL[:,3],color = 'b')# Direction of repulsion from shepherds
       #ax.quiver(self.SheepMatrix[:,0],self.SheepMatrix[:,1],self.CMAL_ALL[:,4],self.CMAL_ALL[:,5], color = 'g')# Direction of attraction to other sheep.
       plt.xlabel('Paddock Length')
       plt.ylabel('Paddock Height')
       plt.pause(0.05)
       plt.show(block=False)
       #Sys.sleep(0.05)
      #dev.off()



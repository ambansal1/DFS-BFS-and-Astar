import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):
        successors = []
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        #import IPython
        #IPython.embed()
        for i in range(len(coord)):
            if(coord[i] > 0):
                tempcoord = coord[:]
                tempcoord[i] = coord[i] - 1
                position = self.discrete_env.GridCoordToConfiguration(tempcoord)
                if(self.CheckCollision(position)):
                        successors.append(self.discrete_env.GridCoordToNodeId(tempcoord))
            if(coord[i] < self.discrete_env.num_cells[i]-1):
                tempcoord = coord[:]
                tempcoord[i] = coord[i] + 1
                position = self.discrete_env.GridCoordToConfiguration(tempcoord)
                if(self.CheckCollision(position)):
                        successors.append(self.discrete_env.GridCoordToNodeId(tempcoord))

        return successors

    def CheckCollision(self, conf):
      
        for i in range(len(conf)):
            if conf[i] > self.upper_limits[i] or conf[i] < self.lower_limits[i]:
                return False
        self.robot.SetActiveDOFValues(conf)
        for body in self.robot.GetEnv().GetBodies():
            if (body.GetName() != self.robot.GetName() and self.robot.GetEnv().CheckCollision(body, self.robot)):
                return False
        return True

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start = self.discrete_env.NodeIdToGridCoord(start_id)
        end = self.discrete_env.NodeIdToGridCoord(end_id)

        for i in range(len(end)):
            dist = dist + abs(start[i] - end[i])
       
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        return self.ComputeDistance(start_id, goal_id) #manhatten distance yall



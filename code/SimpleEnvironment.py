import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        #self.lower_limits = [0., 0.]
        #self.upper_limits = [1., 1.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):
        successors = []
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        

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
        transform = self.robot.GetTransform()
        transform[0, 3] = conf[0]
        transform[1, 3] = conf[1]
        self.robot.SetTransform(transform);

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

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
    def PlotPlan(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'r.-', linewidth=2.5)
        pl.draw()

        

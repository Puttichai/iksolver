import numpy as np
import openravepy as orpy
from RobustIKSolver import RobustIKSolver6D


class CartesianStraightLineTracker(object):
  
    def __init__(self, robot, manipulatorname):
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        
        self._robustiksolver = RobustIKSolver6D(robot, manipulatorname)

        self._printextrainfo = True


    def track(self, Tinit, direction, length, trackersteplength=0.0001, qinit=None):
        """
        Track a straight line in Cartesian space (3D).

        Tinit -- the initial transforamtion of the manipulator. Along
                 the path, the manipulator will stay pointing along 
                 Tinit[0:3, 2].
        direction -- a 3D-vector of workspace direction. The path is 
                     therefore starting from Tinit[0:3, 3] and ending at
                     Tinit[0:3, 3] + length*direction.
        length -- the length to go along direction
        trackersteplength -- a step size (in m.) for the tracker
        qinit -- an initial ik solution corresponding to Tinit.

        Note: Tinit[0:3, 2] and direction may be different from each other.

        Return a list of status and waypointslist.
        """
        functionname = '[CartesianStraightLineTracker]'

        if qinit is None:
            qinit = self._robustiksolver.FindIKSolution(Tinit)
            if qinit is None:
                message = ' failed to find an initial IK solution'
                print  functionname + message
                return [False, []]

        nsteps = int(length/trackersteplength)
        assert(nsteps > 0) # check soundness

        direction = direction / np.linalg.norm(direction)
        M = np.array(Tinit) # dummy transformation matrix
        waypointslist = [] # solution

        stepvector = trackersteplength * direction

        waypointslist.append(qinit)
        qprev = np.array(qinit)
        for i in xrange(nsteps):
            M[0:3, 3] += stepvector
            targetpose = orpy.poseFromMatrix(M)
            result = self._robustiksolver.diffiksolver.solve(targetpose, qprev,
                                                             dt=1.0, conv_tol=1e-8)
            if not result[0]:
                message = ' failed to track the path at step {0}'.format(i + 1)
                print functionname + message
                
                if self._printextrainfo:
                    print 'qprev = np.' + repr(qprev)
                    print 'Ttarget = np.' + repr(M)                    
                
                return [False, []]

            waypointslist.append(result[2])
            qprev = np.array(result[2])

        return [True, waypointslist]


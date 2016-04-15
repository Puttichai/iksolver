import numpy as np
import openravepy as orpy
import logging
from RobustIKSolver import RobustIKSolver6D


class CartesianStraightLineTracker(object):
  
    def __init__(self, robot, manipulatorname, loglevel=10):
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        
        self._robustiksolver = RobustIKSolver6D(robot, manipulatorname)

        # Python logging
        self.logger = logging.getLogger(__name__)
        self.loglevel = loglevel
        FORMAT = "[%(module)s::%(funcName)s] %(message)s"
        logging.basicConfig(format=FORMAT)
        self.logger.setLevel(self.loglevel)

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
        if qinit is None:
            qinit = self._robustiksolver.FindIKSolution(Tinit)
            if qinit is None:
                message = "Failed to find an initial IK solution"
                self.logger.info(message)
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
            
            [reached, _, qsol] = self._robustiksolver.diffiksolver.solve\
            (targetpose, qprev, dt=1.0, max_it=100, conv_tol=1e-8)
            
            if not reached:
                message = 'Failed to track the path at step {0}'.format(i + 1)
                self.logger.info(message)
                
                if self._printextrainfo:
                    print 'qprev = np.' + repr(qprev)
                    print 'Ttarget = np.' + repr(M)                    
                
                return [False, []]

            waypointslist.append(qsol)
            qprev = np.array(qsol)

        if not np.allclose(M[0:3, 3], Tinit[0:3, 3] + length*direction):
            M[0:3, 3] = Tinit[0:3, 3] + length*direction
            targetpose = orpy.poseFromMatrix(M)
            
            [reached, _, qsol] = self._robustiksolver.diffiksolver.solve\
            (targetpose, qprev, dt=1.0, max_it=100, conv_tol=1e-8)
            
            if not reached:
                message = 'Failed to track the path at the last step'
                self.logger.info(message)
                
                if self._printextrainfo:
                    print 'qprev = np.' + repr(qprev)
                    print 'Ttarget = np.' + repr(M)                    
                
                return [False, []]

            waypointslist.append(qsol)
            qprev = np.array(qsol)

        return [True, waypointslist]


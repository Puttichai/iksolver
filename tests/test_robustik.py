import unittest
import random
import time
import numpy as np
from EnvironmentSetup import OpenRAVEsetup
from openravepy import matrixFromPose

from iksolver.RobustIKSolver import RobustIKSolver5D


rng = random.SystemRandom()


class Test_RobustIK5D(OpenRAVEsetup):
    
    def setUp(self):
        robotfile = '/robots/denso_all_withbase.robot.xml'
        manipulatorname = 'drill'
        super(Test_RobustIK5D, self).setUp(robotfile=robotfile, 
                                           manipulatorname=manipulatorname)

        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.HOME = np.zeros(6)
        
        self.points = []
        self.directions = []
        
        self.nsuccess = 0
        self.total_time = 0.0


    def tearDown(self):
        super(Test_RobustIK5D, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.nsuccess, self.nqueries)
        if self.nsuccess > 0:
            print "Average running time = {0}".format(self.total_time/self.nsuccess)


    def test_robustik5d_case_1(self):
        """
        Easy test. Robust IK Solver 5D.
        """
        X = np.arange(0.25, 0.51, 0.01)
        direction = np.array([0.0, 0.0, -1.0])
        for x in X:
            point = np.array([x, -0.25, 0.0001])
            self.points.append(point)
            self.directions.append(direction)
            
        for x in X:
            point = np.array([x, 0.00, 0.0001])
            self.points.append(point)
            self.directions.append(direction)

        for x in X:
            point = np.array([x, 0.25, 0.0001])
            self.points.append(point)
            self.directions.append(direction)

        self.nqueries = len(self.points)

        import sys
        print ''
        count = 0
        for (point, direction) in zip(self.points, self.directions):
            count += 1
            sys.stdout.write('|')
            sys.stdout.flush()
            if np.mod(count, 10) == 0:
                print ''
            # print 'point = {0}'.format(point)
            iksolver = RobustIKSolver5D(self.robot, self.manip.GetName(), ntrials=10000)
            ts = time.time()
            sol = iksolver.FindIKSolution(point, direction)
            te = time.time()

            if sol is not None:
                self.nsuccess += 1
                self.total_time += te - ts

                with self.robot:
                    self.robot.SetActiveDOFValues(sol)
                    Tactual = self.manip.GetTransform()
                    
                np.testing.assert_allclose(point, Tactual[0:3, 3], 
                                           rtol=1e-5, atol=1e-5)
                np.testing.assert_allclose(direction, Tactual[0:3, 2], 
                                           rtol=1e-5, atol=1e-5)
        print ''

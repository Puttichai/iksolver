import unittest
import random
import time
import numpy as np
from EnvironmentSetup import OpenRAVEsetup
from openravepy import matrixFromPose

from iksolver.RobustIKSolver import RobustIKSolver6D
from iksolver.CartesianStraightLineTracker import CartesianStraightLineTracker

rng = random.SystemRandom()


class Test_CartTracker(OpenRAVEsetup):

    def setUp(self):
        super(Test_CartTracker, self).setUp()        
        self.q_min = self.robot.GetActiveDOFLimits()[0] * 0.8
        self.q_max = self.robot.GetActiveDOFLimits()[1] * 0.8

        self.total_time = 0.0
        self.no_success = 0
        self.total_iter = 0
        self.no_total = 100

        self.robustiksolver = RobustIKSolver6D(self.robot, self.manip.GetName())

        # Construct random test case
        self.qinits = []
        self.Tinits = []
        self.Tfinals = []
        self.directions = []
        ndof = self.robot.GetActiveDOF()
        self.length = 0.03 # track a stright line of length 0.1 m.
        i = 0
        print "\nStart generating random test cases"
        while i < self.no_total:
            # Sample an initial configuration
            qinit_passed = False
            while not qinit_passed:
                qinit = []
                for j in xrange(ndof):
                    qinit.append(rng.random()*(self.q_max[j] - self.q_min[j]) + 
                                 self.q_min[j])
                with self.robot:
                    self.robot.SetActiveDOFValues(qinit)
                    Tinit = self.manip.GetTransform()
                    p = Tinit[0:3, 3]
                    if (np.linalg.norm(p) > 0.8) or (p[2] < 0):
                        # Limit qinit not to be too far
                        continue
                    
                    incollision = (self.env.CheckCollision(self.robot) or
                                   self.robot.CheckSelfCollision())
                    if incollision:
                        continue

                qinit_passed = True
                qinit = np.asarray(qinit)

            # Sample a direction
            direction_passed = False
            while not direction_passed:
                direction = []
                for k in xrange(3):
                    direction.append(rng.random())
                try:
                    direction = direction / np.linalg.norm(direction)
                except:
                    # direction cannot be normalized
                    continue
                direction_passed = True
            direction = np.asarray(direction)

            # Check if the end of the straight line is reachable
            Tfinal = np.array(Tinit)
            Tfinal[0:3, 3] += self.length * direction
            qfinal = self.robustiksolver.FindIKSolution(Tfinal, qref=qinit)
            if qfinal is None:
                continue

            delta = qfinal - qinit
            if np.dot(delta, delta) > self.length*20:
                continue

            # Now everything is fine
            self.qinits.append(qinit)
            self.Tinits.append(Tinit)
            self.directions.append(direction)
            self.Tfinals.append(Tfinal)
            i += 1

        print "Finish generating random test cases"


    def tearDown(self):
        super(Test_CartTracker, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)

            
    def test_carttracker_case_1(self):
        """
        Easy test. Cartesian straight line-tracker\n
        """
        carttracker = CartesianStraightLineTracker(self.robot, self.manip.GetName())
        carttracker._printextrainfo = False
        for (qinit, Tinit, direction, Tfinal) in zip(self.qinits, self.Tinits, 
                                                     self.directions, self.Tfinals):
            ts = time.time()
            [reached, wplist] = carttracker.track(Tinit, direction, 
                                                  self.length, trackersteplength=0.001,
                                                  qinit=qinit)
            te = time.time()

            if reached:
                self.total_time += te - ts
                self.no_success += 1
                
                with self.robot:
                    self.robot.SetActiveDOFValues(wplist[-1])
                    T = self.manip.GetTransform()
                np.testing.assert_allclose(T, Tfinal, rtol=1e-5, atol=1e-5)
            # else:
            #     print 'qinit = np.' + repr(qinit)
            #     print 'Tinit = np.' + repr(Tinit)
            #     print 'direction = np.' + repr(direction)
            #     print 'Tfinal = np.' + repr(Tfinal)
                    
                    

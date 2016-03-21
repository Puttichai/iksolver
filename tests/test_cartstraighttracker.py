import random
import time
import numpy as np
import openravepy as orpy
from EnvironmentSetup import OpenRAVEsetup

from iksolver.CartesianStraightLineTracker import CartesianStraightLineTracker

rng = random.SystemRandom()


class Test_CartesianStraightLineTracker(OpenRAVEsetup):

    def setUp(self):
        super(Test_CartesianStraightLineTracker, self).setUp()
        
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.no_total = 100
        self.purturbation_mag = 0.1

        self.trackersteplength = 0.0001
        self.length = 0.01

        self.carttracker = CartesianStraightLineTracker(self.robot, self.manip.GetName())

        # Construct random test case
        self.qseeds = []
        self.poses = []
        ndof = self.robot.GetActiveDOF()
        for i in xrange(self.no_total):
            incollision = True
            while incollision:
                while True:
                    qseed = []
                    for j in xrange(ndof):
                        # qseed.append(rng.random()*(self.q_max[j] - self.q_min[j]) + 
                        #              self.q_min[j])
                        qseed.append(rng.random())
                    with self.robot:
                        self.robot.SetActiveDOFValues(qseed)
                        p = self.manip.GetTransform()[0:3, 3]
                    if (np.linalg.norm(p) < 0.8) and (p[2] >= 0):
                        # Constrain the end-effector to be not too far
                        break
                    
                with self.robot:
                    self.robot.SetActiveDOFValues(qseed)
                    pose = self.manip.GetTransformPose()
                    incollision = self.env.CheckCollision(self.robot) or\
                    self.robot.CheckSelfCollision()
                    
            if pose[0] < 0:
                pose[:4] *= -1.
            self.poses.append(pose)

            # Perturb qseed from the desired position
            for k in xrange(ndof):
                qseed[j] += (rng.random()*2 - 1) * self.purturbation_mag
            qseed = np.maximum(np.minimum(qseed, self.q_max), self.q_min)
            self.qseeds.append(np.asarray(qseed))

        
    def tearDown(self):
        super(Test_CartesianStraightLineTracker, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)


    def test_cartstraighttracker_case_1(self):
        """
        Easy Test. Cartesian Straight Line Tracker
        """
        i = 0
        for (qseed, pose) in zip(self.qseeds, self.poses):
            i += 1
            T = orpy.matrixFromPose(pose)
            direction = T[0:3, 2]
            direction = direction / np.linalg.norm(direction)

            ts = time.time()
            result = self.carttracker.track(T, direction, self.length, 
                                            trackersteplength=self.trackersteplength, 
                                            qinit=None)
            te = time.time()

            if result[0]:
                self.total_time += te - ts
                self.no_success += 1

                waypointslist = result[1]
                # print 'waypointslist = {0}'.format(len(waypointslist))
                with self.robot:
                    self.robot.SetActiveDOFValues(waypointslist[-1])
                    Tmanip = self.manip.GetTransform()

                Texpected = np.array(T)
                Texpected[0:3, 3] += self.length * direction                

                # print 'pose = np.' + repr(pose)

                # Examine direction
                try:
                    np.testing.assert_allclose(Tmanip[0:3, 2], direction, 
                                               rtol=1e-5, atol=1e-5)
                except:
                    print 'Texpected = np.' + repr(Texpected)
                    print 'Tmanip = np.' + repr(Tmanip)
                # Examine position
                try:
                    np.testing.assert_allclose(Tmanip[0:3, 3], Texpected[0:3, 3], 
                                               rtol=1e-5, atol=1e-5)
                except:
                    print 'Texpected = np.' + repr(Texpected)
                    print 'Tmanip = np.' + repr(Tmanip)

                

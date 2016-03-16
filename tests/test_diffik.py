import unittest
import random
import time
import numpy as np
from EnvironmentSetup import OpenRAVEsetup

from iksolver.DiffIKSolver import DiffIKSolver
from iksolver.DiffIKSolver import DiffIKSolver5D


rng = random.SystemRandom()


class Test_DiffIksolver_Vanilla(OpenRAVEsetup):

    def setUp(self):
        super(Test_DiffIksolver_Vanilla, self).setUp()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.total_iter = 0
        self.no_total = 100

        # Construct random test case
        self.qseeds = []
        self.poses = []
        ndof = self.robot.GetActiveDOF()
        for i in xrange(self.no_total):
            qseed = []
            for j in xrange(ndof):
                qseed.append(rng.random()*(self.q_max[j] - self.q_min[j]) + self.q_min[j])
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                pose = self.manip.GetTransformPose()
            if pose[0] < 0:
                pose[:4] *= -1.
            self.poses.append(pose)

            # Perturb qseed from the desired position
            for k in xrange(ndof):
                qseed[j] += (rng.random()*2 - 1) * .2
            self.qseeds.append(np.asarray(qseed))


    def tearDown(self):
        super(Test_DiffIksolver_Vanilla, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)
            print "Average iterations: {0}".format(1.0*self.total_iter/self.no_success)


    def test_diffiksolver_vanilla_case_1(self):
        """
        Easy test. Differential IK Solver (vanilla version; IK6D)
        """
        i = 0
        for (q, pose) in zip(self.qseeds, self.poses):
            i += 1
            iksolver = DiffIKSolver(self.robot, self.manip.GetName())
            ts = time.time()
            result = iksolver.solve(pose, q, 1., conv_tol=1e-8)
            te = time.time()

            if result[0]:
                self.total_time += te - ts
                self.no_success += 1
                self.total_iter += result[1]
                
                q = result[2]
                with self.robot:
                    self.robot.SetActiveDOFValues(q)
                    pose_actual = self.manip.GetTransformPose()
                if pose_actual[0] < 0:
                    pose_actual[:4] *= -1.

                # print 'iter', i
                # print 'desired pose', pose
                # print 'actual pose', pose_actual

                np.testing.assert_allclose(pose_actual, pose, rtol=1e-5, atol=1e-5)
                self.assertTrue((q <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= q).all(), msg="Violate joint limits")


class Test_DiffIksolver_5D(OpenRAVEsetup):

    def setUp(self):
        super(Test_DiffIksolver_5D, self).setUp()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.total_iter = 0
        self.no_total = 100

        # Construct random test case
        self.qseeds = []
        self.poses = []
        ndof = self.robot.GetActiveDOF()
        for i in xrange(self.no_total):
            qseed = []
            for j in xrange(ndof):
                qseed.append(rng.random()*(self.q_max[j] - self.q_min[j]) + self.q_min[j])
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                pose = self.manip.GetTransformPose()
            if pose[0] < 0:
                pose[:4] *= -1.
            self.poses.append(pose)

            # Perturb qseed from the desired position
            for k in xrange(ndof):
                qseed[j] += (rng.random()*2 - 1) * .2
            self.qseeds.append(np.asarray(qseed))


    def tearDown(self):
        super(Test_DiffIksolver_5D, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)
            print "Average iterations: {0}".format(1.0*self.total_iter/self.no_success)


    def test_iksolver_5d_case_1(self):
        """
        Easy test. Differential IK Solver 5D
        """
        i = 0
        for (q, pose) in zip(self.qseeds, self.poses):
            i += 1
            iksolver = DiffIKSolver5D(self.robot, self.manip.GetName())
            ts = time.time()
            result = iksolver.solve(pose, q, .8*1., conv_tol=1e-8)
            te = time.time()

            if result[0]:
                self.total_time += te - ts
                self.no_success += 1
                self.total_iter += result[1]
                
                q = result[2]
                with self.robot:
                    self.robot.SetActiveDOFValues(q)
                    pose_actual = self.manip.GetTransformPose()
                if pose_actual[0] < 0:
                    pose_actual[:4] *= -1.
                    
                # Check direction
                cos_alpha_desired = pose[0]
                sin_alpha_desired = np.sqrt(1 - cos_alpha_desired**2)
                direction_desired = pose[1:4] / sin_alpha_desired

                cos_alpha_actual = pose_actual[0]
                sin_alpha_actual = np.sqrt(1 - cos_alpha_actual**2)
                direction_actual = pose_actual[1:4] / sin_alpha_actual

                print 'iter', i
                print 'desired direction', direction_desired
                print 'actual direction', direction_actual
                print 'desired position', pose[4:]
                print 'actual position', pose_actual[4:]
                print 'targetpose', pose
                print 'actualpose', pose_actual
                
                np.testing.assert_allclose(direction_desired, direction_actual, 
                                           rtol=1e-5, atol=1e-5)

                # Check position
                np.testing.assert_allclose(pose_actual[4:], pose[4:], 
                                           rtol=1e-5, atol=1e-5)
                
                print 'q', q
                self.assertTrue((q <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= q).all(), msg="Violate joint limits")


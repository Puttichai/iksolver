import unittest
import random
import time
import numpy as np
from EnvironmentSetup import OpenRAVEsetup
from openravepy import matrixFromPose

from iksolver.DiffIKSolver import DiffIKSolver


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
        # A seed configuration is created by perturbing the true
        # joints values with uniform noises in the range
        # [-perturbation_mag, +perturbation_mag].
        self.perturbation_mag = 0.1

        # Construct random test case
        self.qseeds = []
        self.poses = []
        ndof = self.robot.GetActiveDOF()
        i = 0
        while i < self.no_total:
            qsol = []
            for j in xrange(ndof):
                qsol.append(rng.random()*(self.q_max[j] - self.q_min[j]) + 
                            self.q_min[j])
            with self.robot:
                self.robot.SetActiveDOFValues(qsol)
                incollision = (self.env.CheckCollision(self.robot) or
                               self.robot.CheckSelfCollision())
                
                if incollision:
                    continue
                
                pose = self.manip.GetTransformPose()            
                i += 1
                
            if pose[0] < 0:
                pose[:4] *= -1.
            self.poses.append(pose)

            # Perturb qseed from the desired position
            incollision = True
            while incollision:
                qseed = np.array(qsol)
                for k in xrange(ndof):
                    qseed[k] += (rng.random()*2 - 1) * self.perturbation_mag
                qseed = np.maximum(np.minimum(qseed, self.q_max), self.q_min)
                
                with self.robot:
                    self.robot.SetActiveDOFValues(qseed)
                    incollision = (self.env.CheckCollision(self.robot) or
                                   self.robot.CheckSelfCollision())
                
            self.qseeds.append(qseed)


    def tearDown(self):
        super(Test_DiffIksolver_Vanilla, self).tearDown()

        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)
            print "Average iterations: {0}".format(1.0*self.total_iter/self.no_success)


    def test_diffiksolver_vanilla_case_1(self):
        """
        Easy test. Differential IK Solver (vanilla version; IK6D)\n
        """
        i = 0
        iksolver = DiffIKSolver(self.robot, self.manip.GetName())
        for (q, pose) in zip(self.qseeds, self.poses):
            i += 1            
            ts = time.time()
            [reached, it, qsol] = iksolver.solve(pose, q, dt=1., conv_tol=1e-8)
            te = time.time()

            if reached:
                self.total_time += te - ts
                self.no_success += 1
                self.total_iter += it
                
                with self.robot:
                    self.robot.SetActiveDOFValues(qsol)
                    pose_actual = self.manip.GetTransformPose()
                if pose_actual[0] < 0:
                    pose_actual[:4] *= -1.

                # print 'iter', i
                # print 'desired pose', pose
                # print 'actual pose', pose_actual

                np.testing.assert_allclose(pose_actual, pose, rtol=1e-5, atol=1e-5)
                self.assertTrue((qsol <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= qsol).all(), msg="Violate joint limits")


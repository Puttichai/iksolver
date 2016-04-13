import unittest
import random
import time
import numpy as np
from EnvironmentSetup import OpenRAVEsetup

import openravepy as orpy

ikfilter_checkcollision = orpy.IkFilterOptions.CheckEnvCollisions
ikfilter_ignorecollision = orpy.IkFilterOptions.IgnoreEndEffectorCollisions
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D
rng = random.SystemRandom()


class Test_IKFast_6D(OpenRAVEsetup):

    def setUp(self):
        super(Test_IKFast_6D, self).setUp()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.no_total = 100
        self.perturbation_mag = 0.2

        # Load IKFast
        self.ikmodel6D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype6D)
        if not self.ikmodel6D.load():
            print 'Generating IK model 6D database. This may take a while. . .'
            self.ikmodel6D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)
        
        # Construct random test cases
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
        super(Test_IKFast_6D, self).tearDown()
        
        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)

    
    def test_ikfast_6d_case_1(self):
        """
        Easy test. IKFast 6D
        """
        i = 0
        for (qseed, pose) in zip(self.qseeds, self.poses):
            i += 1
            T = orpy.matrixFromPose(pose)
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                ts = time.time()
                sol = self.manip.FindIKSolution(T, ikfilter_checkcollision)
                te = time.time()

            if sol is not None:
                self.total_time += te - ts
                self.no_success += 1
                
                with self.robot:
                    self.robot.SetActiveDOFValues(sol)
                    pose_actual = self.manip.GetTransformPose()
                if pose_actual[0] < 0:
                    pose_actual[:4] *= -1.
                    
                np.testing.assert_allclose(pose_actual, pose, rtol=1e-5, atol=1e-5)
                self.assertTrue((sol <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= sol).all(), msg="Violate joint limits")
                

class Test_IKFast_5D(OpenRAVEsetup):

    def setUp(self):
        super(Test_IKFast_5D, self).setUp()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.no_total = 100
        self.perturbation_mag = 0.2

        # Load IKFast
        self.ikmodel5D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype5D)
        if not self.ikmodel5D.load():
            print 'Generating IK model 5D database. This may take a while. . .'
            self.ikmodel5D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel5D.iksolver)
        
        # Construct random test cases
        self.qsols = []
        self.qseeds = []
        self.transformations = []
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
                
                T = self.manip.GetTransform()            
                i += 1
                
            self.qsols.append(np.asarray(qsol))
            self.transformations.append(T)

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
        super(Test_IKFast_5D, self).tearDown()
        
        # Report
        print "Successful instances: {0}/{1}".format(self.no_success, self.no_total)
        if self.no_success > 0:
            print "Average time: {0} sec.".format(self.total_time/self.no_success)

    
    def test_ikfast_5d_case_1(self):
        """
        Easy test. IKFast 5D
        """
        i = 0
        for (initsol, qseed, T) in zip(self.qsols, self.qseeds, self.transformations):
            i += 1
            point = T[0:3, 3]
            direction = T[0:3, 2] / np.linalg.norm(T[0:3, 2])
            ikparam = orpy.IkParameterization(orpy.Ray(point, direction), iktype5D)
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                ts = time.time()
                qsol = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
                te = time.time()
                
            if qsol is not None:
                self.total_time += te - ts
                self.no_success += 1
                
                with self.robot:
                    self.robot.SetActiveDOFValues(qsol)
                    Tmanip = self.manip.GetTransform()

                # Check direction
                direction_actual = Tmanip[0:3, 2] / np.linalg.norm(Tmanip[0:3, 2])

                try:
                    np.testing.assert_allclose(direction, direction_actual, 
                                               rtol=1e-5, atol=1e-5)
                except:
                    print 'initsol = np.' + repr(initsol)
                    print 'qsol = np.' + repr(qsol)

                # Check position
                point_actual = Tmanip[0:3, 3]
                np.testing.assert_allclose(point_actual, point, 
                                           rtol=1e-5, atol=1e-5)
                
                self.assertTrue((qsol <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= qsol).all(), msg="Violate joint limits")

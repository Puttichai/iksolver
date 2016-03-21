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
        self.purturbation_mag = 0.2

        # Load IKFast
        self.ikmodel6D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype6D)
        if not self.ikmodel6D.load():
            print 'Generating IK model 6D database. This may take a while. . .'
            self.ikmodel6D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)
        
        # Construct random test case
        self.qseeds = []
        self.poses = []
        ndof = self.robot.GetActiveDOF()
        for i in xrange(self.no_total):
            incollision = True
            while incollision:
                qseed = []
                for j in xrange(ndof):
                    qseed.append(rng.random()*(self.q_max[j] - self.q_min[j]) + self.q_min[j])
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
        for (q, pose) in zip(self.qseeds, self.poses):
            i += 1
            T = orpy.matrixFromPose(pose)
            with self.robot:
                self.robot.SetActiveDOFValues(q)
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
                self.assertTrue((q <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= q).all(), msg="Violate joint limits")
                
            else:
                pose_string = self._string_from_vect(pose, 'pose')
                q_string = self._string_from_vect(q, 'q')
                print pose_string
                print q_string
                print ''                


    def _string_from_vect(self, vect, name='vect'):
        resstring = name + ' = np.array(['
        separator = ''
        for v in vect:
            resstring += separator
            resstring += '{0}'.format(v)
            separator = ', '
        resstring += '])'
        return resstring


class Test_IKFast_5D(OpenRAVEsetup):

    def setUp(self):
        super(Test_IKFast_5D, self).setUp()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]

        self.total_time = 0.0
        self.no_success = 0
        self.no_total = 100
        self.purturbation_mag = 0.2

        # Load IKFast
        self.ikmodel5D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype5D)
        if not self.ikmodel5D.load():
            print 'Generating IK model 5D database. This may take a while. . .'
            self.ikmodel5D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel5D.iksolver)
        
        # Construct random test case
        self.qseeds = []
        self.mats = []
        ndof = self.robot.GetActiveDOF()
        for i in xrange(self.no_total):
            incollision = True
            while incollision:
                qseed = []
                for j in xrange(ndof):
                    qseed.append(rng.random()*(self.q_max[j] - self.q_min[j]) + 
                                 self.q_min[j])
                with self.robot:
                    self.robot.SetActiveDOFValues(qseed)
                    T = self.manip.GetTransform()
                    incollision = self.env.CheckCollision(self.robot) or\
                    self.robot.CheckSelfCollision()
                    
            self.mats.append(T)

            # Perturb qseed from the desired position
            for k in xrange(ndof):
                qseed[j] += (rng.random()*2 - 1) * self.purturbation_mag
            qseed = np.maximum(np.minimum(qseed, self.q_max), self.q_min)
            self.qseeds.append(np.asarray(qseed))


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
        for (q, T) in zip(self.qseeds, self.mats):
            i += 1
            point = T[0:3, 3]
            direction = T[0:3, 2] / np.linalg.norm(T[0:3, 2])
            ikparam = orpy.IkParameterization(orpy.Ray(point, direction), iktype5D)
            with self.robot:
                self.robot.SetActiveDOFValues(q)
                ts = time.time()
                sol = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
                te = time.time()

            if sol is not None:
                self.total_time += te - ts
                self.no_success += 1
                
                with self.robot:
                    self.robot.SetActiveDOFValues(sol)
                    Tmanip = self.manip.GetTransform()

                q_string = self._string_from_vector(q, 'q')
                print q_string
                Tmanip_string = self._string_from_matrix(Tmanip, 'Tmanip')
                print Tmanip_string
                    
                # Check direction
                direction_actual = Tmanip[0:3, 2]

                np.testing.assert_allclose(direction, direction_actual, 
                                           rtol=1e-5, atol=1e-5)

                # Check position
                point_actual = T[0:3, 3]
                np.testing.assert_allclose(point_actual[4:], point[4:], 
                                           rtol=1e-5, atol=1e-5)
                
                self.assertTrue((q <= self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min <= q).all(), msg="Violate joint limits")

            else:
                q_string = self._string_from_vector(q, 'q')
                print pose_string
                print q_string
                print ''


    def _string_from_vector(self, vect, name='vect'):
        resstring = name + ' = np.array(['
        separator = ''
        for v in vect:
            resstring += separator
            resstring += '{0}'.format(v)
            separator = ', '
        resstring += '])'
        return resstring

    def _string_from_matrix(self, mat, name='mat'):
        resstring = name + ' = np.array(['
        offset = len(resstring)

        shape = mat.shape
        
        rows = []
        for i in xrange(shape[0]):
            row_string = '['
            separator = ''
            for j in xrange(shape[1]):
                row_string += separator
                row_string += '{0}'.format(mat[i, j])
                separator = ', '
            row_string += ']'
            rows.append(row_string)

        separator = ''
        for string in rows:
            resstring += separator
            resstring += string
            separator = ',\n' + ' '*offset

        resstring += '])'
        return resstring
        

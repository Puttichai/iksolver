import unittest
from numpy import testing

import numpy as np
from iksolver.sqp_ik import *
from EnvironmentSetup import OpenRAVEsetup


class Test_Iksolver_ik6D(OpenRAVEsetup):

    def setUp(self):
        super(Test_Iksolver_ik6D, self).setUp()
        # Test parameters
        self.q_max = self.robot.GetDOFLimits()[1]
        self.q_min = self.robot.GetDOFLimits()[0]
        self.total_time = 0.0
        self.no_success = 0
        self.total_iter = 0
        self.no_total = 10

        # Construct random test case
        self.qseeds = []
        self.poses = []
        for i in range(self.no_total):
            qseed = np.random.rand(6)
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                pose_desired = self.manip.GetTransformPose()

            qseed += np.random.rand(6) * 0.1
            self.qseeds.append(qseed)
            self.poses.append(pose_desired)

    def tearDown(self):
        super(Test_Iksolver_ik6D, self).tearDown()
        # Report data
        print "Success %d times in a total of %d" % (self.no_success, self.no_total)
        if self.no_success != 0:
            print "Average time %0.5f seconds" % (self.total_time / self.no_success)
            print "Average iteration %0.2f" % (self.total_iter / self.no_success)

    def test_ik6D_case_1(self):
        'Easy test. Formulating both as constraints'

        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik6D',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':['constraint', 'constraint']}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.total_iter += result['noiter']
                self.robot.SetActiveDOFValues(q)
                pose_actual = self.manip.GetTransformPose()

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")

class Test_Iksolver_ik3DRotation(OpenRAVEsetup):

    def setUp(self):
        super(Test_Iksolver_ik3DRotation, self).setUp()
        # Test parameters
        self.q_max = self.robot.GetDOFLimits()[1]
        self.q_min = self.robot.GetDOFLimits()[0]
        self.total_time = 0.0
        self.no_success = 0
        self.total_iter = 0
        self.no_total = 10

        # Construct random test case
        self.qseeds = []
        self.poses = []
        for i in range(self.no_total):
            qseed = np.random.rand(6)
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                pose_desired = self.manip.GetTransformPose()[0:4]

            qseed += np.random.rand(6) * 0.4
            self.qseeds.append(qseed)
            self.poses.append(pose_desired)
    def tearDown(self):
        super(Test_Iksolver_ik3DRotation, self).tearDown()
        # Report data
        print "Success %d times in a total of %d" % (self.no_success, self.no_total)
        if self.no_success != 0:
            print "Average time %0.5f seconds" % (self.total_time / self.no_success)
            print "Average iteration %0.2f" % (self.total_iter / self.no_success)

    def test_3DRotation_as_constraint_case_1(self):
        'Easy test. Formulating 3DRotation as constraint'
        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik3DRotation',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':'constraint'}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.total_iter += result['noiter']
                self.robot.SetActiveDOFValues(q)
                pose_actual = self.manip.GetTransformPose()[:4]

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")

    def test_3DRotation_as_objective_case_1(self):
        'Easy test. Formulating 3DRotation as objective'
        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik3DRotation',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':'objective'}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.total_iter += result['noiter']
                self.robot.SetActiveDOFValues(q)
                pose_actual = self.manip.GetTransformPose()[:4]

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")




class Test_Iksolver_ik3DTranslation(OpenRAVEsetup):

    def setUp(self):
        super(Test_Iksolver_ik3DTranslation, self).setUp()
        self.q_max = self.robot.GetDOFLimits()[1]
        self.q_min = self.robot.GetDOFLimits()[0]
        self.total_time = 0.0
        self.no_success = 0.0
        self.total_iter = 0.0
        self.no_total = 50

        # Construct random test case
        self.qseeds = []
        self.poses = []
        for i in range(self.no_total):
            qseed = np.random.rand(6)
            with self.robot:
                self.robot.SetActiveDOFValues(qseed)
                pose_desired = self.manip.GetTransformPose()[4:]

            qseed += np.random.rand(6) * 0.5
            self.qseeds.append(qseed)
            self.poses.append(pose_desired)

    def tearDown(self):
        super(Test_Iksolver_ik3DTranslation, self).tearDown()

        # Report data
        print "Success %d times in a total of %d" % (self.no_success, self.no_total)
        if self.no_success != 0:
            print "Average time %0.5f seconds" % (self.total_time / self.no_success)
            print "Average iteration %0.2f" % (self.total_iter / self.no_success)



    def test_3Dtranslation_Iksolver_as_constraint_case_1(self):
        'Easy test. Formulating position constraint as constraint'

        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik3DTranslation',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':'constraint'}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.total_iter += result['noiter']
                self.robot.SetActiveDOFValues(q)
                pose_actual = self.manip.GetTransformPose()[4:]

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")

    def test_3Dtranslation_Iksolver_as_objective_case_1(self):
        'Easy test. Formulating position constraint as an objective'
        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik3DTranslation',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':'objective'}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.robot.SetActiveDOFValues(q)
                self.total_iter += result['noiter']
                pose_actual = self.manip.GetTransformPose()[4:]

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")

    def test_3Dtranslation_Iksolver_hybrid_case_1(self):
        'Easy test. Formulating position constraint hybrid'
        for i in range(self.no_total):
            qseed = self.qseeds[i]
            pose_desired = self.poses[i]
            param = {'type':'ik3DTranslation',
                     'data':pose_desired,
                     'seed':qseed,
                     'formulation':'hybrid'}

            iksolver = IKsqp(robot=self.robot, manip=self.manip)

            result = iksolver.find_solution(param)
            q = result['q']
            conv = result['converged']
            if conv:
                self.total_time += result['duration']
                self.no_success += 1
                self.robot.SetActiveDOFValues(q)
                self.total_iter += result['noiter']
                pose_actual = self.manip.GetTransformPose()[4:]

                testing.assert_allclose(pose_actual, pose_desired, rtol=1e-5, atol=1e-5)
                self.assertTrue((q < self.q_max).all(), msg="Violate joint limits")
                self.assertTrue((self.q_min < q).all(), msg="Violate joint limits")


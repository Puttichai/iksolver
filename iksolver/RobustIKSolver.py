import numpy as np
import openravepy as orpy
import random
from DiffIKSolver import DiffIKSolver

ikfilter_checkcollision = orpy.IkFilterOptions.CheckEnvCollisions
ikfilter_ignorecollision = orpy.IkFilterOptions.IgnoreEndEffectorCollisions
iktype6D = orpy.IkParameterization.Type.Transform6D
rng = random.SystemRandom()


class RobustIKSolver6D(object):

    def __init__(self, robot, manipulatorname, qd_lim=1):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        self.env = self.robot.GetEnv()
        
        # Load IKFast
        self.ikmodel6D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype6D)
        if not self.ikmodel6D.load():
            print 'Generating IK model 6D database for {0}." This may take a while. . .'.\
            format(self.robot.GetName())
            self.ikmodel6D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)

        # Initialize a differential IK solver
        self.diffiksolver = DiffIKSolver(robot, manipulatorname)

        # parameters
        self._ntrials = 20


    def FindIKSolution(self, T):
        # Use IKFast
        qsol = self.manip.FindIKSolution(T, ikfilter_checkcollision)
        if qsol is not None:
            return qsol

        for i in xrange(self._ntrials):
            Tnew = PerturbT(T)
            q = self.manip.FindIKSolution(Tnew, ikfilter_checkcollision)
            targetpose = orpy.poseFromMatrix(Tnew)
            result = self.diffiksolver.solve(targetpose, q, dt=1.0, conv_tol=1e-8)
            
            if not result[0]:
                continue
        
            qsol = reult[1]
            with self.env:
                self.robot.SetActiveDOFValues(qsol)
                incollision = self.env.CheckCollision(self.robot) or\
                self.robot.CheckSelfCollision()
            if incollision:
                continue
            
            return qsol

        print '[RobustIKSolver::FindIKSolution] failed to find an IK solution after' +\
        '{0} trials'.format(self._ntrials) 
        
        return None


################################################################################
# SE(3) Utilities
################################################################################
def expmat(r):
    """
    expmat is taken from TOPP-SO3 package
    (https://github.com/dinhhuy2109/TOPP-SO3).

    Compute an exponential map of a 3D-vector.
    """
    nr = np.linalg.norm(r)
    if(nr <= 1e-10):
        return np.eye(3)
    R = skewfromvect(r)
    return np.eye(3) + np.sin(nr)/nr*R + (1 - np.cos(nr))/(nr*nr)*np.dot(R, R)


def skewfromvect(r):
    """
    skewfromvect is taken from TOPP-SO3 package
    (https://github.com/dinhhuy2109/TOPP-SO3).

    Return the skew symmetric matrix constructed from r.
    """
    return np.array([[    0., -r[2],  r[1]],
                     [  r[2],     0, -r[0]],
                     [ -r[1],  r[0],    0.]])


def PerturbT(T, noise_mag=0.05):
    rotation_noise = [(2*rng.random() - 1)*noise_mag for _ in xrange(3)]
    translation_noise = [(2*rng.random() - 1)*noise_mag for _ in xrange(3)]

    R = np.array(T[0:3, 0:3])
    p = np.array(T[0:3, 3])

    # Add noise to rotation
    R = np.dot(R, expmat(rotation_noise))
    # Add noise to translation
    translation_noise = np.asarray(translation_noise)
    p = p + translation_noise
    p = p.reshape((3, 1))
    
    Tnew = np.vstack([np.hstack([R, p]), np.array([0., 0., 0., 1.])])
    return Tnew

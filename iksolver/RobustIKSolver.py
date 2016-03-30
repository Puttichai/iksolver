import numpy as np
import openravepy as orpy
import random
from DiffIKSolver import DiffIKSolver

ikfilter_checkcollision = orpy.IkFilterOptions.CheckEnvCollisions
ikfilter_ignorecollision = orpy.IkFilterOptions.IgnoreEndEffectorCollisions
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D
rng = random.SystemRandom()


class RobustIKSolver6D(object):

    def __init__(self, robot, manipulatorname, ntrials=1000, qd_lim=1):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        self.env = self.robot.GetEnv()
        
        # Load IKFast
        self.ikmodel6D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype6D)
        if not self.ikmodel6D.load():
            print 'Generating IK model 6D database for {0}. This may take a while...'.\
            format(self.robot.GetName())
            self.ikmodel6D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)

        # Initialize a differential IK solver
        self.diffiksolver = DiffIKSolver(robot, manipulatorname)
        self.diffiksolver._print = False
        
        # parameters
        self._ntrials = ntrials


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
        
            qsol = result[1]
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


class RobustIKSolver5D(object):

    def __init__(self, robot, manipulatorname, ntrials=1000, qd_lim=1):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        self.env = self.robot.GetEnv()

        # Load IKFast
        self.ikmodel5D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype5D)
        if not self.ikmodel5D.load():
            print 'Generating IK model 5D database for {0}. This may take a while...'.\
            format(self.robot.GetName())
            self.ikmodel5D.autogenerate()
        self.manip.SetIkSolver(self.ikmodel5D.iksolver)

        # Initialize a differential IK solver
        self.diffiksolver = DiffIKSolver(robot, manipulatorname)
        self.diffiksolver._print = False

        # parameters
        self._ntrials = ntrials


    def FindIKSolution(self, point, direction):
        """
        point -- a 3D vector
        direction -- a 3D 'unit' vector
        """
        # Use IKFast
        ikparam = orpy.IkParameterization(orpy.Ray(point, direction), iktype5D)
        qsol = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
        if qsol is not None:
            return qsol

        # Compute an initial rotation
        z = np.array(direction).reshape((3, 1))
        x = PerpendicularTo(z).reshape((3, 1))
        y = np.cross(z.T, x.T).reshape((3, 1))
        R = np.hstack([x, y, z])
        
        for i in xrange(self._ntrials):
            # Sample a rotation
            theta = (2.0*rng.random() - 1.0) * np.pi
            cos = np.cos(theta)
            sin = np.sin(theta)
            Rtheta = np.array([[ cos, -sin,  0.0],
                               [ sin,  cos,  0.0],
                               [ 0.0,  0.0,  1.0]])
            Rnew = np.dot(R, Rtheta)
            T = CombineRotationTranslation(Rnew, point) # desired transformation
            targetpose = orpy.poseFromMatrix(T)  # desired pose

            # Add some perturbation
            Tnew = PerturbT(T)

            # Let IKFast provide an initial solution
            ray = orpy.Ray(Tnew[0:3, 3], Tnew[0:3, 2]/np.linalg.norm(Tnew[0:3, 2]))
            ikparam = orpy.IkParameterization(ray, iktype5D)
            qinit = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
            if qinit is None:
                # print 'iteration {0}: IKFast still failed'.format(i)
                continue
        
            result = self.diffiksolver.solve(targetpose, qinit, dt=1.0, 
                                             max_it=10.0, conv_tol=1e-8)

            if not result[0]:
                # print 'iteration {0}: DiffIKSolver failed'.format(i)
                continue

            qsol = result[-1]
            with self.env:
                self.robot.SetActiveDOFValues(qsol)
                incollision = self.env.CheckCollision(self.robot) or\
                self.robot.CheckSelfCollision()
            if incollision:
                continue
            
            return qsol
        
        print '[RobustIKSolver5D::FindIKSolution] failed to find an IK solution after' +\
        ' {0} trials'.format(self._ntrials)

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


def CombineRotationTranslation(R, p):
    if not (p.shape == (3, 1)):
        p = p.reshape((3, 1))
        
    T = np.vstack([np.hstack([R, p]), np.array([0., 0., 0., 1.])])
    return T


def PerturbT(T, noise_mag=0.0001):
    """
    Perturb a transformation T by uniform noises of magnitude in the
    range [-noise_mag, noise_mag].
    """
    _perturbtranslation = False
    # It seems like when IKFast fails but an ik solution exists, it is
    # probably because some axis alignment issue. Normally, preturbing
    # only the roational part should already work.
    # Anyway, we can enable this any time if we find it useful.

    rotation_noise = [(2*rng.random() - 1)*noise_mag for _ in xrange(3)]
    if _perturbtranslation:
        translation_noise = [(2*rng.random() - 1)*noise_mag for _ in xrange(3)]
    else:
        translation_noise = [0.0, 0.0, 0.0]

    R = np.array(T[0:3, 0:3])
    p = np.array(T[0:3, 3])

    # Add noise to rotation
    R = np.dot(R, expmat(rotation_noise))
    # Add noise to translation
    translation_noise = np.asarray(translation_noise)
    p = p + translation_noise

    return CombineRotationTranslation(R, p)


def PerpendicularTo(v):
    """ Finds an arbitrary perpendicular vector to *v*."""
    # for two vectors (x, y, z) and (a, b, c) to be perpendicular,
    # the following equation has to be fulfilled
    #     0 = ax + by + cz
    if (not (len(v) == 3)):
        raise ValueError('dimension not compatible')    
    
    # x = y = z = 0 is not an acceptable solution
    if v[0] == v[1] == v[2] == 0:
        raise ValueError('zero-vector')

    # If one dimension is zero, this can be solved by setting that to
    # non-zero and the others to zero. Example: (4, 2, 0) lies in the
    # x-y-Plane, so (0, 0, 1) is orthogonal to the plane.
    if v[0] == 0:
        return np.array([1., 0., 0.])
    if v[1] == 0:
        return np.array([0., 1., 0.])
    if v[2] == 0:
        return np.array([0., 0., 1.])

    # arbitrarily set a = b = 1
    # then the equation simplifies to
    #     c = -(x + y)/z
    c = -(v[0] + v[1])/float(v[2])
    d = 1./np.sqrt(2 + abs(c)**2)
    return np.array([d, d, d*c])

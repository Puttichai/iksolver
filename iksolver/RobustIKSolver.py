import numpy as np
import openravepy as orpy
import random
import logging
from DiffIKSolver import DiffIKSolver

ikfilter_checkcollision = orpy.IkFilterOptions.CheckEnvCollisions
ikfilter_ignorecollision = orpy.IkFilterOptions.IgnoreEndEffectorCollisions
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D
rng = random.SystemRandom()


class RobustIKSolver6D(object):

    def __init__(self, robot, manipulatorname, ntrials=1000, qd_lim=1, loglevel=10):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        self.env = self.robot.GetEnv()
        
        # Load IKFast
        self.ikmodel6D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype6D)
        if not self.ikmodel6D.load():
            msg = 'IK model 6D database for manipulator {0} not found.'
            msg += ' Please generate it first.'
            raise Exception(msg)
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)

        # Initialize a differential IK solver
        self.diffiksolver = DiffIKSolver(self.manip, loglevel=40)
        
        # parameters
        self._ntrials = ntrials

        # Python logging
        self.logger = logging.getLogger(__name__)
        self.loglevel = loglevel
        FORMAT = "[%(module)s::%(funcName)s] %(message)s"
        logging.basicConfig(format=FORMAT)
        self.logger.setLevel(self.loglevel)


    def ActivateIKSolver(self):
        self.manip.SetIkSolver(self.ikmodel6D.iksolver)


    def FindIKSolution(self, T, qref=None, checkcollision=True):
        self.ActivateIKSolver()
        # Use IKFast
        with self.robot:
            if qref is not None:
                self.robot.SetActiveDOFValues(qref)
            if checkcollision:
                qsol = self.manip.FindIKSolution(T, ikfilter_checkcollision)
            else:
                qsol = self.manip.FindIKSolution(T, ikfilter_ignorecollision)
        if qsol is not None:
            # IKFast works. Return the IKFast solution directly.
            return qsol

        # Here IKFast does not return anything. It is either that a
        # solution exists but IKFast fails or no solution exists.
        targetpose = orpy.poseFromMatrix(T)
        for i in xrange(self._ntrials):
            # Perturb the desired T
            Tnew = PerturbT(T)
            with self.robot:
                if qref is not None:
                    self.robot.SetActiveDOFValues(qref)
                if checkcollision:
                    qinit = self.manip.FindIKSolution(Tnew, ikfilter_checkcollision)
                else:
                    qinit = self.manip.FindIKSolution(Tnew, ikfilter_ignorecollision)
            if qinit is None:
                continue            
            
            # Since qinit is assumably close to a real solution (if
            # one exists), we set max_it to be only 20.
            [reached, _, qsol] = self.diffiksolver.solve\
            (targetpose, qinit, dt=1.0, max_it=20, conv_tol=1e-8, checkcollision=checkcollision)
            
            if not reached:
                continue
            
            # message = "Desired transformation reached"
            # self.logger.info(message)
            return qsol

        message = "Failed to find an IK solution after {0} trials".format(self._ntrials)
        self.logger.info(message)
        
        return None


class RobustIKSolver5D(object):

    def __init__(self, robot, manipulatorname, ntrials=1000, qd_lim=1, loglevel=10):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        self.env = self.robot.GetEnv()

        # Load IKFast
        self.ikmodel5D = orpy.databases.inversekinematics.InverseKinematicsModel\
        (self.robot, iktype=iktype5D)
        if not self.ikmodel5D.load():
            msg = 'IK model 5D database for manipulator {0} not found.'
            msg += ' Please generate it first.'
            raise Exception(msg)
        self.manip.SetIkSolver(self.ikmodel5D.iksolver)

        # Initialize a differential IK solver
        self.diffiksolver = DiffIKSolver(self.manip, loglevel=40)

        # parameters
        self._ntrials = ntrials

        # Python logging
        self.logger = logging.getLogger(__name__)
        self.loglevel = loglevel
        FORMAT = "[%(module)s::%(funcName)s] %(message)s"
        logging.basicConfig(format=FORMAT)
        self.logger.setLevel(self.loglevel)

        
    def ActivateIKSolver(self):
        self.manip.SetIkSolver(self.ikmodel5D.iksolver)


    def FindIKSolution(self, point, direction, checkcollision=True):
        """
        point -- a 3D vector
        direction -- a 3D 'unit' vector
        """
        self.ActivateIKSolver()
        # Use IKFast
        ikparam = orpy.IkParameterization(orpy.Ray(point, direction), iktype5D)
        if checkcollision:
            qsol = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
        else:
            qsol = self.manip.FindIKSolution(ikparam, ikfilter_ignorecollision)
        if qsol is not None:
            # IKFast works. Return the IKFast solution directly.
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
            if checkcollision:
                qinit = self.manip.FindIKSolution(ikparam, ikfilter_checkcollision)
            else:
                qinit = self.manip.FindIKSolution(ikparam, ikfilter_ignorecollision)
            if qinit is None:
                continue
        
            # Since qinit is assumably close to a real solution (if
            # one exists), we set max_it to be only 20.
            [reached, _, qsol] = self.diffiksolver.solve\
            (targetpose, qinit, dt=1.0, max_it=20, conv_tol=1e-8, checkcollision=checkcollision)

            if not reached:
                continue
            
            # message = "Desired transformation reached"
            # self.logger.info(message)
            return qsol
        
        message = "Failed to find an IK solution after {0} trials".format(self._ntrials)
        self.logger.info(message)

        return None


    def FindIKSolutions(self, point, direction, checkcollision=True):
        """
        point -- a 3D vector
        direction -- a 3D 'unit' vector
        """
        self.ActivateIKSolver()
        # Use IKFast
        ikparam = orpy.IkParameterization(orpy.Ray(point, direction), iktype5D)
        if checkcollision:
            sols = self.manip.FindIKSolutions(ikparam, ikfilter_checkcollision)
        else:
            sols = self.manip.FindIKSolutions(ikparam, ikfilter_ignorecollision)
        if len(sols) > 0:
            # IKFast works. Return the IKFast solution directly.
            return sols

        # Compute an initial rotation
        z = np.array(direction).reshape((3, 1))
        x = PerpendicularTo(z).reshape((3, 1))
        y = np.cross(z.T, x.T).reshape((3, 1))
        R = np.hstack([x, y, z])
        
        nattempts = 3
        sols = []
        
        for i in xrange(nattempts):
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
            if checkcollision:
                sols_init = self.manip.FindIKSolutions(ikparam, ikfilter_checkcollision)
            else:
                sols_init = self.manip.FindIKSolutions(ikparam, ikfilter_ignorecollision)
            if len(sols_init) == 0:
                continue
        
            for qinit in sols_init:
                # Since qinit is assumably close to a real solution (if
                # one exists), we set max_it to be only 20.
                [reached, _, qsol] = self.diffiksolver.solve\
                (targetpose, qinit, dt=1.0, max_it=20, conv_tol=1e-8, checkcollision=checkcollision)
              
                if reached:
                    sols.append(qsol)
            
        if len(sols) == 0:       
            message = "Failed to find an IK solution after {0} trials".format(self._ntrials)
            self.logger.info(message)
        return sols


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

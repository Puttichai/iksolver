import numpy as np
import openravepy as orpy
import logging


class DiffIKSolver(object):
    
    def __init__(self, robot, manipulatorname, qd_lim=1, loglevel=10):
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.manip = robot.SetActiveManipulator(manipulatorname)
        n = self.robot.GetActiveDOF()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]
        self.qd_min = -qd_lim*np.ones(n)
        self.qd_max = +qd_lim*np.ones(n)

        # Python logging
        self.logger = logging.getLogger(__name__)
        self.loglevel = loglevel
        FORMAT = "[%(module)s::%(funcName)s] %(message)s"
        logging.basicConfig(format=FORMAT)
        self.logger.setLevel(self.loglevel)


    def solve(self, targetpose, q, dt=1.0, max_it=1000, conv_tol=1e-8, 
              checkcollision=True):
        """
        Compute joint values which bring the manipulator to T.

        targetpose -- desired manipulator transformation in the form of
                      7D-vector [quaternion, translation]
        q -- initial guess
        dt -- diff ik solver step size
        max_it -- maximum number of differential IK iterations
        conv_tol -- if all elements of the actual pose is not farther 
                    from their corresponding elements in the target pose
                    than conv_tol, the solver has already converged
        """
        if targetpose[0] < 0:
            targetpose[0:4] *= -1.

        cur_obj = 1000. # some arbitrary number
        it = 0 # iteration counter
        reached = False

        while it < max_it:
            it += 1
            
            pose_actual = self._get_pose(q)
            if np.allclose(targetpose, pose_actual, atol=conv_tol, rtol=0):
                # local minimum reached
                if not checkcollision:
                    incollision = False
                else:
                    with self.robot:
                        self.robot.SetActiveDOFValues(q)
                        incollision = (self.env.CheckCollision(self.robot) or
                                       self.robot.CheckSelfCollision())
                        
                if not incollision:
                    reached = True
                    # message = "Desired transformation reached"
                    # self.logger.info(message)
                else:
                    message = "Desired transformation reached but in collision"
                    self.logger.info(message)
                break

            qd = self._compute_velocity(targetpose, q)
            qd = np.maximum(np.minimum(qd, self.qd_max), self.qd_min)
            q = q + (dt * qd)
            q = np.maximum(np.minimum(q, self.q_max), self.q_min)

        if not reached:
            message = "Max iteration ({0}) reached".format(it)
            self.logger.info(message)
        return [reached, it, q]


    def _eval_error(self, x0, x1):
        return x0 - x1


    def _eval_objective(self, x, q):
        currentpose = self._get_pose(q)
        error = self._eval_error(x, currentpose)
        return 0.5*np.dot(error, error)


    def _get_pose(self, q):
        with self.robot:
            self.robot.SetActiveDOFValues(q)
            pose = self.manip.GetTransformPose()
        # Convention: cos(alpha) > 0
        # (from Stephane) this convention enforces Slerp shortest path
        if pose[0] < 0:
            pose[:4] *= -1
        return pose


    def _compute_velocity(self, targetpose, q):
        with self.robot:
            self.robot.SetActiveDOFValues(q)
            currentpose = self.manip.GetTransformPose()
            # Jacobian
            J_trans = self.manip.CalculateJacobian()
            J_quat = self.manip.CalculateRotationJacobian()

        if currentpose[0] < 0:
            currentpose[0:4] *= -1.
            J_quat *= -1.

        # Full Jacobian
        J = np.vstack([J_quat, J_trans])

        pose_error = self._eval_error(targetpose, currentpose)
        try:
            qd = np.linalg.solve(J, pose_error)
        except:
            qd = np.dot(np.linalg.pinv(J), pose_error)
        return qd
            

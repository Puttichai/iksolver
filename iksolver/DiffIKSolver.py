import numpy as np
import openravepy as orpy


class DiffIKSolver(object):

    def __init__(self, robot, manipulatorname, qd_lim=1):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        n = self.robot.GetActiveDOF()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]
        self.qd_min = -qd_lim*np.ones(n)
        self.qd_max = +qd_lim*np.ones(n)


    def solve(self, targetpose, q, dt, max_it=1000, conv_tol=1e-6):
        """
        Compute joint values which bring the manipulator to T.

        T -- desired manipulator transformation
        q0 -- initial guess
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
            # prev_obj = cur_obj
            # cur_obj = self._eval_objective(targetpose, q)
            # if abs(cur_obj - prev_obj) < conv_tol and cur_obj < conv_tol:
            #     # local minimum reached
            #     reached = True
            #     break
            
            pose_actual = self._get_pose(q)
            if np.allclose(targetpose, pose_actual, atol=conv_tol, rtol=0):
                # local minimum reached
                reached = True
                break

            qd = self._compute_velocity(targetpose, q)
            qd = np.maximum(np.minimum(qd, self.qd_max), self.qd_min)
            q = q + (dt * qd)
            q = np.maximum(np.minimum(q, self.q_max), self.q_min)

        if not reached:
            print '[solve] max iteration ({0}) exceeded'.format(it)
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
            

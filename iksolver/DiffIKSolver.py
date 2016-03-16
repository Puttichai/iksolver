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


    def solve(self, targetpose, q, dt, max_it=1000, conv_tol=1e-5):
        """
        Compute joint values which bring the manipulator to T.

        T -- desired manipulator transformation
        q0 -- initial guess
        max_it -- maximum number of differential IK iterations
        conv_tol -- if the objective dwindles by less than this
                    threshold after a differential IK step, we assume
                    the solver has converged to the best solution it
                    can find
        """
        # R = T[0:3, 0:3]
        # p = T[0:3, 3]
        # targetpose = np.hstack([orpy.quatFromRotationMatrix(R), p])
        if targetpose[0] < 0:
            targetpose[0:4] *= -1.

        cur_obj = 1000. # some arbitrary number
        it = 0 # iteration counter
        reached = False

        while it < max_it:
            it += 1
            prev_obj = cur_obj
            cur_obj = self._eval_objective(targetpose, q)
            if abs(cur_obj - prev_obj) < conv_tol and cur_obj < conv_tol:
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
        # err = np.zeros(7)
        # err[:4] = orpy.quatMultiply(x0[:4], orpy.quatInverse(x1[:4]))
        # err[4:] = x0[4:] - x1[4:]
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

            
class DiffIKSolver5D(object):
    def __init__(self, robot, manipulatorname, qd_lim=1):
        self.robot = robot
        self.manip = robot.SetActiveManipulator(manipulatorname)
        n = self.robot.GetActiveDOF()
        self.q_min = self.robot.GetActiveDOFLimits()[0]
        self.q_max = self.robot.GetActiveDOFLimits()[1]
        self.qd_min = -qd_lim*np.ones(n)
        self.qd_max = +qd_lim*np.ones(n)


    def solve(self, targetpose, q, dt, max_it=1000, conv_tol=1e-5):
        """
        Compute joint values which bring the manipulator to T.

        T -- desired manipulator transformation
        q0 -- initial guess
        max_it -- maximum number of differential IK iterations
        conv_tol -- if the objective dwindles by less than this
                    threshold after a differential IK step, we assume
                    the solver has converged to the best solution it
                    can find
        """
        # R = T[0:3, 0:3]
        # p = T[0:3, 3]
        # targetpose = np.hstack([orpy.quatFromRotationMatrix(R), p])
        if targetpose[0] < 0:
            targetpose[0:4] *= -1.

        cur_obj = 1000. # some arbitrary number
        it = 0 # iteration counter
        reached = False

        while it < max_it:
            it += 1
            prev_obj = cur_obj
            cur_obj = self._eval_objective(targetpose, q)
            # if abs(cur_obj - prev_obj) < conv_tol and cur_obj < conv_tol:
            #     # local minimum reached
            #     reached = True
            #     break
            cos_alpha_desired = targetpose[0]
            sin_alpha_desired = np.sqrt(1 - cos_alpha_desired**2)
            if abs(sin_alpha_desired) > 1e-8:
                direction_desired = targetpose[1:4] / sin_alpha_desired
            else:
                direction_desired = np.zeros(3)

            pose_actual = self._get_pose(q)
            cos_alpha_actual = pose_actual[0]
            sin_alpha_actual = np.sqrt(1 - cos_alpha_actual**2)
            if abs(sin_alpha_actual) > 1e-8:
                direction_actual = pose_actual[1:4] / sin_alpha_actual
            else:
                direction_actual = np.zeros(3)
                
            if (np.allclose(direction_desired, direction_actual,
                            rtol=0*1e-8, atol=1e-6) and
                np.allclose(targetpose[4:], pose_actual[4:],
                            rtol=0*1e-8, atol=1e-6)):
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


    def _eval_objective(self, x, q):
        currentpose = self._get_pose(q)
        error_rot = self._eval_error_rotation(x, currentpose)
        error_pos = self._eval_error_position(x, currentpose)
        
        return 0.5*(np.dot(error_rot, error_rot) + np.dot(error_pos, error_pos))


    def _eval_error_rotation(self, x, currentpose):
        cos_alpha1 = currentpose[0]
        sin_alpha1 = np.sqrt(1 - cos_alpha1**2)
        if abs(sin_alpha1) > 1e-8:
            w_current = currentpose[1:4]/sin_alpha1
        else:
            w_current = np.zeros(3)

        cos_alpha2 = x[0]
        sin_alpha2 = np.sqrt(1 - cos_alpha2**2)
        if abs(sin_alpha2) > 1e-8:
            w_desired = x[1:4]/sin_alpha2
        else:
            w_desired = np.zeros(3)
        w_desired = x[1:4]/sin_alpha2

        error_rot = w_desired - w_current
        return error_rot

    
    def _eval_error_position(self, x, currentpose):
        error_pos = x[4:] - currentpose[4:]
        return error_pos


    def _get_pose(self, q):
        with self.robot:
            try:
                self.robot.SetActiveDOFValues(q)
            except:
                print q
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

        error_rot = self._eval_error_rotation(targetpose, currentpose)
        try:
            qd_rot = np.linalg.solve(J_quat[1:, :], error_rot)
        except:
            qd_rot = np.dot(np.linalg.pinv(J_quat[1:, :]), error_rot)

        error_pos = self._eval_error_position(targetpose, currentpose)
        try:
            qd_pos = np.linalg.solve(J_trans, error_pos)
        except:
            qd_pos = np.dot(np.linalg.pinv(J_trans), error_pos)
            
        return qd_rot + qd_pos

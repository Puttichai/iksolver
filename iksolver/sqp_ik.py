import numpy as np
import time

# Openravepy
try:
    from openravepy import Environment
    from openravepy import quatMultiply, quatInverse, axisAngleFromQuat
    import rospkg
except ImportError:
    pass

# cvxopt
try:
    from cvxopt.solvers import options, qp
    from cvxopt import matrix
    # Turn off output from cvxopt
    options['show_progress'] = False
except ImportError:
    print "cvxopt is not installed!"

# Mosek
try:
    import mosek
    options['MOSEK'] = {mosek.iparam.log: 0}
except ImportError:
    mosek_available = False
    pass
    print "mosek is not installed"



class OptimalNotFound(Exception):
    pass


def solve_qp(Q, p, G, h, A, b, solver=None):
    t0 = time.time()
    args = []
    args.append(matrix(Q))
    args.append(matrix(p))
    # args = [None, None]
    if G is not None:
        args.append(matrix(G))
        args.append(matrix(h))
    else:
        args.append(None)
        args.append(None)
    if A is not None:
        args.append(matrix(A))
        args.append(matrix(b))
    t1 = time.time()
    sol = qp(*args, solver=solver)
    t2 = time.time()
    if sol['status'] is not 'optimal':
        print sol['status'], " is not optimal"
        # raise OptimalNotFound(sol['status'])
    # print "Matrix conversion took: %0.10f seconds" % (t1 - t0)
    # print "CVXOPT terminate in: %0.10f seconds" % (t2 - t1)
    return np.array(sol['x']).reshape((Q.shape[0],))


class IksqpTask(object):
    """Constraint/Objective matrix for each task

    All are functions except obj_cost (matrix) and obj_gain (scalar)
    """
    def __init__(self, cnst_A=None, cnst_b=None, cnst_evaluate=None, obj_Q=None, obj_p=None, obj_evaluate=None, obj_gain=1.0):
        self.cnst_A = cnst_A
        self.cnst_b = cnst_b
        self.cnst_evaluate = cnst_evaluate
        self.obj_Q = obj_Q
        self.obj_p = obj_p
        self.obj_evaluate = obj_evaluate
        self.obj_gain = obj_gain

class IKsqp(object):
    ''' Iksolver for robotic manipulator using SQP
    '''
    def __init__(self, robot=None, manip=None):
        self.robot = robot
        self.manip = manip
        self.dofindices = self.manip.GetArmIndices()
        self.dofs = len(self.dofindices)  # dimension of joint values q
        self.robot.SetActiveDOFs(self.dofindices)
        
        # Reformulate joint limit as inequality constraint
        self.joint_limits = self.robot.GetDOFLimits()
        
        self.G = np.concatenate((np.eye(self.dofs), - np.eye(self.dofs)), axis=0)
        self.h = np.concatenate((self.joint_limits[1], - self.joint_limits[0]), axis=0)
        self.h = self.h.reshape((self.h.shape[0], 1))

        # Constraints/Objectives init
        self.constraints = []
        self.objectives = []

        # Setup Iktype
        self.iktype = None

    def _add_constraint(self, constraint):
        """ Add constraint to the solver.

        Notice: a nonlinear constraint f(q) - x_0 is approximate by a quadratic form
                Aq = b
        where A = jacobian_x0(q), b = (x_0 - f(x_0) + jacobian_x0 * x_0)
        """
        self.constraints.append(constraint)

    def _add_objective(self, objective):
        assert objective.obj_gain is not None, "Gain must present"
        self.objectives.append(objective)

    def _clear_constraint_objective(self):
        self.constraints = []
        self.objectives = []

    def _calculate_objective_matrices(self, q_prev):
        ''' Objective always has the form
        cost = q^T Q q + p^T q
        '''
        Q = np.zeros((self.dofs, self.dofs))
        p = np.zeros((self.dofs, 1))
        for obj in self.objectives:
            Q += obj.obj_Q(q_prev) * obj.obj_gain
            p += obj.obj_p(q_prev) * obj.obj_gain
        # Q += np.eye(self.dofs) * 0.001
        return Q, p

    def _calculate_equality_constraints_matrices(self, q_current):
        ''' A, b. Calculated from q_current
        '''
        A = []
        b = []
        for cnst in self.constraints:
            matA = cnst.cnst_A(q_current)
            rows = matA.shape[0]
            matb = cnst.cnst_b(q_current).reshape(rows, )
            A.append(matA)
            # print "Matrix A: %s" % np.array_str(J)
            # print "SVD: %s" % (np.linalg.svd(J)[1])
            b.append(matb)
        if len(A) > 1:
            A = np.concatenate(A, axis=0)
            b = np.concatenate(b, axis=0)
        elif len(A) == 1:
            A = A[0]
            b = b[0]
        elif len(A) == 0:
            A = None
            b = None

        return A, b

    def _evaluate_objective(self, q_prev):
        obj_cost = 0.0
        for obj in self.objectives:
            obj_cost += obj.obj_evaluate(q_prev)
        return obj_cost

    def _evaluate_constraint(self, q_prev):
        cnst_eval = 0.0
        for cnst in self.constraints:
            cnst_eval += np.abs(cnst.cnst_evaluate(q_prev))
        return cnst_eval

    def find_solution(self, param):
        ''' Wrapper for Ik6D,
        '''
        iktype = param['type']
        if iktype == 'ik6D':
            return self._find_solution_ik6D(param)
        elif iktype == 'ik3DTranslation':
            return self._find_solution_ik3DTranslation(param)
        elif iktype == 'ik3DRotation':
            return self._find_solution_ik3DRotation(param)


    def _find_solution_ik6D(self, param):
        pose_goal = param['data']
        qu_goal = pose_goal[0:4]
        x_goal = pose_goal[4:]
        form = param['formulation']
        qseed = param['seed']

        self._clear_constraint_objective()
        self._add_orientation(qu_goal ,form=form[0])
        self._add_position(x_goal, form=form[1])

        result = self._solve_sqp(qseed=qseed)

        return result 

    def _find_solution_ik3DTranslation(self, param):
        # params for ik3DTranslation
        obj_tol = 1e-15
        cnst_tol = 1e-10
        hybrid_obj_thres=0.0005

        x_goal = param['data']
        qseed = param['seed']
        try:
            form = param['formulation']
        except KeyError:
            form = 'constraint'
        if form == 'hybrid':
            self._clear_constraint_objective()
            self._add_position(x_goal, form='objective')
            q_temp = self._solve_sqp(qseed=qseed, obj_tol=hybrid_obj_thres)['q']

            self._clear_constraint_objective()
            self._add_position(x_goal, form='constraint')
            q = self._solve_sqp(qseed=q_temp, obj_tol=obj_tol, cnst_tol=cnst_tol)
        else:
            self._clear_constraint_objective()
            self._add_position(x_goal, form=form)
            q = self._solve_sqp(qseed = qseed, obj_tol=obj_tol, cnst_tol=cnst_tol)
        return q

    def _find_solution_ik3DRotation(self, param):
        x_goal = param['data']
        qseed = param['seed']
        try:
            form = param['formulation']
        except KeyError:
            form = 'constraint'
        self._clear_constraint_objective()
        self._add_orientation(x_goal, form=form, gain=1e5)
        self._add_minimize_norm_objective(qnorm=qseed)
        q = self._solve_sqp(qseed=qseed)
        return q

    def _solve_sqp(self, qseed=None, debug=False, maxiter=300, obj_tol=1e-15, cnst_tol=1e-5):
        """ Find solutions using SQP

        Only converges when 
        (obj[i] - obj[i-1]) < e1 and cnst_eval < e2

        Args:
            qseed:  A nx1 numpy array containing qseed value
        """
        # Profiling
        t0 = time.time()
        
        objs = [self._evaluate_objective(qseed)]
        q_values = [qseed]
        i = 0
        obj_converged = False
        cnst_converged = False
        converged = False
        while i < maxiter and not converged:

            q_prev = q_values[-1]

            Q, p = self._calculate_objective_matrices(q_prev)
            A, b = self._calculate_equality_constraints_matrices(q_prev)

            q_opt = solve_qp(Q, p, self.G, self.h, A, b)  # Solve with cvxopt
            q_values.append(q_opt)
            i += 1
            objs.append(self._evaluate_objective(q_opt))
            if np.abs(objs[i] - objs[i-1]) < obj_tol:  # Converged
                obj_converged = True
            if self._evaluate_constraint(q_opt) < cnst_tol:
                cnst_converged = True
            if obj_converged and cnst_converged:
                converged = True
        duration = time.time() - t0

        result = {'converged': converged,
                  'q': q_opt,
                  'noiter': i,
                  'duration': duration}

        return result

    def _add_position(self, x, form='constraint'):
        ''' Add position constraint
        '''
        W = np.eye(3)
        def cnst_A(q0):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                return self.manip.CalculateJacobian()
        def cnst_b(q0, x=x):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                x0 = self.manip.GetEndEffectorTransform()[0:3, 3]    
            temp_1 = cnst_A(q0).dot(q0)
            temp_2 = np.array(x).reshape(3, ) - x0.reshape(3, )
            b = temp_1 + temp_2
            return b.reshape(3, 1)



            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                x0 = self.manip.GetEndEffectorTransform()[0:3, 3]
            return np.linalg.norm(x - x0)
        def cnst_evaluate(q0, x=x):
            return obj_evaluate(q0, x=x)

        def obj_Q(q0):
            A = cnst_A(q0)
            Q = 2 * np.dot(A.T, W).dot(A)
            return Q

        def obj_p(q0, x=x):
            A = cnst_A(q0)
            b = cnst_b(q0, x)
            p = - 2 * np.dot(A.T, W).dot(b)
            return p.reshape(6, 1)

        def obj_evaluate(q0, x=x):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                x0 = self.manip.GetEndEffectorTransform()[0:3, 3]
            err = x0 - x
            obj = np.dot(err, W).dot(err)
            return obj

        
        position_task = IksqpTask(cnst_A, cnst_b, cnst_evaluate, obj_Q, obj_p, obj_evaluate, obj_gain = 1.0)
        if form == 'constraint':
            self._add_constraint(position_task)
        elif form == 'objective':
            self._add_objective(position_task)

    def _add_orientation(self, qu, form='constraint', gain=1.0):
        """ Add orientation constraint. qu is a quarternion
        """
        W = np.eye(3)
        def cnst_A(q0):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                return self.manip.CalculateAngularVelocityJacobian()

        def cnst_b(q0, qu=qu):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                qu0 = self.manip.GetTransformPose()[0:4]

            quat_err = quatMultiply(qu, quatInverse(qu0))
            axangle_err = axisAngleFromQuat(quat_err)
            temp = np.dot(cnst_A(q0), q0)
            b = axangle_err + temp
            return b.reshape(3, 1)

        def cnst_evaluate(q0, qu=qu):
            with self.robot:
                self.robot.SetActiveDOFValues(q0)
                qu0 = self.manip.GetTransformPose()[0:4]
            quat_err = quatMultiply(qu, quatInverse(qu0))
            axangle_err = axisAngleFromQuat(quat_err)

            return np.linalg.norm(axangle_err)

        def obj_Q(q0):
            A = cnst_A(q0)
            Q = 2 * np.dot(A.T, W).dot(A)
            return Q

        def obj_p(q0, qu=qu):
            A = cnst_A(q0)
            b = cnst_b(q0, qu)
            p = - 2 * np.dot(A.T, W).dot(b)
            return p.reshape(6, 1)

        def obj_evaluate(q0, qu=qu):
            return cnst_evaluate(q0, qu=qu)


        orientation_task = IksqpTask(cnst_A, cnst_b, cnst_evaluate, obj_Q, obj_p, obj_evaluate, gain)
        if form == 'constraint':
            self._add_constraint(orientation_task)
        elif form == 'objective':
            self._add_objective(orientation_task)


    def _add_minimize_norm_objective(self, qnorm=np.zeros(6)):
        W = np.eye(self.dofs)
        def obj_Q(q0):
            return 2 * W

        def obj_p(q0, q=qnorm):
            return -2 * np.dot(q, W).reshape(6, 1)

        def obj_evaluate(q0, q=qnorm):
            dq = q0 - q
            return np.dot(dq, W).dot(dq)
        min_norm = IksqpTask(None, None, None, obj_Q, obj_p, obj_evaluate)
        self._add_objective(min_norm)










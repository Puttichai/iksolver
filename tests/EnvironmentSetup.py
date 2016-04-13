import unittest
import openravepy as orpy

import rospkg
rospack = rospkg.RosPack()


class OpenRAVEsetup(unittest.TestCase):

    def setUp(self, robotfile='robots/denso_vs060.dae', manipulatorname='Flange',
              collisioncheckername='ode'):
        self.env = orpy.Environment()

        # Get the DensoVS60 model - fast loading time
        self.robot = self.env.ReadRobotURI(robotfile)
        self.env.Add(self.robot, True)
        collisionchecker = orpy.RaveCreateCollisionChecker(self.env, collisioncheckername)
        self.env.SetCollisionChecker(collisionchecker)
        self.robot.SetName('Denso')
        self.manip = self.robot.GetManipulator(manipulatorname)

    def tearDown(self):
        self.env.Destroy()

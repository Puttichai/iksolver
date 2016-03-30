import unittest
import openravepy as orpy

import rospkg
rospack = rospkg.RosPack()


class OpenRAVEsetup(unittest.TestCase):

    def setUp(self, robotfile='/robots/denso-vs060.zae', manipulatorname='Flange',
              collisioncheckername='ode'):
        self.env = orpy.Environment()

        # Get the DensoVS60 model - fast loading time
        robot_path = rospack.get_path('cribus_openrave') + robotfile
        self.robot = self.env.ReadRobotURI(robot_path)
        self.env.Add(self.robot, True)
        collisionchecker = orpy.RaveCreateCollisionChecker(self.env, collisioncheckername)
        self.env.SetCollisionChecker(collisionchecker)
        self.robot.SetName('Denso')
        self.manip = self.robot.GetManipulator(manipulatorname)

    def tearDown(self):
        self.env.Destroy()

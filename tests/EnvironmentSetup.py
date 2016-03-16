import unittest
from openravepy import Environment

import rospkg
rospack = rospkg.RosPack()


class OpenRAVEsetup(unittest.TestCase):

    def setUp(self):
        self.env = Environment()

        # Get the DensoVS60 model - fast loading time
        robot_path = rospack.get_path('cribus_openrave') + '/robots/denso-vs060.zae'
        self.robot = self.env.ReadRobotURI(robot_path)
        self.env.Add(self.robot, True)
        self.robot.SetName('Denso')
        self.manip = self.robot.GetManipulator('Flange')

    def tearDown(self):
        self.env.Destroy()

import r3
import r3.ext.CustomController

env = r3.Environment()
robot = r3.Robot()

robot.SetController(r3.ext.CustomController())

c = robot.GetController()
c.CustomCall()

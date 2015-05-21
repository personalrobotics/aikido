import dartpy
import numpy
import time
import r3py

def set_dof_values(dofs, values):
    for dof, value in zip(dofs, values):
        dof.position = value

# Create the environment.
PACKAGE_NAME = 'herb_description'
PACKAGE_PATH = '/home/parallels/ros-herb/src/herb_description'

ROBOT_PATH \
    = '/home/parallels/ros-herb/devel/share/herb_description/robots/herb.urdf'
ROBOT_POSE = numpy.array([
    [ 0.   ,  1.   ,  0.   , -0.915],
    [-1.   ,  0.   , -0.   ,  1.405],
    [-0.   ,  0.   ,  1.   ,  0.   ],
    [ 0.   ,  0.   ,  0.   ,  1.   ]])
ROBOT_DOF_NAMES = [
    '/right/j1', '/right/j2', '/right/j3', '/right/j4',
    '/right/j5', '/right/j6', '/right/j7' ]
ROBOT_DISABLE_PAIRS = [
  ('/left/finger0_0', '/left/finger1_0'),
  ('/left/finger0_0', '/left/finger0_1'),
  ('/left/finger0_0', '/left/finger0_2'),
  ('/left/finger0_0', '/left/finger1_1'),
  ('/left/finger0_0', '/left/finger1_2'),
  ('/left/finger0_0', '/left/finger2_1'),
  ('/left/finger0_0', '/left/finger2_2'),
  ('/left/finger1_0', '/left/finger0_1'),
  ('/left/finger1_0', '/left/finger0_2'),
  ('/left/finger1_0', '/left/finger1_1'),
  ('/left/finger1_0', '/left/finger1_2'),
  ('/left/finger1_0', '/left/finger2_1'),
  ('/left/finger1_0', '/left/finger2_2'),
  ('/left/finger0_1', '/left/finger0_2'),
  ('/left/finger0_1', '/left/finger1_1'),
  ('/left/finger0_1', '/left/finger1_2'),
  ('/left/finger0_1', '/left/finger2_1'),
  ('/left/finger0_1', '/left/finger2_2'),
  ('/left/finger0_2', '/left/finger1_1'),
  ('/left/finger0_2', '/left/finger1_2'),
  ('/left/finger0_2', '/left/finger2_1'),
  ('/left/finger0_2', '/left/finger2_2'),
  ('/left/finger1_1', '/left/finger1_2'),
  ('/left/finger1_1', '/left/finger2_1'),
  ('/left/finger1_1', '/left/finger2_2'),
  ('/left/finger1_2', '/left/finger2_1'),
  ('/left/finger1_2', '/left/finger2_2'),
  ('/left/finger2_1', '/left/finger2_2'),
  ('/left/hand_base', '/left/finger1_0'),
  ('/left/hand_base', '/left/finger0_1'),
  ('/left/hand_base', '/left/finger0_2'),
  ('/left/hand_base', '/left/finger1_1'),
  ('/left/hand_base', '/left/finger1_2'),
  ('/left/hand_base', '/left/finger2_1'),
  ('/left/hand_base', '/left/finger2_2'),
  ('/right/finger0_0', '/right/finger1_0'),
  ('/right/finger0_0', '/right/finger0_1'),
  ('/right/finger0_0', '/right/finger0_2'),
  ('/right/finger0_0', '/right/finger1_1'),
  ('/right/finger0_0', '/right/finger1_2'),
  ('/right/finger0_0', '/right/finger2_1'),
  ('/right/finger0_0', '/right/finger2_2'),
  ('/right/finger1_0', '/right/finger0_1'),
  ('/right/finger1_0', '/right/finger0_2'),
  ('/right/finger1_0', '/right/finger1_1'),
  ('/right/finger1_0', '/right/finger1_2'),
  ('/right/finger1_0', '/right/finger2_1'),
  ('/right/finger1_0', '/right/finger2_2'),
  ('/right/finger0_1', '/right/finger0_2'),
  ('/right/finger0_1', '/right/finger1_1'),
  ('/right/finger0_1', '/right/finger1_2'),
  ('/right/finger0_1', '/right/finger2_1'),
  ('/right/finger0_1', '/right/finger2_2'),
  ('/right/finger0_2', '/right/finger1_1'),
  ('/right/finger0_2', '/right/finger1_2'),
  ('/right/finger0_2', '/right/finger2_1'),
  ('/right/finger0_2', '/right/finger2_2'),
  ('/right/finger1_1', '/right/finger1_2'),
  ('/right/finger1_1', '/right/finger2_1'),
  ('/right/finger1_1', '/right/finger2_2'),
  ('/right/finger1_2', '/right/finger2_1'),
  ('/right/finger1_2', '/right/finger2_2'),
  ('/right/finger2_1', '/right/finger2_2'),
  ('/right/hand_base', '/right/finger1_0'),
  ('/right/hand_base', '/right/finger0_1'),
  ('/right/hand_base', '/right/finger0_2'),
  ('/right/hand_base', '/right/finger1_1'),
  ('/right/hand_base', '/right/finger1_2'),
  ('/right/hand_base', '/right/finger2_1'),
  ('/right/hand_base', '/right/finger2_2'),
  ('/left/wam_base', '/right/wam_base'),
  ('/left/wam1', '/right/wam1'),
  ('/herb_base', '/head/wam2'),
  # These are missing:
  ('/herb_base', '/head/wam1'),
  ('/herb_base', '/left/wam_base'),
  ('/herb_base', '/left/wam1'),
  ('/herb_base', '/right/wam_base'),
  ('/herb_base', '/right/wam1'),
  ('/herb_base', '/segway_wheel_left'),
  ('/herb_base', '/segway_wheel_right'),
  ('/head/wam1', '/head/wam2'),
  ('/left/wam_base', '/left/wam1'),
  ('/left/wam1', '/left/wam2'),
  ('/left/wam1', '/left/wam3'),
  ('/left/wam2', '/left/wam3'),
  ('/left/wam3', '/left/wam4'),
  ('/left/wam4', '/left/wam5'),
  ('/left/wam4', '/left/wam6'),
  ('/left/wam5', '/left/wam6'),
  ('/left/wam6', '/left/wam7'),
  ('/left/wam7', '/left/hand_base'),
  ('/left/hand_base', '/left/finger0_0'),
  ('/right/wam_base', '/right/wam1'),
  ('/right/wam1', '/right/wam2'),
  ('/right/wam1', '/right/wam3'),
  ('/right/wam2', '/right/wam3'),
  ('/right/wam3', '/right/wam4'),
  ('/right/wam4', '/right/wam5'),
  ('/right/wam4', '/right/wam6'),
  ('/right/wam5', '/right/wam6'),
  ('/right/wam6', '/right/wam7'),
  ('/right/wam7', '/right/hand_base'),
  ('/right/hand_base', '/right/finger0_0'),
]

START_CONFIGS = numpy.array([[ 3.68, -1.9 ,  0.  ,  2.2 ,  0.  ,  0.  ,  0. ]])
GOAL_CONFIG = numpy.array([ 4.73498745, -1.05417933, -0.26      ,  1.52308093,
                           -0.44203032, -0.52187769, -2.63664543 ])

OBJECT_PATH = '../urdf/fuze_bottle.urdf'
OBJECT_POSE = numpy.eye(4)
OBJECT_POSE[0:3, 3] = [ -1.46405,  0.4322 ,  0.735 ]

urdf_loader = dartpy.DartLoader()
urdf_loader.add_package_directory(PACKAGE_NAME, PACKAGE_PATH)

"""
right_hand = robot.get_body_node_by_name('/right/hand_base')
bottle.pose = OBJECT_POSE
bottle.get_root_body_node(0).moveTo(right_hand, dartpy.JointType.WELD)
"""

# Load the robot.
skel = urdf_loader.parse_skeleton(ROBOT_PATH)
skel.pose = ROBOT_POSE

robot = skel.get_root_body_node(0)

# load the bottle.
bottle_skel = urdf_loader.parse_skeleton(OBJECT_PATH)
bottle_skel.get_root_body_node(0).moveTo(skel, None, dartpy.JointType.FREE)
bottle = skel.get_root_body_node(1)

# Setup the collision checker.
# TODO: Does this work if we add BodyNode's to the Skeleton.
collision_checker = dartpy.FCLCollisionDetector()
collision_checker.add_skeleton(skel)

skel.enable_self_collision(False) # ignore adjacent links
for node1_name, node2_name in ROBOT_DISABLE_PAIRS:
    collision_checker.disable_pair(
        skel.get_body_node_by_name(node1_name),
        skel.get_body_node_by_name(node2_name)
    )

# Render the trajectory
world = dartpy.World()
world.add_skeleton(skel)

window = dartpy.SimWindow(1600, 1200, 'ADA')
window.world = world

# call the planner
dofs = [ skel.get_dof_by_name(name) for name in ROBOT_DOF_NAMES ]
weights = numpy.ones(len(dofs))
resolutions = 0.02 * numpy.ones(len(dofs))

path = r3py.ompl_plan(collision_checker, dofs, weights, resolutions,
                      START_CONFIGS, GOAL_CONFIG)
set_dof_values(dofs, START_CONFIGS[:, 0])

"""
while True:
    for waypoint1, waypoint2 in zip(path[:-1], path[1:]):
        for r in numpy.linspace(0., 1., 200):
            values = (1 - r) * waypoint1 + (r) * waypoint2
            set_dof_values(dofs, values)
            time.sleep(0.02)
"""


"""
q_start = numpy.array([[ 1.486,  -1.570,  0.000,  2.034,  4.818,  1.934 ]])
q_goal = numpy.array([ 0.43135194, -1.25267446,  0.70220488,  0.2222944 ,
                      -0.92543907, -1.33936598 ])


for waypoint1, waypoint2 in zip(path[:-1], path[1:]):
    for r in numpy.linspace(0., 1., 200):
        values = (1 - r) * waypoint1 + (r) * waypoint2

        for dof, value in zip(dofs, values):
            dof.position = value

        time.sleep(0.02)
"""

import dartpy
import numpy
import time
import r3py

def set_dof_values(dofs, values):
    for dof, value in zip(dofs, values):
        dof.position = value

def load_urdf(skel, urdf_path, urdf_paths=None, pose=None):
    from dartpy import JointType

    # Setup the URDF loader.
    urdf_loader = dartpy.DartLoader()
    if urdf_paths is not None:
        for package_name, package_path in urdf_paths.iteritems():
            urdf_loader.add_package_directory(package_name, package_path)

    # Load the URDF file.
    urdf_skel = urdf_loader.parse_skeleton(urdf_path)
    urdf_root = urdf_skel.get_root_body_node(0)

    # Merge the loaded file into the existing Skeleton.
    props = dartpy.FreeJoint.Properties()
    if pose is not None:
        props.T_child_body_to_joint = numpy.linalg.inv(pose)

    urdf_root.moveTo(JointType.FREE, skel, None, props)

    return urdf_root

def grab(gripper, target):
    joint_props = dartpy.WeldJoint.Properties()
    joint_props.T_parent_body_to_joint = numpy.dot(
        numpy.linalg.inv(gripper.world_transform),
        target.world_transform
    )

    target.moveTo(dartpy.JointType.WELD, gripper, joint_props)

def release(gripper, target):
    joint_props = dartpy.FreeJoint.Properties()
    joint_props.T_child_body_to_joint = numpy.linalg.inv(target.world_transform)

    target.moveTo(dartpy.JointType.FREE, gripper.skeleton, None, joint_props)

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

skel = dartpy.Skeleton('environment')

# Load the robot.
bottle = load_urdf(skel, OBJECT_PATH, pose=OBJECT_POSE)
robot = load_urdf(skel, ROBOT_PATH,
    urdf_paths={'herb_description': '/home/parallels/ros-herb/src/herb_description'},
    pose=ROBOT_POSE)

# Setup the collision checker.
# TODO: Does this work if we add BodyNode's to the Skeleton.
collision_checker = dartpy.FCLCollisionDetector()
"""
collision_checker.add_skeleton(skel)

skel.enable_self_collision(False) # ignore adjacent links
for node1_name, node2_name in ROBOT_DISABLE_PAIRS:
    collision_checker.disable_pair(
        skel.get_body_node_by_name(node1_name),
        skel.get_body_node_by_name(node2_name)
    )
"""

# Render the trajectory
world = dartpy.World()
world.add_skeleton(skel)

window = dartpy.SimWindow(1600, 1200, 'HERB')
window.world = world

# call the planner
print 'Planning to grasp.'
dofs = [ skel.get_dof_by_name(name) for name in ROBOT_DOF_NAMES ]
weights = numpy.ones(len(dofs))
resolutions = 0.02 * numpy.ones(len(dofs))

path = r3py.ompl_plan(collision_checker, dofs, weights, resolutions,
                      START_CONFIGS, GOAL_CONFIG)

print 'Snapping to end of grasp path.'
set_dof_values(dofs, path[-1, :])

print 'Grabbing glass.'
grab(skel.get_body_node_by_name('/right/hand_base'), bottle)

path = r3py.ompl_plan(collision_checker, dofs, weights, resolutions,
        GOAL_CONFIG.reshape((1,7)), START_CONFIGS[0, :])

release(skel.get_body_node_by_name('/right/hand_base'), bottle)

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

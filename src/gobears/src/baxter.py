from planner import PathPlanner
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface.camera import CameraController
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform, Quaternion
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
from moveit_msgs.msg import Constraints, OrientationConstraint

RESET_POSE = PoseStamped()
RESET_POSE.header.frame_id = "base" 
RESET_POSE.pose.position.x = 0.730
RESET_POSE.pose.position.y = 0.316
RESET_POSE.pose.position.z = 0.308
RESET_POSE.pose.orientation.y = 1

G_AL_SETUP = np.array([[-0.02543927,  0.99577036,  0.08828496, -0.02402326],
       [ 0.97391449,  0.00476705,  0.22686526, -0.07992888],
       [ 0.22548485,  0.09175329, -0.96991645,  0.0265129 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_FINAL = np.array([[-0.0135772 ,  0.99782634,  0.06448454, -0.01883563],
       [ 0.97529907, -0.0010041 ,  0.2208862 , -0.0659357 ],
       [ 0.22047082,  0.06589073, -0.97316547, -0.05047197],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_PLANT_HOVER = np.array([[-0.00727591, -0.999484  , -0.03128564,  0.04294672],
       [-0.99968043,  0.00802766, -0.02397076,  0.05877309],
       [ 0.02420954,  0.03110123, -0.999223  ,  0.09003119],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_PLANT = np.array([[ 0.12182882, -0.99106231, -0.05434363,  0.0326852 ],
       [-0.99246839, -0.12092902, -0.01956176,  0.05275602],
       [ 0.0128152 ,  0.05631752, -0.99833066,  0.00728708],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_WATER_HOVER = np.array([[ 0.06309455, -0.99749481, -0.03198712,  0.01534037],
       [-0.20222637, -0.04416438,  0.97834248,  0.01183699],
       [-0.97730424, -0.05525944, -0.20450629,  0.06188132],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_WATER_PICK = np.array([[ 0.09096591, -0.99384733, -0.06318778,  0.00619209],
       [-0.18901519, -0.07952819,  0.97874845,  0.08207197],
       [-0.97775174, -0.0770893 , -0.19508659,  0.09284914],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_WATER_BIN_HOVER = np.array([[-0.28975986, -0.04960144,  0.95581323,  0.01587432],
       [-0.03319443,  0.99857621,  0.04175754,  0.08506615],
       [-0.95652358, -0.01962802, -0.29099379,  0.20764598],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_WATER_BIN_WATER = np.array([[-0.96268888, -0.03020991, -0.26891909, -0.05169169],
       [-0.04542564,  0.99768853,  0.05053817,  0.08517795],
       [ 0.26677074,  0.06086836, -0.96183596,  0.20559746],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_PLANT_HOVER = np.array([[ 0.00504796, -0.99884453, -0.04779254,  0.01810498],
       [-0.99960864, -0.00372524, -0.02772508,  0.05309667],
       [ 0.02751501,  0.04791379, -0.99847243,  0.09399243],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_PLANT_HOVER_2 = np.array([[-0.99624721, -0.08180927, -0.02826198,  0.05263162],
       [-0.08085862,  0.99616962, -0.0332862 ,  0.06690625],
       [ 0.03087685, -0.03087606, -0.99904619,  0.30490677],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_PLANT_PICK = np.array([[-3.94317501e-04, -9.99724926e-01,  2.34503050e-02,
         1.55665657e-02],
       [-9.98514304e-01, -8.84155649e-04, -5.44830485e-02,
         5.34379820e-02],
       [ 5.44887953e-02, -2.34369486e-02, -9.98239290e-01,
         2.59223723e-02],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
         1.00000000e+00]])


G_AL_PLANT_PLACE = np.array([[-0.99920738,  0.03322426, -0.02192644,  0.05750589],
       [ 0.03354202,  0.99933519, -0.01428699,  0.09105917],
       [ 0.02143718, -0.01501112, -0.9996575 ,  0.0831595 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


G_AL_DIG_1 = np.array([[-0.81119025, -0.16395576, -0.56132778, -0.1401922 ],
       [-0.22736393,  0.97279576,  0.04443027,  0.03417519],
       [ 0.53877269,  0.16366709, -0.82640007,  0.34344615],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_DIG_2 = np.array([[-0.81674723, -0.04052531, -0.57557072, -0.16189517],
       [-0.10347557,  0.99164598,  0.07701335,  0.05287688],
       [ 0.5676414 ,  0.12245794, -0.81411749,  0.1677779 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

G_AL_DIG_3 = np.array([[-0.99187391, -0.0805536 , -0.09847467,  0.02518267],
       [-0.09458515,  0.98455644,  0.1473169 ,  0.07614999],
       [ 0.08508696,  0.15543403, -0.98417502,  0.15460154],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
def open_cam(camera, res):
    # Check if valid resolution
    if not any((res[0] == r[0] and res[1] == r[1])for r in CameraController.MODES):
        rospy.logerr("Invalid resolution provided.")
        # Open camera
        cam = CameraController(camera)
        # Create camera object
        cam.resolution = res
        # Set resolution
        cam.open()  
        # open# Close camera
def close_cam(camera):
    cam = CameraController(camera)  # Create camera object
    cam.close() # close

# AR_MARKER_LIST = [0, 1, 2, 3, 4]
AR_MARKER_LIST = [0, 1, 2, 3]

def get_r_from_quaternion(quaternion):
    quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    r = R.from_quat(quaternion)
    return r

def get_quaternion_from_r(r):
    r_quat = r.as_quat()
    quaternion = Quaternion()
    quaternion.x = r_quat[0]
    quaternion.y = r_quat[1]
    quaternion.z = r_quat[2]
    quaternion.w = r_quat[3]

    return quaternion

# takes in Transform object (not TransformStamped), returns 4x4 homogenous transform matrix
def make_homog_from_transform(transform): 
    quaternion = transform.rotation
    translation = transform.translation
    # rpy =  euler_from_quaternion(quaternion) #given in native TF API
    quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    translation = [translation.x, translation.y, translation.z]
    r = R.from_quat(quaternion)
    rot_mat = r.as_dcm()

    homog = np.zeros((4, 4))
    homog[:3, :3] = np.array(rot_mat)
    homog[3] = np.array([0, 0, 0, 1])
    homog[:3, 3] = np.array(translation).T
    return homog

def get_pose_from_homog(g_ba, g_al):
    g_bl = np.dot(g_ba, g_al)
    pose = Pose()
    r = R.from_dcm(g_bl[:3, :3])
    quat = r.as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose.position.x = g_bl[0][3]
    pose.position.y = g_bl[1][3]
    pose.position.z = g_bl[2][3]
    return pose


class Baxter:
    def camera_callback(self, image):
        self.image = image

    def ar_pose_callback(self, alvar_markers):
        for marker in alvar_markers.markers:
            marker_id = marker.id
            if not self.scan_again:
                continue
            if marker_id not in AR_MARKER_LIST:
                continue
            if self.ar_marker_positions[marker_id] == 0:
                pose = marker.pose.pose 
                self.ar_marker_positions[marker_id] = pose
        self.scan_again = False

    def setup_left_hand_camera(self):
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = 0.737
        pose.pose.position.y = 0.219
        pose.pose.position.z = 0.107
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 1
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        orientation_constraints = []
        plan = self.plan(pose, orientation_constraints, "left")
        raw_input("Press <Enter> to move the camera to initial pose: ")
        self.execute(plan, "left")

    def setup_table_obstacle(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.3
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0

        self.left_planner.add_box_obstacle(np.array([0.40, 1.20, 0.10]), "table", table_pose)

    def remove_table_obstacle(self):
        self.left_planner.remove_obstacle("table")

    def calibrate_gripper(self):
        self.left_gripper.calibrate()

    def __init__(self):
        self.image = None
        self.ar_marker_positions = [0] * len(AR_MARKER_LIST)

        self.left_planner = PathPlanner("left_arm")

        rospy.init_node('Baxter')
        # self.remove_table_obstacle()
        self.setup_table_obstacle()

        self.scan_again = True
        rospy.Subscriber('cameras/right_hand_camera/image', Image, self.camera_callback)

        while(self.image == None):
            continue
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_callback)
        while(self.ar_marker_positions == None):
            continue
        self.left_gripper = robot_gripper.Gripper('left')
        self.left_gripper.open()

        self.left_gripper.calibrate()
        rospy.sleep(2.0)

        self.bridge = cv_bridge.CvBridge()
        print("Finished init")

    def get_image(self):
        return self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")

    def get_left_hand_pose(self):
        return self.lookup_transform("base", "left_gripper").transform

    def get_ar_pose(self, ar_tag_id):
        #Get pose of ar_tag with respect to base frame
        return self.ar_marker_positions[ar_tag_id]

    def close_gripper(self):
        self.left_gripper.close()

    def open_gripper(self):
        self.left_gripper.open()

    def rescan(self):
        raw_input("Press enter to scan again")
        self.scan_again = True

    def change_velocity(self, scaling_factor):
        self.left_planner.change_velocity(scaling_factor)

    def plan(self, target, orientation_constraints, arm):
        if arm == "left":
            return self.left_planner.plan_to_pose(target, orientation_constraints)
        else:
            return self.right_planner.plan_to_pose(target, orientation_constraints)

    def execute(self, plan, arm):
        if arm == "left":
            return self.left_planner.execute_plan(plan)
        else:
            return self.right_planner.execute_plan(plan)

    def move_to_pose(self, pose, orientation_constraints, gripper):
        success = False
        while not success:
            plan_found = False
            while not plan_found:
                try:
                    plan = self.plan(pose, orientation_constraints, gripper)
                    plan_found = len(plan.joint_trajectory.points) > 0
                except:
                    print("Plan not found, continuing to search")
                if not plan_found:
                    print("Plan not found, continuing to search")
            raw_input("Press enter to move to pose")
            success = self.execute(plan, gripper)

    def lookup_transform(self, target_frame, source_frame):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        r = rospy.Rate(10) # 10hz
        done = False
        trans = None
        while not rospy.is_shutdown() and not done:
            try:
                trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
                done = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("Exception: {}".format(e))
            r.sleep()

        return trans # of type TransformStamped

    def move_forward(self, forward_amount):
        target_trans = self.get_left_hand_pose()
        target_pose = Pose()
        target_pose.orientation = target_trans.rotation
        target_pose.position = target_trans.translation
        current_orientation = target_pose.orientation
        target_pose.position.x = target_pose.position.x + forward_amount
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "left_gripper"
        orientation_constraint.header.frame_id = "base"
        orientation_constraint.orientation = current_orientation
        orientation_constraint.absolute_x_axis_tolerance = .05
        orientation_constraint.absolute_y_axis_tolerance = .05
        orientation_constraint.absolute_z_axis_tolerance = .05

        return self.plan(target_pose, [orientation_constraint], "left")

    def move_up(self, up_amount):
        target_trans = self.get_left_hand_pose()
        target_pose = Pose()
        target_pose.orientation = target_trans.rotation
        target_pose.position = target_trans.translation
        current_orientation = target_pose.orientation
        target_pose.position.z = target_pose.position.z + up_amount
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "base"
        target_pose_stamped.pose = target_pose
        return self.plan(target_pose_stamped, [], "left")
            
    def test(self):
        print("Test")
        gripper_to_tag = self.lookup_transform("ar_marker_3", "left_gripper")
        g_al = make_homog_from_transform(gripper_to_tag.transform)
        print("GAL: ", g_al)
        return



if __name__ == '__main__':
    b = Baxter()
    b.test()
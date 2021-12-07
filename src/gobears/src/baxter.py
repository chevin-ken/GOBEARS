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

# Open camera(camera is a string and res is a2-elementvector)
#g_al_setup: transform from ar tag to l gripper at setup stage, before descending
# G_AL_SETUP = np.array([[ 0.00179543,  0.99388374,  0.11041685, -0.00250071],
#  [ 0.98573788, -0.02033984,  0.16705428, -0.06594194],
#  [ 0.16827839,  0.10854214, -0.97974537,  0.03593001],
#  [ 0.,          0.,          0.,          1.        ]])


G_AL_SETUP = np.array([[ 1.74885426e-02,  9.97841627e-01,  6.32948526e-02, -3.70693018e-04],
 [ 9.82872667e-01, -2.87724216e-02,  1.82026011e-01, -4.03238486e-02],
 [ 1.83454277e-01,  5.90274110e-02, -9.81254449e-01,  3.27414226e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]
)


#g_al_final: transform from ar tag to l gripper at final config (about to close grip)
# G_AL_FINAL = np.array([[-1.46622016e-02,  9.97445744e-01,  6.99071311e-02, -6.92087378e-05],
#  [ 9.86086809e-01,  2.84643318e-03,  1.66206805e-01, -6.41808053e-02],
#  [ 1.65583284e-01,  7.13714576e-02, -9.83609827e-01, -2.28385614e-02],
#  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

G_AL_FINAL = np.array([[ 0.04773602,  0.99882483,  0.00838042,  0.00168912],
 [ 0.95533287, -0.04810397,  0.29159067, -0.05074548],
 [ 0.29165113, -0.00591329 ,-0.95650648, -0.05208556],
 [ 0.,          0. ,         0.,          1.        ]]
)



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
AR_MARKER_LIST = [0, 1, 2]

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
        if self.ar_marker_positions == None:
            done = False
            self.ar_marker_positions = [0] * len(AR_MARKER_LIST)
            seen = set()
            check = set(AR_MARKER_LIST)
            while not done:
                for marker in alvar_markers.markers:
                    marker_id = marker.id
                    pose = marker.pose.pose
                    self.ar_marker_positions[marker_id] = pose
                    seen.add(marker_id)
                if seen == check:
                    done = True
            print("Done retrieving initial ar tag positions")

    def setup_left_hand_camera(self):
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = 0.595
        pose.pose.position.y = 0.133
        pose.pose.position.z = -0.086
        pose.pose.orientation.x = 0.996
        pose.pose.orientation.y = 0.067
        pose.pose.orientation.z = -0.025
        pose.pose.orientation.w = -0.046
        orientation_constraints = []
        plan = self.plan(pose, orientation_constraints, "left")
        raw_input("Press <Enter> to move the camera to initial pose: ")
        self.execute(plan, "left")

    def __init__(self):
        self.image = None
        self.ar_marker_positions = None

        self.left_planner = PathPlanner("left_arm")
        self.right_planner = PathPlanner("right_arm")
        # self.setup_left_hand_camera()

        rospy.init_node('Baxter')
        # rospy.Subscriber('cameras/left_hand_camera/image', Image, self.left_hand_camera_callback)
        rospy.Subscriber('cameras/right_hand_camera/image', Image, self.camera_callback)

        while(self.image == None):
            continue
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_callback)
        while(self.ar_marker_positions == None):
            continue
        print(self.ar_marker_positions)
        self.left_gripper = robot_gripper.Gripper('left')
        self.left_gripper.calibrate()
        rospy.sleep(2.0)
        self.left_gripper.close()
        self.left_gripper.open()
        rospy.sleep(1.0)

        self.bridge = cv_bridge.CvBridge()
        print("Finished init")

    def get_image(self):
        return self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")

    def get_left_hand_pose(self):
        return self.lookup_transform("base", "left_gripper").transform

    def get_ar_pose(self, ar_tag_id):
        #Get pose of ar_tag with respect to base frame
        return self.ar_marker_positions[ar_tag_id]

    def close_gripper():
        return

    def release_gripper():
        return

    def scoop():
        return

    def pour():
        return

    def reset():
        return
        
    # def detect_color_object(self, color):  
    #     head_point, hand_point = self.detector.detect(color)
    #     head_pose = self.get_head_orientation()
    #     hand_pose = self.get_hand_orientation()
    #     return self.calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose)

    def plan(self, target, orientation_constraints, arm):
        if arm == "left":
            return self.left_planner.plan_to_pose(target, orientation_constraints)
        return self.right_planner.plan_to_pose(target, orientation_constraints)

    def execute(self, plan, arm):
        if arm == "left":
            return self.left_planner.execute_plan(plan)
        else:
            return self.right_planner.execute_plan(plan)




    #  TODO: refactor below (or any appropriate) functions to be in a separate file with tools
    # returns TransformStamped object
    def lookup_transform(self, target_frame, source_frame):
        #TODO: aren't target and source frame names swapped?
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        # tfListener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
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
        # orientation_constraint = OrientationConstraint()
        # orientation_constraint.link_name = "left_gripper"
        # orientation_constraint.header.frame_id = "base"
        # orientation_constraint.orientation = current_orientation
        # orientation_constraint.absolute_x_axis_tolerance = .05
        # orientation_constraint.absolute_y_axis_tolerance = .05
        # orientation_constraint.absolute_z_axis_tolerance = .05
        return self.plan(target_pose, [], "left")

    def move_to_drop_off(self):
        return
        # curr_pose = get_current_pose()
        # target_pose = get_pose_from_ar_tag(deposit_ar_tag)
        # orientation_constraint = current_orientation
        # move(curr_pose, target_pose)



            
    def test():
        b = Baxter()
        gripper_to_tag = b.lookup_transform("ar_marker_2", "left_gripper")
        g_al = make_homog_from_transform(gripper_to_tag.transform)
        print(g_al)
        return



if __name__ == '__main__':
    test()
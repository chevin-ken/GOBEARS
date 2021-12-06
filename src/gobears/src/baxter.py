from planner import PathPlanner
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface.camera import CameraController
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper
from geometry_msgs.msg import PoseStamped, Pose

# Open camera(camera is a string and res is a2-elementvector)

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
AR_MARKER_LIST = [0]

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
        self.left_gripper = robot_gripper.Gripper('left')
        self.left_gripper.calibrate()
        rospy.sleep(2.0)
        self.left_gripper.close()
        rospy.sleep(1.0)

        self.bridge = cv_bridge.CvBridge()
        print("Finished init")

    def get_image(self):
        return self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")

    def get_left_hand_orientation(self):
        return None

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
        
        
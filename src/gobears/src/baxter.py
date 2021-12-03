from planner import PathPlanner
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_interface.camera import CameraController
from baxter_interface import Limb
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

TOTAL_AR_MARKERS = 20

class Baxter:
    def right_hand_camera_callback(self, image):
        self.image = image

    def ar_pose_callback(self, alvar_markers):
        if self.ar_marker_positions == None:
            self.ar_marker_positions = [0] * TOTAL_AR_MARKERS
        for marker in alvar_markers.markers:
            marker_id = marker.id
            pose = marker.pose.pose
            self.ar_marker_positions[marker_id] = pose

    def __init__(self):
        open_cam('right_hand_camera', [1280,800])
        self.image = None
        rospy.init_node('Right_camera_subscriber')
        rospy.Subscriber('cameras/right_hand_camera/image', Image, self.right_hand_camera_callback)
        while(self.image == None):
            continue
        self.ar_marker_positions = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_callback)
        while(self.ar_marker_positions == None):
            continue
        #Initialize path planner for left arm
        self.planner = PathPlanner("left_arm")

        self.bridge = cv_bridge.CvBridge()
        print("Finished init")

    def get_image(self):
        return self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")

    def get_left_hand_orientation(self):
        return None

    def get_ar_pose(self, ar_tag_id):
        #Get pose of ar_tag with respect to base frame
        return self.ar_marker_positions[ar_tag_id]

    # def detect_color_object(self, color):  
    #     head_point, hand_point = self.detector.detect(color)
    #     head_pose = self.get_head_orientation()
    #     hand_pose = self.get_hand_orientation()
    #     return self.calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose)

    def execute(self, target, orientation_constraints):
        plan = self.planner.plan_to_pose(target, orientation_constraints)
        return self.planner.execute_plan(plan)
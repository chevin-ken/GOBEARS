from planner import PathPlanner
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController
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

class Baxter:
    def right_hand_camera_callback(self, image):
        self.image = image

    def __init__(self):
        #
        print('here')
        open_cam('right_hand_camera', [1280,800])
        self.image = None
        rospy.init_node('Right_camera_subscriber')
        rospy.Subscriber('cameras/right_hand_camera/image', Image, self.right_hand_camera_callback)
        while(self.image == None):
            a = 1+1
        print('hi')
        #Initialize head camera subscriber
        self.right_hand_camera = None
        #Initialize hand camera subscriber
        self.left_hand_camera = None
        #Initialize cv2 object detector
        self.detector = None
        #Initialize path planner for right arm
        #self.planner = PathPlanner("right_arm")

        self.bridge = cv_bridge.CvBridge()

    def get_image(self):
        return self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")

    def get_right_hand_orientation(self):
        return None

    def get_left_hand_orientation(self):
        return None

    def calculate_left_hand_translation(self, ar_tag):
        return None

    def detect_color_object(self, color):  
        head_point, hand_point = self.detector.detect(color)
        head_pose = self.get_head_orientation()
        hand_pose = self.get_hand_orientation()
        return self.calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose)

    def execute(self, target, orientation_constraints):
        plan = self.planner.plan_to_pose(target, orientation_constraints)
        return self.planner.execute(self.planner.execute(plan))
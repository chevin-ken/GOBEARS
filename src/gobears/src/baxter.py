from planner import PathPlanner

class Baxter:
    def __init__(self):
        #Initialize head camera subscriber
        self.head_camera = None
        #Initialize hand camera subscriber
        self.hand_camera = None
        #Initialize cv2 object detector
        self.detector = None
        #Initialize path planner for right arm
        self.planner = PathPlanner("right_arm")

    def get_head_orientation(self):
        return None

    def get_hand_orientation(self):
        return None

    def calculate_world_frame_coordinates(self, head_point, head_pose, hand_point, hand_pose):
        return None

    def detect_color_object(self, color):  
        head_point, hand_point = self.detector.detect(color)
        head_pose = self.get_head_orientation()
        hand_pose = self.get_hand_orientation()
        return self.calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose)

    def execute(self, target, orientation_constraints):
        plan = self.planner.plan_to_pose(target, orientation_constraints)
        return self.planner.execute(self.planner.execute(plan))
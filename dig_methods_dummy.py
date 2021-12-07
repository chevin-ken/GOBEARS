def move_forward():
	curr_pose = get_current_pose()
	target_pose = curr_pose.x + forward_amount
	orientation_constraint = current_orientation
	move()

def move_up():
	curr_pose = get_current_pose()
	target_pose = curr_pose.z + up_amount
	orientation_constraint = current_orientation
	move()

def move_to_drop_off()
	curr_pose = get_current_pose()
	target_pose = get_pose_from_ar_tag(deposit_ar_tag)
	orientation_constraint = current_orientation
	move(curr_pose, target_pose)
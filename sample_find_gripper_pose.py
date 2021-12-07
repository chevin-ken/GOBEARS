#note: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html

def make_homog_from_transform():
	tf.RPY_from_quaternion() #given in native TF API
	make_rotation_from_RPY() #see http://planning.cs.uiuc.edu/node102.html
	#note: order of roll pitch yaw when making rotation matrix

	#to make homog matrix from rotation and translation by:
	#g = [[r, p], rotation = r, translation = p. just make the matrix
	#	  [0, 1]]
	return make_homog_from_rotation_translation()

# let b = base/body frame
# let l = left hand gripper frame
# let a = ar tag frame
#Goal: to find the for left gripper to enter relative to any orientation of an AR tag to be able to pick up the trowel
# since all poses are defined relative to body frame, we want transform of left_gripper relative to body
# we can find this if we know the homogenous transform matrix g_bl
# we can find g_bl by doing g_ba @ g_al
# g_al is pre-defined by positioning the gripper relative to a test ar_tag position
# g_ba should be found for the ar tag at runtime by getting ar tag pose
# once we found g_bl, convert back into a pose by deconstructing it back into quaternions and translation
def hand_pose_relative_to_ar_for_trowel():
	#find g_al:
	ar_hand_transform = tf.request_transform(test_artag, left_gripper)
	g_al = make_homog_from_transform(ar_hand_transform)

	#finding g_ba:
	body_ar_transform = tf.request_transform(base, ar_tag_n)
	g_ba = make_homog_from_transform(body_ar_transform)

	#make Homog transform matrix for rotation by RPY and translation by pose.position
	g_bl = np.dot(g_ba, g_al)
	target_pose = Pose()
	Pose.position = get_translation(g_bl)
	target_RPY = get_RPY_from_rotation(g_bl)
	target_quaternion = tf.quaternion_from_euler(target_RPY)
	target.orientation = target_quaternion

return target
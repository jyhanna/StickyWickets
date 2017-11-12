#!/usr/bin/env python
import roslib
import sys
import rospy
import numpy as np
import tf
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid

IMAGE_WIDTH = 640
IMAGE_TARGET = 320
DIST_COL_FROM_TARGET = IMAGE_WIDTH - IMAGE_TARGET
MAX_TURN_RATE = 0.785

class SignHandler:
	"""
	Handles signs from object detector. Rensibilities include publishing
	reactions to signs on /sticky_wickets/sign/move_goal, as well as sign
	landmark map integration on topic /sticky_wickets/sign/map. Squares
	are landmarks, triangles are hazards, and circles are stop-and-turn
	signs. Currently, triangles and circles exhibit the same behavior.
	Reactions occur when a certain distance to the sign of interest is
	achieved (this analysis uses /camera/depth/image_raw).
	"""
	def __init__(self):
		self.depth_image_data = None
		self.bridge = CvBridge()
		self.signs = []
		self.listener = tf.TransformListener()
		self.map_meta = None
		self.sign_map = None

		self.react_pub = rospy.Publisher("/sticky_wickets/sign/map", Image, queue_size=10)
		self.move_pub = rospy.Publisher("/sticky_wickets/sign/move_goal", MoveBaseGoal, queue_size=0)

		self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_updated)
		self.shape_sub = rospy.Subscriber("/sticky_wickets/object_data", String, self.shape_updated)
		self.map_grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

		self.map_publisher()

	def depth_updated(self, depth_image):
		"""
		ROS depth image callback that decodes the depth data.
		"""
		depth_image = self.bridge.imgmsg_to_cv2(depth_image, '32FC1')
		self.depth_image_data = np.array(depth_image, dtype=np.float32)

	def shape_updated(self, shapes_data):
		"""
		ROS updated shape callback. Analyzes centroids of shapes,
		determining distance based on decoded image depth data.
		"""
		if self.depth_image_data is not None:
			shapes_dict = eval(shapes_data.data)
			self.handle_signs(shapes_dict)

	def map_callback(self, data):
		self.map_meta = data.info

		if self.map_meta is not None:
			self.sign_map = self.make_sign_map(self.map_meta)

	def handle_signs(self, shapes_dict):
		"""
		Handles reaction to detected signs. If within a certain distance,
		react to the sign. This method is invoked from shape_updated callback.
		"""
		react_dist = 0.7 #this should be smaller, but the astra sucks and the kinect's camera doesn't work with our object detector

		for num_sides, shapes in shapes_dict.items():
			for (x, y, w, h) in shapes:
				shape_object = (x, y, w, h)
				centroid = self.get_centroid(shape_object)

				dist_in_mtrs = self.depth_image_data[centroid[1], centroid[0]] / 1000.0

				if dist_in_mtrs < react_dist and dist_in_mtrs != 0.0:
					if num_sides == "4":
						self.investigate_sign(centroid, dist_in_mtrs)
					elif num_sides == "0":
						self.react_to_stopper()
					elif num_sides == "3":
						self.react_to_hazard()

	def get_centroid(self, sign_object):
		"""
		Calculates the centroid of a detected shapes from the bounding box published on /sticky_wickets/object_data
		"""
		(x, y, w, h) = sign_object

		centroid_x = x + (w/2)
		centroid_y = y + (h/2)

		centroid = (centroid_x, centroid_y)

		return centroid

	def react_to_hazard(self):
		"""
		Publish a reaction to current sign status = hazard ahead
		"""
		desired_movement = (2.0, 0.0) #180 degrees?

		self.pub_move_goal(desired_movement)

	def react_to_stopper(self):
		"""
		Publish a reaction to current sign status = avoid area (stop, turn)
		"""
		desired_movement = (2.0, 0.0) #Not sure what 180 degree turn is

		self.pub_move_goal(desired_movement)

	def pub_move_goal(self, target):
		"""
		Publishes move commands to the /sticky_wickets/sign/move_goal topic
		"""
		move_goal_msg = MoveBaseGoal()

		move_goal_msg.target_pose.header.frame_id = "base_link"
		move_goal_msg.target_pose.header.stamp = rospy.Time.now()

		move_goal_msg.target_pose.pose.position.x = target[1]
		move_goal_msg.target_pose.pose.orientation.z = target[0]
		move_goal_msg.target_pose.pose.orientation.w = 1.0

		rospy.loginfo(target)
		self.move_pub.publish(move_goal_msg)

	def investigate_sign(self, shape_centroid, dist_in_mtrs):
		"""
		Drives to directly in front of square signs in order to approximate their location (proof of concept).
		Functions called sequentially to hopefully avoid command failing (ran into trouble setting rotation and forward movement at the same time)
		"""
		target_col = shape_centroid[0]
		col_dist = IMAGE_TARGET - target_col

		threshold = 20 #this is to prevent oscillation due to overshoot
		row_speed = 0.0

		if abs(col_dist) > threshold:
			col_ang_speed = self.mapAngle(col_dist, DIST_COL_FROM_TARGET, -DIST_COL_FROM_TARGET, MAX_TURN_RATE, -MAX_TURN_RATE)
		else:
			col_ang_speed = 0.0

			if dist_in_mtrs > 0.60:
				row_speed = 0.1
			else:
				rospy.loginfo('Landmark stored')
				self.map_handler(shape_centroid)
				rospy.sleep(1.5) #wait for robot to finish centering

				self.react_to_stopper() #resets the wanderer by making landmark out of view

		desired_movement = (col_ang_speed, row_speed)

		if col_ang_speed != 0 or row_speed != 0: #don't publish if we don't need to move because this hijacks the wanderer
			self.pub_move_goal(desired_movement)

	def mapAngle(self, centroid_x, max_px, min_px, max_ang, min_ang):
		"""
		Proportional controller that maps pixels from target to rotation.
		"""
		fromRange = max_px - min_px
		trueVal = centroid_x - min_px
		proportion = trueVal/float(fromRange)
		toRange = max_ang - min_ang
		rawToVal = proportion * toRange
		toVal = rawToVal + min_ang

		return toVal

	def convert_pos_to_cell(self, position):
		"""
		Returns occupancy grid space of pose
		"""
		pos_grid_x = int((position[0] - self.map_meta.origin.position.x) / self.map_meta.resolution)
		pos_grid_y = int((position[1] - self.map_meta.origin.position.y) / self.map_meta.resolution)

		return [pos_grid_x, pos_grid_y]

	def make_sign_map(self, map_data):
		"""
		Generates white image of equal size to gmapping map and returns the image.
		"""
		blank_map = np.zeros((self.map_meta.height, self.map_meta.width, 3), np.uint8) #gens black image
		blank_map.fill(255)	#makes image white

		return blank_map

	def integrate_into_map(self, grid_location):
		"""
		Integrates shape onto map
		"""
		(x, y) = (grid_location[0], grid_location[1])
		color = (0, 0, 255) # (B, G, R)

		sign_size = 3 #fills in more than one pixel so it can be seen (testing would be needed for actual product)

		y = self.map_meta.height - y #numpy indexes from top not bottom, so we have to invert it

		self.sign_map[y-sign_size:y+sign_size, x - sign_size:x + sign_size] = color #ros is (y, x), not (x, y)

	def map_handler(self, centroid):
		"""
		Adds sign location (converts pose to occupancy grid equivalent to map being navigated) to landmark map
		"""
		(curr_pos,_) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0)) #get pose of robot in map frame

		grid_cell = self.convert_pos_to_cell(curr_pos)

		if grid_cell not in self.signs: #want to do tolerance here, but not easy
			self.signs.append(grid_cell)
			self.integrate_into_map(grid_cell)

	def map_publisher(self):
		"""
		Publishes map to map topic so it can be viewed in image viewer
		"""
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			if self.sign_map is not None:
				ros_img = self.bridge.cv2_to_imgmsg(self.sign_map, 'bgr8')
				self.react_pub.publish(ros_img)

			rate.sleep()

def main(args):
	rospy.init_node('sign_handler', anonymous=True)
	sign_handler = SignHandler()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)

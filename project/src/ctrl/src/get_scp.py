import sys
import os
import rospy

remotehost = "douglash@192.168.2.118"
localfile1, localfile2 = "~/scp_board_state/state_1.txt", "~/scp_board_state/state_2.txt"
remotefile1, remotefile2 = "~/board_state/state_1.txt", "~/board_state/state_2.txt"

def main():
	rospy.init_node("scp_board_state")
	r = rospy.Rate(2)

	pub = rospy.Publisher("pente_ctrl/board_state", String, queue_size=10)

	while not rospy.is_shutdown():
		os.system('scp "%s" "%s:%s"' % (localfile1, remotehost, remotefile1))
		os.system('scp "%s" "%s:%s"' % (localfile2, remotehost, remotefile2))

		f1 = open(localfile1, 'r')
		f2 = open(localfile2, 'r')

		s1 = f1.readline()[:-1]
		s2 = f2.readline()[:-1]

		f1.close()
		f2.close()

		if len(s1) > len(s2):
			pub.publish(data=s1)
		else:
			pub.publish(data=s1)

		r.sleep()


if __name__ == '__main__':
	main()
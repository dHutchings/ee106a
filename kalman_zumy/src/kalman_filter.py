#!/usr/bin/env python
import rospy
import numpy as np
from numpy.linalg import inv
import sys

from geometry_msgs.msg import Transform,TransformStamped
from kalman_zumy.srv import ImuSrv,ImuSrvResponse,NuSrv,NuSrvResponse
from tf2_msgs.msg import TFMessage

class KalmanFilter:
  def __init__(self,mname):

    #Initialize the node
    rospy.init_node('kalman_filter_' + mname)
    self.hertz = 10
    self.dt = 1./self.hertz
    self.rate = rospy.Rate(self.hertz)
    self.calTime = 5 # Calibration time in seconds
    self.initial_position_uncertainty = 0.10 # meters
    self.initial_orientation_uncertainty = 0.3 # radians (0.3rad ~ 15deg)
    self.camera_position_error = 0.05 # meters
    self.camera_orientation_error = 0.1 # radians (0.1rad ~ 5deg)
    self.C_lin = np.array([[1.,0.,0.,0.],[0.,0.,1.,0]]) # position camera measurement
    self.C_ang = 1. # orientation camera measurement
    self.G_lin = np.array([[0.5*pow(self.dt,2),self.dt,0.,0.],
                          [0.,0.,0.5*pow(self.dt,2),self.dt]]).T # position IMU input
    self.G_ang = self.dt # orientation IMU input
    self.A_lin = np.array([[1.,self.dt,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,self.dt],[0.,0.,0.,1.]]) # translation dynamics
    self.A_ang = 1. # rotation dynamics
    self.updateFlag = True
    self.mname = mname

    # Measurement initialization -- only necessary temporarily while AR does not work
    self.u = None
    self.z = Transform()
    self.origin_tag = 'usb_cam'
    
    #Create a publisher to the state estimation topic
    self.state_pub = rospy.Publisher('/' + mname + '/state_estimate', Transform,queue_size=2)
    self.state_pub_tf = rospy.Publisher('/tf', TFMessage,queue_size=2)

    #Create the service for the AR tag client
    rospy.Service('innovation', NuSrv, self.triggerUpdate)


  # Compute the time update for every time step based on measured variations
  def timeUpdate(self,u):
    # Define inputs
    u_lin = np.array([u.linear_acceleration_filtered.x,u.linear_acceleration_filtered.y]) - self.acc_bias
    u_ang = u.angular_velocity_filtered.z - self.gyro_bias
    # Determine orientation
    rot = np.array([[np.cos(self.psi),-np.sin(self.psi)],[np.sin(self.psi),np.cos(self.psi)]])
    rotxv = np.kron(rot,np.eye(2))
    # Propagate dynamics
    self.x_lin += self.v_lin*self.dt + 0.5*rot.dot(u_lin)*pow(self.dt,2)
    self.v_lin += rot.dot(u_lin)*self.dt
    self.psi += u_ang*self.dt
    # Update uncertainty
    self.P_lin = self.A_lin.dot(self.P_lin).dot(self.A_lin.T)  \
                 + rotxv.dot(self.Q_lin).dot(rotxv.T)
    self.P_ang += self.Q_ang
 

  # Compute a measurement update based on the received information
  def measurementUpdate(self,z):
    # Compute innovation
    e_lin = np.array([z.translation.x, z.translation.y]) - self.x_lin
    e_ang = z.rotation.w - self.psi
    S_lin  = self.C_lin.dot(self.P_lin).dot(self.C_lin.T) + self.R_lin
    S_ang  = self.C_ang*self.P_ang*self.C_ang + self.R_ang
    # Compute Kalman gain
    K_lin = self.P_lin.dot(self.C_lin.T).dot(inv(S_lin))
    K_ang = self.P_ang*self.C_ang/S_ang
    # Update state
    xv = np.array([self.x_lin[0],self.v_lin[0],self.x_lin[1],self.v_lin[1]])
    xv += K_lin.dot(e_lin)
    self.x_lin = np.array([xv[0],xv[2]])
    self.v_lin = np.array([xv[1],xv[3]]) # np.array([0,0]) # 
    self.psi += K_ang*e_ang
    # Update uncertainty
    self.P_lin = (np.eye(4)-K_lin.dot(self.C_lin)).dot(self.P_lin)
    # Reset flag
    self.updateFlag = False


  # When another node calls the service sending a measurement, incorporate it
  def triggerUpdate(self,request):
    self.z = request.transform
    self.origin_tag = request.origin_tag
    # We will ignore the time stamp and assume that the AR fix is from the latest time step
    self.updateFlag = True
    return []

  # Calibrate sensors to determine bias and variance
  def calibrateSensors(self):
    startCal = rospy.get_rostime() # node time in seconds
    print "Starting sensor calibration..."
    m = np.empty([6,int(self.hertz*self.calTime)])
    for j in range(m.shape[1]):
        rospy.wait_for_service('last_imu')
        try:
            get_imu = rospy.ServiceProxy('last_imu', ImuSrv)
            u = get_imu()
            m[:,j] = [u.linear_acceleration.x,u.linear_acceleration.y,u.linear_acceleration.z,
                      u.angular_velocity.x, u.angular_velocity.y, u.angular_velocity.z]
        except rospy.ServiceException, e:
            #print "Service call to IMU Server failed: %s"%e
            print "No IMU update this time step"
            m[:,j] = np.zeros(6)  
        self.rate.sleep()
    endCal = rospy.get_rostime()
    print "Calibration complete. Took %f seconds"%(endCal-startCal).to_sec()
    Q = np.cov(m,bias=1)
    mu = np.mean(m,axis=1)
    print "Average accelerometer measurement: [%f, %f, %f]"%(mu[0], mu[1], mu[2])
    print "Accelerometer covariance matrix:"
    print Q[:3,:3]
    print "Average gyroscope measurement: [%f, %f, %f]"%(mu[3], mu[4], mu[5])
    print "Gyroscope covariance matrix:"
    print Q[3:,3:]
    return (Q,mu)


  # Main node execution function
  def run(self):

    counter = 0;

    # set default Q and mu values to eye(6) and zero
    Q = np.eye(6)
    mu = np.zeros(6)

    # Initial sensor calibration
    Q, mu = self.calibrateSensors()

    # Initialize Kalman filter
    self.P_lin = np.diag([1.,0.,1.,0.])*pow(self.initial_position_uncertainty,2)
    self.Q_lin = self.G_lin.dot(Q[:2,:2]).dot(self.G_lin.T)
    self.P_ang = pow(self.initial_orientation_uncertainty,2)
    self.Q_ang = self.G_ang*Q[5,5]*self.G_ang
    self.R_lin = np.eye(2)*self.camera_position_error
    self.R_ang = self.camera_orientation_error
    self.acc_bias = mu[:2]
    self.gyro_bias = mu[5]
    self.x_lin = np.array([self.z.translation.x, self.z.translation.y])
    self.v_lin = np.array([0.,0.])
    self.psi = 2*np.arccos(self.z.rotation.w)*np.sign(self.z.rotation.z) # Assume we have a quaternion with vertical axis

    # Run Kalman filter
    while not rospy.is_shutdown():
      # Obtain IMU measurement
      rospy.wait_for_service('last_imu')
      try:
        get_imu = rospy.ServiceProxy('last_imu', ImuSrv)
        self.u = get_imu()
      except rospy.ServiceException, e:
        #print "Service call to IMU Server failed: %s"%e
        print "No IMU update this time step"
        # Assume previous measured u (Zero-Order Hold)

      # Perform time and measurement updates as appropriate
      self.timeUpdate(self.u)
      if self.updateFlag:
        self.measurementUpdate(self.z)

      # Publish state estimate in topic
      state = Transform()
      state.translation.x = self.x_lin[0]
      state.translation.y = self.x_lin[1]
      state.rotation.z = 1 
      state.rotation.w = self.psi # Quaternion form
      self.state_pub.publish(state)

      state_tf = TFMessage()
      state_tf.transforms = [TransformStamped()]
      state_tf.transforms[0].header.seq = counter
      state_tf.transforms[0].header.frame_id = self.origin_tag
      state_tf.transforms[0].child_frame_id = self.mname
      state_tf.transforms[0].transform = state
      self.state_pub_tf.publish(state_tf)

      counter = counter + 1
            # Finish cycle and loop
      self.rate.sleep()
      

#Python's syntax for a main() method
if __name__ == '__main__':
    mname = sys.argv[1]
    node = KalmanFilter(mname)
    node.run()
import rospy
from std_msgs.msg import Float32, Bool
from hover_bringup.srv import SetOutputInts, SetOutputIntsResponse, SetOutputIntsRequest
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty, EmptyRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

class HoverStatemachine:

    def __init__(self):        
        # SUBSCRIBERS
        self.weight_sub = rospy.Subscriber("weight", Float32, self.weightCB)
        self.button_sub = rospy.Subscriber("button_pressed", Bool, self.buttonCB)

        # PUBLISHER
        self.init_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)


        # SERVICE PROXIES
        self.output_service_proxy = rospy.ServiceProxy('set_output_ints', SetOutputInts)
        self.tare_weight_proxy = rospy.ServiceProxy('tare_weight', Trigger)
        self.stop_laser = rospy.ServiceProxy('stop_motor', Empty)
        self.start_laser = rospy.ServiceProxy('start_motor', Empty)

        # INTERNAL STATE
        self.empty = True
        self.wait_time_after_loading = 3.0
        self.min_weight = 0.2
        self.update_rate = 10
        self.driving = False

        # MOVEBASE
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # UPDATE LOOP
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update)

        # TEST POSES
        self.load_pose = Pose() # -1.25, 3
        self.load_pose.position.x = -1.156
        self.load_pose.position.y = 2.995
        self.load_pose.orientation.z = -0.534
        self.load_pose.orientation.w = 0.845
        self.deliver_pose = Pose() 
        self.deliver_pose.position.x = 0.446
        self.deliver_pose.position.y = -0.316
        self.deliver_pose.orientation.z = 0.932
        self.deliver_pose.orientation.w = 0.362

        self.DRIVE_STATE = 'LOAD'
        self.goal_sent = False
        self.previous_button_event = False
        rospy.wait_for_service('stop_motor')
        self.stop_laser.call(EmptyRequest())
        self.start_laser.call(EmptyRequest())

    def weightCB(self, data):
        if data.data > self.min_weight:
            self.empty = False
        else:
            self.empty = True
        self.weight = data.data

    def buttonCB(self, data):
        if data.data == True and self.previous_button_event == False:
            # Tare the weight and set initial Pose of robot
            req = TriggerRequest()
            self.tare_weight_proxy.call(req)
            initpose = PoseWithCovarianceStamped()
            initpose.header.frame_id = "map"
            initpose.pose.pose = self.load_pose
            initpose.pose.covariance[0] = 0.25
            initpose.pose.covariance[7] = 0.25
            initpose.pose.covariance[35] = 0.06853891945200942
            self.init_pose_pub.publish(initpose)
        self.previous_button_event = data.data

    def setStatusLEDs(self, val):
        req = SetOutputIntsRequest()
        req.output_names = ['BACK_LED_L', 'BACK_LED_R']
        req.vals = [val, val]
        self.output_service_proxy(req)

    def sendPose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w
        self.goal_sent = True
        self.client.send_goal(goal)

    def update(self, event):
        if (self.goal_sent and self.client.get_result() == 1):
            self.driving = True
        else:
            self.driving = False
        if self.empty == True:
            self.setStatusLEDs(2)
            if self.driving == False and self.DRIVE_STATE != 'LOAD':
                rospy.sleep(1.5)
                self.driving = True
                self.DRIVE_STATE = 'LOAD'
                # Send Drive Goal To robot
                self.sendPose(self.load_pose)
        else:
            self.setStatusLEDs(1)
            if self.driving == False and self.DRIVE_STATE != 'DELIVER':
                rospy.sleep(1.5)
                self.driving = True
                self.DRIVE_STATE = 'DELIVER'
                # Send Drive Goal To robot
                self.sendPose(self.deliver_pose)
                


        
##############################################################################
# Imports
##############################################################################

from typing import Any
import py_trees
from py_trees.common import Status
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from nav2_msgs.srv import LoadMap, ClearEntireCostmap, ManageLifecycleNodes, GetCostmap

# loadmap
from nav2_msgs.srv import LoadMap

# initial pose
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import String

##############################################################################
# Behaviours
##############################################################################
class Finding_Ball(py_trees.behaviour.Behaviour):
    '''
    Tree for finding ball
    '''
    def __init__(self,name: str,topic_name: str="/behavior/finding_ball",):
        super(Finding_Ball, self).__init__(name=name)
      
        self.topic_name = topic_name
        self.subscribe_topic = 'ball/detection'   #get bool of ball detection via this topic (should be published from camera node)
        
        self.i = 0

    # def callback_msg(self,msg):
    #     self.msg_callback = msg.is_detected
    #     return  self.msg_callback

    def setup(self, **kwargs):
        '''
        Default setup
        '''
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        # Create Publisher data to behavior's topic
        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,  #Using String as msg type
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        # Create Subscriber to subscribe data of this topic
        # self.subscriber = self.node.create_subscription(msg_type=Ball,topic=self.subscribe_topic ,callback =self.callback_msg,qos_profile=1)

    
    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        self.i += 1
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        
        self.publisher.publish(msg)

        return py_trees.common.Status.RUNNING

        #Conditioning to update tree status
        # try:
        #     ball_detected = self.msg_callback   # try to subscribe msg
        # except:
        #     ball_detected = False
        # if not ball_detected:   
        #     return py_trees.common.Status.RUNNING   # Keep waiting for ball detection
        # else:
        #     return py_trees.common.Status.SUCCESS  # if ball is deteced return Success to do next task

    def terminate(self, new_status: py_trees.common.Status):
        '''
        Default terminate funtion
        '''
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.publisher.publish(std_msgs.String(data=""))
        self.feedback_message = "cleared"

class Navigator(py_trees.behaviour.Behaviour):
    '''
    Tree for finding ball
    '''
    def __init__(self,name: str, goal_pose: list):
    # def __init__(self,name: str,topic_name: str="/behavior/finding_ball",):
        super(Navigator, self).__init__(name=name)
      
      
        self.subscribe_topic = 'ball/detection'   #get bool of ball detection via this topic (should be published from camera node)
        
        self.i = 0

        self.goal_pose = goal_pose

        self.goal_handle = None
        self.result_future = None

        self.get_feedback = False
        self.goal_reject = False
        self.goal_success = False

    # def callback_msg(self,msg):
    #     self.msg_callback = msg.is_detected
    #     return  self.msg_callback

    def setup(self, **kwargs):
        '''
        Default setup
        '''
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # Create Action client
        self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self.node.get_logger().info('start')

    def initialise(self):
        self.node.get_logger().info('initialise')
        self.send_goal()
    
    def update(self) -> py_trees.common.Status:
        # self.send_goal()
        # self.logger.debug("%s.update()" % self.__class__.__name__)

        # self.i += 1
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        
        # self.publisher.publish(msg)

        # return py_trees.common.Status.RUNNING

        # self.send_goal()
        # self.node.get_logger().info('I heard: {0}'.format(self.get_feedback))
        
        # return py_trees.common.Status.RUNNING

        if self.goal_reject:
            return py_trees.common.Status.FAILURE

        if self.goal_success:
            return py_trees.common.Status.SUCCESS

        # if self.get_feedback:
        #     return py_trees.common.Status.RUNNING

        else:
            return py_trees.common.Status.RUNNING
        

        
        
        # self.send_goal()
        

    def terminate(self, new_status: py_trees.common.Status):
        '''
        Default terminate funtion
        '''
        # self.logger.debug(
        #     "{}.terminate({})".format(
        #         self.qualified_name,
        #         "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
        #     )
        # )
        # self.publisher.publish(std_msgs.String(data=""))
        # self.feedback_message = "cleared"

        pass
    
    def send_goal(self):
        # goal_msg = Fibonacci.Goal()
        # goal_msg.order = order

        # self._action_client.wait_for_server()

        # return self._action_client.send_goal_async(goal_msg)
    
        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_pose[0]
        goal_pose.pose.position.y = self.goal_pose[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = self.goal_pose[2]
        goal_pose.pose.orientation.w = self.goal_pose[3]

        goal_msg.pose = goal_pose

        # self._action_client.wait_for_server(timeout_sec=5.0)

        wait = self._action_client.wait_for_server(timeout_sec=5.0)
        if wait == False:
            self.goal_reject = True

        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                                                                   
        # rclpy.spin_until_future_complete(self.node, send_goal_future)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # self.goal_handle = self.send_goal_future.result()
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        # self.node.get_logger().info('{0} :'.format(goal_handle.result))

        if not goal_handle.accepted:
            # self.get_logger().warning('Goal rejected :(')
            self.node.get_logger().info('Goal rejected')
            # self.goal_reject = True
            # return

        # self.get_logger().info('Goal accepted :)')

        self.node.get_logger().info('Goal accepted :')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # return py_trees.common.Status.RUNNING
        self.get_feedback = True

    def feedback_callback(self, feedback_msg):
        # NavigateToPose should have no feedback
        # self.node.get_logger().debug('Received feedback')

        # self.node.get_logger().info('Received feedback')
        self.get_feedback = True

    def get_result_callback(self, result_msg):
        result = result_msg.result().result
        self.node.get_logger().info('Goal success {0}'.format(result.result))
        # result = result.result().result
        self.goal_success = True
        # # Expecting empty result (std_msgs::Empty) for NavigateToPose
        # self.get_logger().debug('Result: {0}'.format(result.result))




        # self.debug('Received action feedback message')
        # self.get_logger().info("Received action feedback message")
        # print('Received action feedback message')

        
class LoadNewMap(py_trees.behaviour.Behaviour):
    '''
    Tree for load map
    '''
    def __init__(self,name: str,map_path: str="/home/nuc/h1.yaml"):
        super(LoadNewMap, self).__init__(name=name)

        self.map_path = map_path
    
    def setup(self, **kwargs):
        '''
        Default setup
        '''
        # self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.client = self.node.create_client(LoadMap, '/map_server/load_map')
        self.client.wait_for_service(timeout_sec=1.0)
    def initialise(self):
        self.load_map()
    
    def update(self) -> py_trees.common.Status:

       
        if self.future.result() is not None:
            if self.future.result().result == 0:
                print('OK')
                return py_trees.common.Status.SUCCESS
            else:
                if self.future.result().result == 1:
                    print('problem: map does not exist.')
                    return py_trees.common.Status.FAILURE
                elif self.future.result().result == 2:
                    print('problem: invalid map data.')
                    return py_trees.common.Status.FAILURE
                elif self.future.result().result == 3:
                    print('problem: invalid map metadata.')
                    return py_trees.common.Status.FAILURE
                elif self.future.result().result == 255:
                    print('problem: undefined failure.')
                    return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING
            
        
    
    def load_map(self):
        # self.client.wait_for_service(timeout_sec=1.0)
        request = LoadMap.Request()
        request.map_url = self.map_path
        self.future = self.client.call_async(request)


class Initialpose(py_trees.behaviour.Behaviour):
    '''
    Tree for load map
    '''
    def __init__(self,name: str, pose: list=[0.0, 0.0, 0.0, 1.0] ): # pose = x(poistion), y(position), z(quarternion), w(quarternion)
        super(Initialpose, self).__init__(name=name)

        self.pose = pose
    
    def setup(self, **kwargs):
        '''
        Default setup
        '''
        # self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher_ = self.node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.subscriber = self.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)

        self.amcl_receive_new_pose = False

    def initialise(self):
        self.amcl_receive_new_pose = False

        self.initial_pose_msg = PoseWithCovarianceStamped()
        self.initial_pose_msg.header.frame_id = 'map'
        # Set the initial pose values (e.g., x, y, z, orientation)
        self.initial_pose_msg.pose.pose.position.x = self.pose[0]
        self.initial_pose_msg.pose.pose.position.y = self.pose[1]
        self.initial_pose_msg.pose.pose.position.z = 0.0
        self.initial_pose_msg.pose.pose.orientation.x = 0.0
        self.initial_pose_msg.pose.pose.orientation.y = 0.0
        self.initial_pose_msg.pose.pose.orientation.z = self.pose[2]
        self.initial_pose_msg.pose.pose.orientation.w = self.pose[3]
        self.initial_pose_msg.pose.covariance = [0.25      , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.25      , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.        , 0.        , 0.        , 0.        , 0.        ,
                                            0.06853892]
    
    def update(self) -> py_trees.common.Status:

        self.publisher_.publish(self.initial_pose_msg)

        if self.amcl_receive_new_pose == True:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def amcl_callback(self, msg):
        self.amcl_receive_new_pose = True

class Waitkey(py_trees.behaviour.Behaviour):
    '''
    Tree for load map
    '''
    def __init__(self,name: str): # pose = x(poistion), y(position), z(quarternion), w(quarternion)
        super(Waitkey, self).__init__(name=name)
    
    def setup(self, **kwargs):
        '''
        Default setup
        '''
        # self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.subscriber = self.node.create_subscription(PoseWithCovarianceStamped, 'wait_key', self.amcl_callback, 10)

        self.amcl_receive_new_pose = False

    def initialise(self):
        self.amcl_receive_new_pose = False
    
    def update(self) -> py_trees.common.Status:

        # self.publisher_.publish(self.initial_pose_msg)

        if self.amcl_receive_new_pose == True:
            self.amcl_receive_new_pose = False
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def amcl_callback(self, msg):
        self.amcl_receive_new_pose = True
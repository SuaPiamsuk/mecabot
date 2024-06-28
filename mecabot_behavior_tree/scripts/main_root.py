#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

#!/usr/bin python3
import py_trees
import py_trees_ros.trees
import py_trees.console as console

from py_trees_ros import subscribers
from py_trees.common import OneShotPolicy


# from . import behaviours

import behaviours 

import rclpy
import sys




def create_root():
    '''
    Main Root
    '''

    # #######################################################################
    # loop

    # main_root = py_trees.composites.Sequence(name="Topics2BB", memory=False)

    # pub = behaviours.Finding_Ball(name='Ball Deteteced')

    # nav = behaviours.Navigator(name='nav2')

    # load = behaviours.LoadNewMap(name='new_map')

    # # Connecting root
    # main_root.add_children([pub])

    # main_root.add_children([nav])

    # main_root.add_children([load])

    # #######################################################################
    # oneshot

    seq = py_trees.composites.Sequence(name="Sequence", memory=True)
    main_root = py_trees.decorators.OneShot(
            name="root", child=seq, policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )

    

    # pub = behaviours.Finding_Ball(name='Ball Deteteced')
     # ###########################################
    #pattern A

    # goal_A = behaviours.Navigator(name='go_to_A', goal_pose=[-4.271, 0.755, -0.707, 0.707]) # x(poistion), y(position), z(quarternion), w(quarternion)

    # load_map_A = behaviours.LoadNewMap(name='load_map_room2', map_path='/home/nuc/h1.yaml')

    # load_map_B = behaviours.LoadNewMap(name='load_map_room2', map_path='/home/nuc/h2.yaml')
    
    # initial_A = behaviours.Initialpose(name='initial_A', pose=[-4.271, 0.755, -0.707, 0.707])

    # initial_B = behaviours.Initialpose(name='initial_A', pose=[0.0, 0.0, 0.0, 1.0])

    # goal_B = behaviours.Navigator(name='go_to_B', goal_pose=[3.0, 0.0, 0.0, 1.0]) # x(poistion), y(position), z(quarternion), w(quarternion)
   

    # goal_C = behaviours.Navigator(name='go_to_C', goal_pose=[0.0, 0.0, 0.0, 1.0])

    # goal_D = behaviours.Navigator(name='go_to_D', goal_pose=[-4.0, 2.5, 0.0, 1.0])

    

    # seq_A_to_B = py_trees.composites.Sequence(name="seq_A_to_B", memory=True)
    # seq_A_to_B.add_children([
    #     goal_A, 
    #     load_map_B, 
    #     initial_B, 
    #     behaviours.Waitkey(name='waitforkeyboard'),
    #     goal_B
    # ])

    # seq_B_to_A = py_trees.composites.Sequence(name="seq_B_to_A", memory=True)
    # seq_B_to_A.add_children([
    #     goal_C, 
    #     load_map_A, 
    #     initial_A, 
    #     behaviours.Waitkey(name='waitforkeyboard'),
    #     goal_D
    # ])

    # # seq.add_children([load])
    # seq.add_children([
    #     seq_A_to_B, 
    #     seq_B_to_A
    # ])

    # #######################################################################

    seq_A_to_B = py_trees.composites.Sequence(name="seq_A_to_B", memory=True)
    seq_B_to_A = py_trees.composites.Sequence(name="seq_B_to_A", memory=True)

    seq.add_children([
        seq_A_to_B.add_children([
            # goal_A, 
            # load_map_B, 
            # initial_B, 
            # behaviours.Waitkey(name='waitforkeyboard'),
            # goal_B

            behaviours.Navigator(name='go_to_A', goal_pose=[-4.271, 0.755, -0.707, 0.707]), # x(poistion), y(position), z(quarternion), w(quarternion), 
            behaviours.LoadNewMap(name='load_map_room2', map_path='/home/nuc/h2.yaml'),
            behaviours.Initialpose(name='initial_A', pose=[0.0, 0.0, 0.0, 1.0]),
            behaviours.Waitkey(name='wait_topic'),
            behaviours.Navigator(name='go_to_B', goal_pose=[3.0, 0.0, 0.0, 1.0]) # x(poistion), y(position), z(quarternion), w(quarternion)

        ]), 
        seq_B_to_A.add_children([
            behaviours.Navigator(name='go_to_C', goal_pose=[0.0, 0.0, 0.0, 1.0]), 
            behaviours.LoadNewMap(name='load_map_room2', map_path='/home/nuc/h1.yaml'), 
            behaviours.Initialpose(name='initial_A', pose=[-4.271, 0.755, -0.707, 0.707]), 
            behaviours.Waitkey(name='wait_topic'),
            behaviours.Navigator(name='go_to_D', goal_pose=[-4.0, 2.5, 0.0, 1.0])
        ])
    ])



    # seq.add_children([
    #     seq_A_to_B.add_children([
    #         # goal_A, 
    #         # load_map_B, 
    #         # initial_B, 
    #         # behaviours.Waitkey(name='waitforkeyboard'),
    #         # goal_B

    #         # behaviours.Navigator(name='go_to_AB', goal_pose=[0.494, 1.509, 0.0, 1.0]), # x(poistion), y(position), z(quarternion), w(quarternion), 
    #         behaviours.LoadNewMap(name='load_map_roomB', map_path='/home/pcn1705/room_B.yaml'),
    #         behaviours.Initialpose(name='initial_B', pose=[0.9, 0.0, 0.0, 1.0]),
    #         behaviours.Waitkey(name='wait_topic'),
    #         # behaviours.Navigator(name='go_to_BC', goal_pose=[1.625, 4.296, 0.707, 0.707]) # x(poistion), y(position), z(quarternion), w(quarternion)
    #         behaviours.Navigator(name='go_to_BC', goal_pose=[3.5, 3.0, 0.707, 0.707]) # x(poistion), y(position), z(quarternion), w(quarternion)

    #     ])
    # ])
  

    # #######################################################################


    # topics2bb.add_children([ballToBB,robotToBB])
    # ball_priorities.add_child(finding_ball)
    # finding_ball.add_children([ball_check_success])
    # robot_priorities.add_child(robot_nearest)
    # robot_nearest.add_child(robot_nearest_check)    
    
    return main_root


def main():
    """
    Create pytree ros from create_root() function
    """
    rclpy.init(args=None)
    tree = py_trees_ros.trees.BehaviourTree(
        root=create_root(),
        unicode_tree_debug=True
    )
    try:
        # Set Node name
        tree.setup(node_name="root_node", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
   main()

  

    
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

pkg_name = 'cart_controller'

def generate_launch_description():
    ld = LaunchDescription(
    )
          
    cart_controller = Node(
        package             = pkg_name,
        executable          = 'controller_node',
        name                = 'controller_node',
        #output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    ld.add_action(cart_controller)

    return ld
# SPDX-FileCopyrightText: 2025 tento
# SPDX-License-Identifier: BSD-3-Clause

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    BLEutms_sub = launch_ros.actions.Node(
            package = 'BLEutms',
            executable = 'BLEutms_sub',
            output = 'screen'
            )
    
    BLEutms_display = launch_ros.actions.Node(
            package = 'BLEutms',
            executable = 'BLEutms_display',
            output = 'screen'
            )
    
    BLEutms = launch_ros.actions.Node(
            package = 'BLEutms',
            executable = 'BLEutms',
            output = 'screen'
            )
    

    return launch.LaunchDescription([BLEutms, BLEutms_sub])
    #return launch.LaunchDescription([BLEutms, BLEutms_display])
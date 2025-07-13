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
    
    BLEutms = launch_ros.actions.Node(
            package = 'BLEutms',
            executable = 'BLEutms',
            )
    

    return launch.LaunchDescription([BLEutms_sub, BLEutms])
#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

# ros imports
from launch import LaunchDescription, actions, substitutions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments with default values
        actions.DeclareLaunchArgument('image_topic', default_value='/sensing/front_facing_camera/raw'),
        actions.DeclareLaunchArgument('vehicle_state_topic', default_value='/vehicle/state'),
        actions.DeclareLaunchArgument('objects_topic', default_value='/perception/objects'),
        actions.DeclareLaunchArgument('node_name', default_value='yolov5_detector'),
        actions.DeclareLaunchArgument('model', default_value='data/real.onnx'),
        actions.DeclareLaunchArgument('camera_calibration_file', default_value='data/calibration.json'),
        actions.DeclareLaunchArgument('vis', default_value='False'),
        
        # Define the Node action
        Node(
            package='cone_detector',
            executable=substitutions.LaunchConfiguration('node_name'),
            name=substitutions.LaunchConfiguration('node_name'),
            remappings=[
                ('~/input/image', substitutions.LaunchConfiguration('image_topic')),
                ('~/input/vehicle_state', substitutions.LaunchConfiguration('vehicle_state_topic')),
                ('~/output/objects', substitutions.LaunchConfiguration('objects_topic'))
            ],
            parameters=[
                {'model': substitutions.LaunchConfiguration('model')},
                {'camera_calibration_file': substitutions.LaunchConfiguration('camera_calibration_file')},
                {'vis': substitutions.LaunchConfiguration('vis')},
                # Assuming 'use_sim_time' is supposed to be a boolean, setting default to False
                {'use_sim_time': False}
            ]
        )
    ])


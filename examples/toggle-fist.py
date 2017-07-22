#!/usr/bin/env python
# Copyright (C) 2017 rerobots, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Example: repeatedly toggle the "fist" position

This repeatedly sends the "fist" motion primitive. Each successive
command negates the previous, so the hand should alternate between
opened and closed fists.
"""

import roslib; roslib.load_manifest('brunel_hand_ros')
import rospy
from brunel_hand_ros.msg import HandPrimitive


rospy.init_node('open_close_fist', anonymous=True)

pubprime = rospy.Publisher('/motion_primitive', HandPrimitive, queue_size=1)

rate = rospy.Rate(0.3)
while not rospy.is_shutdown():
    hp = HandPrimitive()
    hp.primitive = HandPrimitive.FIST
    pubprime.publish(hp)
    rate.sleep()

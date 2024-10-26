#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

GOALS = [
Pose(position=Point(x=-0.13842177391052246, y=-4.058953285217285, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.6762678863188706, w=0.7366557852442396)),

Pose(position=Point(x=-1.835866928100586, y=-6.92816686630249, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.946619688696926, w=0.3223525476420728)),

Pose(position=Point(x=-1.5035991668701172, y=-10.367730140686035, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.32894449115836716, w=0.9443492583449001)),

Pose(position=Point(x=-0.00019979476928710938, y=-13.330530166625977, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.3623350013612281, w=0.9320479315939492)),

Pose(position=Point(x=2.817479133605957, y=-15.553749084472656, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.31915242143691414, w=0.9477033986912542)),

Pose(position=Point(x=6.7756547927856445, y=-16.894346237182617, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.039550402043849735, w=0.9992175767560185)),

Pose(position=Point(x=10.743718147277832, y=-17.901214599609375, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.02429169613238109, w=0.9997049132113996)),

Pose(position=Point(x=12.698801040649414, y=-16.599123001098633, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.7250281526160504, w=0.6887192301033545)),

Pose(position=Point(x=12.496702194213867, y=-12.142489433288574, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.6726556095511892, w=0.7399556952543295)),

Pose(position=Point(x=12.671045303344727, y=-5.241645812988281, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.6831001452729678, w=0.7303247164981138)),

Pose(position=Point(x=12.543193817138672, y=-3.4387428760528564, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.9606530200561303, w=0.2777512827279761)),

Pose(position=Point(x=10.761127471923828, y=-1.582363486289978, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.8535574804900348, w=0.520998682819356)),

Pose(position=Point(x=8.079907417297363, y=2.2801122665405273, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.9999015134802468, w=0.014034362825290471)),

Pose(position=Point(x=1.7933197021484375, y=1.9002684354782104, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.973144246990475, w=0.23019616536628373)),

Pose(position=Point(x=0.11189079284667969, y=0.06722772121429443, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.6803837384964916, w=0.7328560352412592)),
]

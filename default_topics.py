from geometry_msgs.msg import PoseStamped
from actuator_msgs.msg import Actuators
from std_msgs.msg import String
from builtin_interfaces.msg import Time

initial_pub = {'pose1':PoseStamped,
               'pose2':PoseStamped,
               'pose3':PoseStamped,
               'actuator1':Actuators,
               'actuator2':Actuators,
               'time1':Time,
               'time2':Time,
               'time3':Time
               }
initial_sub = {'output_pose':PoseStamped,
               'joint1':Actuators,
               'string1':String,
               'string2':String,
               }

#!/usr/bin/env python
#12/2/18
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import utils # <----------- LOOK AT THESE FUNCTIONS ***************************

PUB_TOPIC = '/world_pose_array' # The topic that you should publish to
MAP_TOPIC = 'static_map' # The service topic that will provide the map
#POSE_TOPIC = '/sim_car_pose/pose'
POSE_TOPIC = "/pf/viz/inferred_pose"

class PointMap:

  def __init__(self):
    self.pub = rospy.Publisher(PUB_TOPIC, PoseArray, queue_size=1) 
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub, queue_size = 1)# Create a subscriber to the current position of the car

    self.plan1 = [[ 49.85113144,  12.11145401,   1.25342306], [ 49.87564152,  12.18607142,   1.30578294], [ 49.89621284,  12.26186934,   1.35814281],[ 49.91278902,  12.33863999,   1.41050269], [ 49.92532461,  12.41617296,   1.46286257], [ 49.93378525,  12.49425574,   1.51522245], [ 49.93814777,  12.5726743 ,   1.56758232], [ 49.9384002 ,  12.65121371,   1.6199422 ], [ 49.93758832,  12.6677201 ,   1.63095976], [ 49.93286595,  12.74611782,   1.57859988], [ 49.93225306,  12.82465524,   1.52624001], [ 49.93575135,  12.90311711,   1.47388013], [ 49.94335122,  12.98128836,   1.42152025], [ 49.95503184,  13.05895474,   1.36916037], [ 49.9707612 ,  13.13590336,   1.3168005 ], [ 49.99049618,  13.21192331,   1.26444062], [ 50.01418269,  13.28680623,   1.21208074], [ 50.04175581,  13.36034687,   1.15972086], [ 50.07313997,  13.43234366,   1.10736099], [ 50.10824913,  13.50259926,   1.05500111], [ 50.14698707,  13.57092111,   1.00264123], [ 50.18924761,  13.63712194,   0.95028135], [ 50.23491491,  13.70102029,   0.89792148], [ 50.28386382,  13.76244104,   0.8455616 ], [ 50.33596015,  13.82121582,   0.79320172], [ 50.39106112,  13.87718354,   0.74084184], [ 50.4490157 ,  13.93019079,   0.68848196], [ 50.50966505,  13.98009229,   0.63612209], [ 50.57284291,  14.02675127,   0.58376221], [ 50.63837614,  14.07003982,   0.53140233], [ 50.7060851 ,  14.10983931,   0.47904245], [ 50.77578422,  14.14604064,   0.42668258], [ 50.84728244,  14.17854458,   0.3743227 ], [ 50.92038381,  14.20726205,   0.32196282], [ 50.99488795,  14.23211434,   0.26960294], [ 51.07059065,  14.25303332,   0.21724307], [ 51.14728442,  14.26996166,   0.16488319], [ 51.22475904,  14.28285296,   0.11252331], [ 51.30280216,  14.29167188,   0.06016343], [  5.13811999e+01,   1.42963943e+01,   7.80355656e-03], [  5.14597373e+01,   1.42970071e+01,  -4.45563210e-02], [ 51.53819917,  14.29350885,  -0.0969162 ], [ 51.61637043,  14.28590898,  -0.14927608], [ 51.6940368 ,  14.27422836,  -0.20163595], [ 51.77098542,  14.258499  ,  -0.25399583], [ 51.84700537,  14.23876402,  -0.30635571], [ 51.9218883 ,  14.21507751,  -0.35871559], [ 51.99542894,  14.18750439,  -0.41107546], [ 52.06742573,  14.15612024,  -0.46343534], [ 52.13768133,  14.12101108,  -0.51579522], [ 52.20600317,  14.08227314,  -0.5681551 ], [ 52.272204  ,  14.0400126 ,  -0.62051497], [ 52.33610236,  13.99434529,  -0.67287485], [ 52.3975231 ,  13.94539639,  -0.72523473], [ 52.45629788,  13.89330005,  -0.77759461], [ 52.5122656 ,  13.83819908,  -0.82995448], [ 52.56527286,  13.7802445 ,  -0.88231436], [ 52.61517436,  13.71959516,  -0.93467424], [ 52.66183333,  13.65641729,  -0.98703412], [ 52.70512189,  13.59088406,  -1.03939399], [ 52.74492137,  13.5231751 ,  -1.09175387], [ 52.7811227 ,  13.45347599,  -1.14411375], [ 52.81362665,  13.38197776,  -1.19647363], [ 52.84234412,  13.3088764 ,  -1.2488335 ], [ 52.8671964 ,  13.23437226,  -1.30119338], [ 52.88811539,  13.15866956,  -1.35355326], [ 52.90504373,  13.08197579,  -1.40591314], [ 52.91793502,  13.00450117,  -1.45827302], [ 52.92675395,  12.92645804,  -1.51063289], [ 52.93147632,  12.84806032,  -1.56299277], [ 52.93208921,  12.7695229 ,  -1.61535265], [ 52.92859092,  12.69106103,  -1.66771253], [ 52.92099105,  12.61288978,  -1.7200724 ], [ 52.90931043,  12.5352234 ,  -1.77243228], [ 52.89358107,  12.45827478,  -1.82479216], [ 52.87384609,  12.38225483,  -1.87715204], [ 52.85015957,  12.30737191,  -1.92951191], [ 52.82258645,  12.23383127,  -1.98187179], [ 52.7912023 ,  12.16183448,  -2.03423167], [ 52.75609314,  12.09157888,  -2.08659155], [ 52.7173552 ,  12.02325703,  -2.13895142], [ 52.67509466,  11.9570562 ,  -2.1913113 ],[ 52.66631387,  11.94477   ,  -2.20137892], [ 52.62000559,  11.88133463,  -2.14901904], [ 52.57708072,  11.81556262,  -2.09665916], [ 52.53765691,  11.74763423,  -2.04429929], [ 52.51198523,  11.69753146,  -2.00676813],[ 52.27908707,  11.08817387,  -2.04597281],[ 52.24315546,  11.01833532,  -2.09833269],[ 52.20361804,  10.950473  ,  -2.15069257],[ 52.16058315,  10.88477291,  -2.20305245],[ 52.11416877,  10.82141513,  -2.25541232],[ 52.06450211,  10.76057333,  -2.3077722 ],[ 52.0117193 ,  10.70241425,  -2.36013208],[ 51.95596501,  10.64709733,  -2.41249196],[ 51.89739208,  10.59477416,  -2.46485183],[ 51.83616103,  10.54558817,  -2.51721171],[ 51.7724397 ,  10.49967418,  -2.56957159],[ 51.70640274,  10.45715803,  -2.62193147],[ 51.63823116,  10.41815625,  -2.67429134],[ 51.56811182,  10.38277574,  -2.72665122],[ 51.4962369 ,  10.35111349,  -2.7790111 ],[ 51.4228034 ,  10.32325627,  -2.83137098],[ 51.34801261,  10.29928044,  -2.88373085],[ 51.27206952,  10.27925172,  -2.93609073],[ 51.19518228,  10.263225  ,  -2.98845061],[ 51.11756165,  10.25124421,  -3.04081049],[ 51.03942036,  10.24334219,  -3.09317036],[ 50.9609726 ,  10.23954059,   3.13765506],[ 50.88243339,  10.23984985,   3.08529519],[ 50.80401801,  10.24426911,   3.03293531],[ 50.76821301,  10.24817497,   3.0089237 ],[ 50.47084929,  10.287859  ,   3.0089237 ],[ 50.17348558,  10.32754303,   3.0089237 ],[ 49.87612187,  10.36722707,   3.0089237 ],[ 49.57875816,  10.4069111 ,   3.0089237 ],[ 49.28139444,  10.44659513,   3.0089237 ],[ 48.98403073,  10.48627916,   3.0089237 ],[ 48.68666702,  10.52596319,   3.0089237 ],[ 48.38930331,  10.56564723,   3.0089237 ],[ 48.09193959,  10.60533126,   3.0089237 ],[ 47.79457588,  10.64501529,   3.0089237 ],[ 47.49721217,  10.68469932,   3.0089237 ],[ 47.19984846,  10.72438336,   3.0089237 ],[ 47.04948297,  10.74445006,   3.0089237 ],[ 46.97163333,  10.75483931,   3.06128358],[ 46.98812651,  10.7535119 ,   3.05025258]]


  def viz_sub(self, msg):

    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()

    map_img, self.map_info = utils.get_map(MAP_TOPIC) # Get and store the map
    plan_array = []     
    map_pose = []      
                 
    for k in xrange(len(self.plan1)):
         temp_xy = self.plan1[k]
         ori = Pose()
         #Setup the pose
         pose_out = Pose()
         pose_out.position.x = temp_xy[0]
         pose_out.position.y = temp_xy[1]
         pose_out.position.z = 0
         pose_out.orientation = utils.angle_to_quaternion(temp_xy[2])

         #Append the pose onto the array
         pa.poses.append(pose_out)
         print(pose_out)

    self.pub.publish(pa)
         
if __name__ == '__main__':
  
  rospy.init_node('point_map', anonymous=True) # Initialize the node
  
  pm = PointMap() # Create a clone follower
  rospy.spin() # Spin

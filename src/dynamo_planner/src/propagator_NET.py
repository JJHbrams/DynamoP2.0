#!/usr/bin/env python3
# Pytorch
import torch
from torch.autograd import Variable
import torch.nn.functional as F
import torch.utils.data as Data
from torch import nn
# Matplot
import matplotlib.pyplot as plt
# Numpy
import numpy as np
# ROS
import rospy
from std_msgs.msg import String
from dynamo_planner.msg import custom_states_msgs
# ETC
import time

class Net(nn.Module):

    def __init__(self): #
        super(Net, self).__init__()
        # input_dim=5(x y yaw ctrlA ctrlB), output_dim=3(x y yaw).
        self.fc1 = nn.Linear(in_features=6, out_features=512, bias=True)                                 # input_dim=5(x y yaw ctrlA ctrlB), output_dim=3(x y yaw).
        self.fc2 = nn.Linear(in_features=512, out_features=756, bias=True)
        self.fc3 = nn.Linear(in_features=756, out_features=3, bias=True)

    def forward(self, x):
        x=(self.fc1(x))
        x=F.relu(self.fc2(x))
        x=(self.fc3(x))

        return x

# Global variables
PropStates = custom_states_msgs()
TIME=[]
# PropStates.pre_x = -20;
# PropStates.pre_y = -20;
# PropStates.pre_yaw = 3.14;
pub = rospy.Publisher('/Prop_States',custom_states_msgs,queue_size=1)
model = Net()

def prop_cb(msg):
    rate = rospy.Rate(500);
    # print(msg)
    # print(PropStates)
    # if((PropStates.pre_x != msg.x) or (PropStates.pre_y != msg.y) or (PropStates.pre_yaw != msg.yaw)):
    data = [msg.x, msg.y, msg.yaw, msg.controlA, msg.controlB, msg.duration]
    data= torch.FloatTensor(data).float()
    # print("INPUT  : ",data)
    # Eval
    ts=int(round(time.time() * 1000))
    output = model(data)
    tf=int(round(time.time() * 1000))
    # print("OUTPUT : ",output)
    # print("TIME   : ",tf-ts,"ms")
    TIME.append(tf-ts)
    output = output.detach().numpy()
    PropStates.pre_x = msg.x;
    PropStates.pre_y = msg.y;
    PropStates.pre_yaw = msg.yaw;
    PropStates.duration = msg.duration;
    PropStates.controlA = msg.controlA;
    PropStates.controlB = msg.controlB;
    PropStates.x = output[0]
    PropStates.y = output[1]
    PropStates.yaw = output[2]
    # print("INPUT : X=",PropStates.pre_x,"Y=",PropStates.pre_y,"Z=",PropStates.pre_yaw,"CONTROLA=",msg.controlA,"CONTROLB=",msg.controlB)
    # print("OUTPUT: X=",PropStates.x,"Y=",PropStates.y,"Z=",PropStates.yaw)
    # print(PropStates)
    pub.publish(PropStates)
    # rate.sleep()

if __name__=='__main__':
    # Initialize ROS python
    print("Load propagator_NET!")
    rospy.init_node('propagator_NET', anonymous = False)
    rospy.Subscriber("/PlannerStates", custom_states_msgs, prop_cb, queue_size=1)

    # load net.prm
    params = torch.load("src/dynamo_planner/model/net.tar", map_location = "cpu")
    model.load_state_dict(params['model_state_dict'])
    model.eval()

    # callback
    rospy.spin()

    print("\nMaximum Time : ",np.max(TIME),"ms")
    print("Mean Time    : ",np.mean(TIME),"ms")

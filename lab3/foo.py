#!/usr/bin/env python

import kin_func_skeleton as skeleton
import numpy as np


q1=np.array([.0635,.2598,.1188])
w1=np.array([-.0059,.0113,.9999]) 
q2=np.array([.1106,.3116,.3885])
w2=np.array([-.7077,.7065,-.0122]) 
q3=np.array([.1827,.3838,.3881])
w3=np.array([.7065,.7077,-.0038]) 
q4=np.array([.3682,.5684,.3181])
w4=np.array([-.7077,.7065,-.0122]) 
q5=np.array([.4417,.6420,.3177])
w5=np.array([.7065,.7077,-.0038])   
q6=np.array([.6332,.8337,.3067])
w6=np.array([-.7077,.7065,-.0122]) 
q7=np.array([.7152,.9158,.3063])
w7=np.array([.7065,.7077,-.0038])
q_hand=np.array([.7957,.9965,.3058]) 

q = [q1,q2,q3,q4,q5,q6,q7]
w = [w1,w2,w3,w4,w5,w6,w7]

def twist(q,w):
    
    ans = np.zeros(6)
    ans[0:3] = np.cross(-w,q)
    ans[3:6] = w

    
    return ans

t = np.zeros((6,7))

for i in range(0,7):
    t[:,i] = twist(q[i],w[i])


tested_theta = np.array([-0.13920875634155275, 0.9690923616394044, -0.05675728908691407, -0.9046651686218262, 0.07133010655517578, 1.3683108612304689, -0.1223349676940918])

baxt_trans = np.array([0.656,0.724,0.374])

baxt_w = np.array([-3.096,0.139,-2.389])
baxt_w_norm = baxt_w/np.linalg.norm(baxt_w)
baxt_theta = np.linalg.norm(baxt_w)


baxt_homo_transform = np.zeros((4,4))
baxt_homo_transform[0:3,3] = baxt_trans[0:3]

baxt_rot = skeleton.rotation_3d(baxt_w_norm,baxt_theta)

baxt_homo_transform[0:3,0:3] = baxt_rot[0:3,0:3]
baxt_homo_transform[3,3] = 1

print("What we think what Baxter thinks the homogenous transform is")
print(baxt_homo_transform)



def task1(angles):
    return skeleton.prod_exp(t,angles)

I = np.eye(4)
I[0:3,3] = q_hand
print(I)

print("What we think the homogenous transform is based on the angles")
print(np.dot(task1(tested_theta),I))
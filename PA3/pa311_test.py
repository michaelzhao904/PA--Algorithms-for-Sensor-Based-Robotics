# -*- coding: utf-8 -*-
"""PA311.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1-6jsx9B-m_4Dfk_H0x41v9BZ9ynVmXm3
"""

import numpy as np

text_file = open("/content/drive/My Drive/PA3——ASBR/HW3-PA1/pa1-debug-a-calbody.txt", "r")

arr = np.loadtxt("/content/drive/My Drive/PA3——ASBR/HW3-PA1/pa1-debug-a-calbody.txt",delimiter=',',skiprows=1)
#print(arr)
np.shape(arr[:,0:2])
np.mean(arr[:,0:2],0)

aa = np.array([[1,2,3],[4,5,6]])
bb=np.mean(aa,0)
cc = np.tile(bb,(3,1))
cc
np.shape(cc)[0]

aa = np.array([[1,2,3]])
bb = aa.T
aa.T.dot(aa)

np.tile(mean,(arr.shape[0],1))

import numpy as np
def registration3D(ptsA,ptsB):
  # ptsA: (n,3) 3D data points
  # ptsB: (n,3) 3D data points
  # return T: (4,4) [R p;0 1] transformation matrix
  ave_A = np.mean(ptsA,0)
  ave_B = np.mean(ptsB,0)
  n = np.shape(ptsA)[0]
  aveRep_A = np.tile(ave_A,(n,1))
  aveRep_B = np.tile(ave_B,(n,1))
  np.shape(aveRep_A)
  tilde_A = ptsA - aveRep_A
  tilde_B = ptsB - aveRep_B
  # find R using SVD approach
  H = np.zeros((3,3))
  for i in range(n):
    a_i = tilde_A[i,:][:,None]
    b_i = tilde_B[i,:][None,:]
    H_i = a_i.dot(b_i)
    H += H_i
  U,s,VT = np.linalg.svd(H)
  R = VT.T.dot(U.T)
  # find p vector
  p = ave_B - R.dot(ave_A)
  p = p[:,None]
  T = np.c_[R,p]
  T = np.concatenate((T,np.array([[0,0,0,1]])))
  return T

registration3D(arr,arr)

ptsA = np.array([[1,0,0],[2,0,0],[3,0,0]])
ptsB = np.array([[1,0,0],[2,0,0],[3,0,0]])
#np.linalg.norm(R,2,0)
#np.linalg.det(R.T)
ave_A = np.mean(ptsA,0)
ave_B = np.mean(ptsB,0)
n = np.shape(ptsA)[0]
aveRep_A = np.tile(ave_A,(n,1))
aveRep_B = np.tile(ave_B,(n,1))
np.shape(aveRep_A)
tilde_A = ptsA - aveRep_A
tilde_B = ptsB - aveRep_B
# find R using SVD approach
H = np.zeros((3,3))
for i in range(n):
  a_i = tilde_A[i,:][:,None]
  b_i = tilde_B[i,:][None,:]
  H_i = a_i.dot(b_i)
  H += H_i
U,s,VT = np.linalg.svd(H)
R = VT.T.dot(U.T)
print(R)
p = ave_B - R.dot(ave_A)
print(p)
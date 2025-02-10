import numpy as np
import sympy as sy
from numpy import pi as pi
from numpy import cos as cos
from numpy import sin as sin
from numpy import tan as tan
from numpy import *
import math

class M:

  @staticmethod
  def CurMult(matrix1,matrix2):
    ans=np.dot(matrix1,matrix2).astype(np.float64)
    for row in range(ans.shape[0]):
      for col in range(ans.shape[1]):
        number=float(ans[row,col])
        
        if np.isclose(number,round(number),atol=1e-10):
          ans[row,col]=int(round(number))
          
    return ans

  @staticmethod
  def FixMult(matrix1,matrix2):
    ans=np.dot(matrix2,matrix1).astype(np.float64)
    for row in range(ans.shape[0]):
      for col in range(ans.shape[1]):
        number=float(ans[row,col])
        
        if np.isclose(number,round(number),atol=1e-10):
          ans[row,col]=int(round(number))
          
    return ans

  @staticmethod
  def trans(MovementInListForm):
    if not isinstance(MovementInListForm,(np.ndarray,list)):
      return 'Make input to Trans a list'
    translationmatrix=np.eye(3)
    translationmatrix[0][2]=MovementInListForm[0]
    translationmatrix[1][2]=MovementInListForm[1]
    translationmatrix[2][2]=MovementInListForm[2]
    return translationmatrix

  @staticmethod
  def rot(variable,Θ):
    Θ=Θ*pi/180
    if variable=='x':
      rotmatrix=np.array([[1,0,0],[0,cos(Θ),-sin(Θ)],[0,sin(Θ),cos(Θ)]])
    elif variable=='y':
      rotmatrix=np.array([[cos(Θ),0,sin(Θ)],[0,1,0],[-sin(Θ),0,cos(Θ)]])
    elif variable=='z':
      rotmatrix=np.array([[cos(Θ),-sin(Θ),0],[sin(Θ),cos(Θ),0],[0,0,1]])
    else:
      return "Invalid rotation axis. Choose 'x','y',or 'z'."

    rotmatrix=np.where(abs(rotmatrix)>1E-10,rotmatrix,0)
    #print(rotmatrix.shape[0])
    for row in range(rotmatrix.shape[0]):
        for col in range(rotmatrix.shape[1]):
            value=float(rotmatrix[row,col])
            if np.isclose(value,round(value),atol=1E-10):
              rotmatrix[row,col]=int(round(value))
    return rotmatrix

  @staticmethod
  def EulerAngles(matrix1):
    Θ=math.atan2(math.sqrt(1-matrix1[2][2]**2),matrix1[2][2])
    Θ=Θ*180/pi
    Φ=math.atan2(matrix1[1][2],matrix1[0][2])
    Φ=Φ*180/pi
    Ψ=math.atan2(matrix1[2][1],-matrix1[2][0])
    Ψ=Ψ*180/pi
    return round(Θ,6),round(Φ,6),round(Ψ,6)

  @staticmethod
  def ReverseEuler(RotationsAsList,AnglesAsList):           
    answerlist=[]

    #Checks
    if not isinstance(RotationsAsList, (list,np.ndarray)) or not isinstance(AnglesAsList, (list,np.ndarray)):
      return 'Please type both as a list! [RotationsAsList],[AnglesAsList]. Numpy lists are ok too!'
    elif np.array(RotationsAsList).ndim!=1 or np.array(AnglesAsList).ndim!=1:
      return 'Please enter both as a 1D list!'

    #Actual work
    else:
      #Converts list into lowercase in case it was inputted wrong
      RotationsAsList=[letters.lower() for letters in RotationsAsList]
      for i,rots in enumerate(RotationsAsList):
        if rots=='z':
          answerlist.append(M.rot('z',AnglesAsList[i]))
        if rots=='y':
          answerlist.append(M.rot('y',AnglesAsList[i]))
        if rots=='x':
          answerlist.append(M.rot('x',AnglesAsList[i]))
        if rots!='z' and rots!='y' and rots!='x':
          return "Please enter only 'x', 'y', and 'z'!"

    biganswer=answerlist[0]
    return M.CurMult(M.CurMult(answerlist[0],answerlist[1]),answerlist[2])
#    for i in range(len(answerlist)):
#        biganswer=M.CurMult(biganswer,answerlist[i])

    return biganswer

  @staticmethod
  def homogenous(variable,Θ,MovementSetFalseIfNone):
    homgenousmatrix=np.eye(4)

    if Θ!=False:
      rotationpart=M.rot(variable,Θ)
      homgenousmatrix[:3,:3]=rotationpart

    if MovementSetFalseIfNone!=False:
      if not isinstance(MovementSetFalseIfNone,(list,np.ndarray)):
        print('Please Type it in as [x,y,z] for the last collum.')
      else:
        homgenousmatrix[:3,3]=MovementSetFalseIfNone
    
    if Θ==False and MovementSetFalseIfNone==False:
      return "Don't make both 'false' conditions bro!"

    return homgenousmatrix

  @staticmethod
  def axisfind(rotmatrix):
    Θ = math.acos((np.trace(rotmatrix) - 1) / 2)
    k = (1 / (2 * math.sin(np.deg2rad(Θ)))) * np.array([[rotmatrix[2, 1] - rotmatrix[1, 2]], [rotmatrix[0, 2] - rotmatrix[2, 0]], [rotmatrix[1, 0] - rotmatrix[0, 1]]])
    return[k / np.linalg.norm(k), np.rad2deg(Θ)]

  @staticmethod
  def inverse(matrix):
    return np.linalg.inv(matrix)

  @staticmethod
  def DH(a,α,d,Θ):
    row1 = np.array([math.cos(np.deg2rad(Θ)), -math.sin(np.deg2rad(Θ)) * math.cos(np.deg2rad(α)), math.sin(np.deg2rad(Θ)) * math.sin(np.deg2rad(α)), a * math.cos(np.deg2rad(Θ))])
    row2 = np.array([math.sin(np.deg2rad(Θ)), math.cos(np.deg2rad(Θ)) * math.cos(np.deg2rad(α)), -math.cos(np.deg2rad(Θ)) * math.sin(np.deg2rad(α)), a * math.sin(np.deg2rad(Θ))])
    row3 = np.array([0, math.sin(np.deg2rad(α)), math.cos(np.deg2rad(α)), d])
    row4 = np.array([0, 0, 0, 1])
    return np.array([row1, row2, row3, row4])

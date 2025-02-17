import numpy as np
import sympy as sy

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
    translationmatrix=np.eye(4)
    translationmatrix[0,3]=MovementInListForm[0]
    translationmatrix[1,3]=MovementInListForm[1]
    translationmatrix[2,3]=MovementInListForm[2]
    return translationmatrix

  @staticmethod
  def rot(variable,Θ):
    Θ=Θ*np.pi/180
    if variable=='x':
      rotmatrix=np.array([[1,0,0],[0,np.cos(Θ),-np.sin(Θ)],[0,np.sin(Θ),np.cos(Θ)]])
    elif variable=='y':
      rotmatrix=np.array([[np.cos(Θ),0,np.sin(Θ)],[0,1,0],[-np.sin(Θ),0,np.cos(Θ)]])
    elif variable=='z':
      rotmatrix=np.array([[np.cos(Θ),-np.sin(Θ),0],[np.sin(Θ),np.cos(Θ),0],[0,0,1]])
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
    Θ=np.atan2(np.sqrt(1-matrix1[2,2]**2),matrix1[2,2])
    Θ=Θ*180/np.pi
    Φ=np.atan2(matrix1[1,2],matrix1[0,2])
    Φ=Φ*180/np.pi
    Ψ=np.atan2(matrix1[2,1],-matrix1[2,0])
    Ψ=Ψ*180/np.pi
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

    return M.CurMult(M.CurMult(answerlist[0],answerlist[1]),answerlist[2])
    
  @staticmethod
  def homogenous(variable,Θ,MovementSetFalseIfNone):
    homgenousmatrix=np.eye(4)

    if Θ!=False and variable!=False:
      rotationpart=M.rot(variable,Θ)
      homgenousmatrix[:3,:3]=rotationpart

    if MovementSetFalseIfNone!=False:
      if not isinstance(MovementSetFalseIfNone,(list,np.ndarray)):
        print('Please Type it in as [x,y,z] for the last collum.')
      else:
        homgenousmatrix[:3,3]=MovementSetFalseIfNone
    
    if (Θ==False or variable==False) and MovementSetFalseIfNone==False:
      return "Don't make both rotation and movement 'false' bro! You get an error till you fix it"

    return homgenousmatrix

  @staticmethod
  def axisfind(rotmatrix):
    Θ = np.acos((np.trace(rotmatrix) - 1) / 2)
    k = (1 / (2 * np.sin(np.deg2rad(Θ)))) * np.array([[rotmatrix[2, 1] - rotmatrix[1, 2]], [rotmatrix[0, 2] - rotmatrix[2, 0]], [rotmatrix[1, 0] - rotmatrix[0, 1]]])
    return[k / np.linalg.norm(k), np.rad2deg(Θ)]

  @staticmethod
  def inv(matrix):
    return np.linalg.inv(matrix)

  @staticmethod
  def DH(a,α,d,Θ):
    row1 = np.array([np.cos(np.deg2rad(Θ)), -np.sin(np.deg2rad(Θ)) * np.cos(np.deg2rad(α)), np.sin(np.deg2rad(Θ)) * np.sin(np.deg2rad(α)), a * np.cos(np.deg2rad(Θ))])
    row2 = np.array([np.sin(np.deg2rad(Θ)), np.cos(np.deg2rad(Θ)) * np.cos(np.deg2rad(α)), -np.cos(np.deg2rad(Θ)) * np.sin(np.deg2rad(α)), a * np.sin(np.deg2rad(Θ))])
    row3 = np.array([0, np.sin(np.deg2rad(α)), np.cos(np.deg2rad(α)), d])
    row4 = np.array([0, 0, 0, 1])
    return np.array([row1, row2, row3, row4])
  
class MS:

  @staticmethod
  def CurMult(matrix1,matrix2):
    matrix1=sy.Matrix(matrix1); matrix2=sy.Matrix(matrix2)
    return matrix1*matrix2

  @staticmethod
  def FixMult(matrix1,matrix2):
    matrix1=sy.Matrix(matrix1); matrix2=sy.Matrix(matrix2)
    return matrix2*matrix1

  @staticmethod
  def trans(MovementInListForm):
    if not isinstance(MovementInListForm,(sy.Matrix,list)):
      return 'Make input to Trans a list! Also make sure its either Python list or Sympy List since your using symbolic'
    translationmatrix=sy.eye(4)
    translationmatrix[0,3]=sy.sympify(MovementInListForm[0])
    translationmatrix[1,3]=sy.sympify(MovementInListForm[1])
    translationmatrix[2,3]=sy.sympify(MovementInListForm[2])
    return translationmatrix

  @staticmethod
  def rot(variable,Θ):
    Θ=sy.sympify(Θ)*sy.pi/180
    if variable=='x':
      rotmatrix=sy.Matrix([[1,0,0],[0,sy.cos(Θ),-sy.sin(Θ)],[0,sy.sin(Θ),sy.cos(Θ)]])
    elif variable=='y':
      rotmatrix=sy.Matrix([[sy.cos(Θ),0,sy.sin(Θ)],[0,1,0],[-sy.sin(Θ),0,sy.cos(Θ)]])
    elif variable=='z':
      rotmatrix=sy.Matrix([[sy.cos(Θ),-sy.sin(Θ),0],[sy.sin(Θ),sy.cos(Θ),0],[0,0,1]])
    else:
      return "Invalid rotation axis. Choose 'x','y',or 'z'."

    return rotmatrix

  @staticmethod
  def EulerAngles(matrix1):
    Θ=sy.atan2(sy.sqrt(1-matrix1[2,2]**2),matrix1[2,2])
    Θ=Θ*180/sy.pi
    Φ=sy.atan2(matrix1[1,2],matrix1[0,2])
    Φ=Φ*180/sy.pi
    Ψ=sy.atan2(matrix1[2,1],-matrix1[2,0])
    Ψ=Ψ*180/sy.pi
    return Θ,Φ,Ψ

  @staticmethod
  def ReverseEuler(RotationsAsList,AnglesAsList):           
    answerlist=[]

    #Checks
    if not isinstance(RotationsAsList, (list,sy.Matrix)) or not isinstance(AnglesAsList, (list,sy.Matrix)):
      return 'Please type both as a list! [RotationsAsList],[AnglesAsList]. Sympy lists are ok too!'
    elif len(RotationsAsList)!=1 or len(AnglesAsList)!=1:
      return 'Please enter both as a 1D list!'

    #Actual work
    else:
      #Converts list into lowercase in case it was inputted wrong
      RotationsAsList=[letters.lower() for letters in RotationsAsList]
      for i,rots in enumerate(RotationsAsList):
        if rots=='z':
          answerlist.append(MS.rot('z',AnglesAsList[i]))
        if rots=='y':
          answerlist.append(MS.rot('y',AnglesAsList[i]))
        if rots=='x':
          answerlist.append(MS.rot('x',AnglesAsList[i]))
        if rots!='z' and rots!='y' and rots!='x':
          return "Please enter only 'x', 'y', and 'z'!"

    return MS.CurMult(MS.CurMult(answerlist[0],answerlist[1]),answerlist[2])
    
  @staticmethod
  def homogenous(variable,Θ,MovementSetFalseIfNone):
    homgenousmatrix=sy.eye(4)

    if Θ!=False and variable!=False:
      rotationpart=MS.rot(variable,Θ)
      homgenousmatrix[:3,:3]=rotationpart

    if MovementSetFalseIfNone!=False:
      if not isinstance(MovementSetFalseIfNone,(list,sy.Matrix)):
        print('Please Type it in as [x,y,z] for the last collum.')
      else:
        homgenousmatrix[:3,3]=sy.sympify(MovementSetFalseIfNone)
    
    if (Θ==False or variable==False) and MovementSetFalseIfNone==False:
      return "Don't make both rotation and movement 'false' bro! You get an error till you fix it"

    return homgenousmatrix

  @staticmethod
  def axisfind(rotmatrix):
    rotmatrix=sy.Matrix(rotmatrix)
    Θ = sy.acos((rotmatrix.trace() - 1) / 2)
    #k = (1 / (2 * np.sin(np.deg2rad(Θ)))) * np.array([[rotmatrix[2, 1] - rotmatrix[1, 2]], [rotmatrix[0, 2] - rotmatrix[2, 0]], [rotmatrix[1, 0] - rotmatrix[0, 1]]])
    #normalize k
    k_x = rotmatrix[2,1] - rotmatrix[1,2]
    k_y = rotmatrix[0,2] - rotmatrix[2,0]
    k_z = rotmatrix[1,0] - rotmatrix[0,1]

    #This factor is 1/(2*sin(Theta))
    denom = 2*sy.sin(Θ)
    k = sy.Matrix([k_x, k_y, k_z]) / denom

    #Normalized axis
    k = k / k.norm()

    #Convert Theta to degrees
    Θ = Θ*180/sy.pi

    return [k, Θ]

  @staticmethod
  def inv(matrix):
    answer=sy.Matrix(matrix)
    return answer.inv()

  @staticmethod
  def DH(a,α,d,Θ,r):
    Θ=sy.sympify(Θ)
    if r=='d':
      Θ*sy.pi/180
    α=sy.sympify(α)
    if α=='d':
      α=α*sy.pi/180
    a=sy.sympify(a)
    d=sy.sympify(d)
    row1 = [sy.cos(Θ), -sy.sin(Θ) * sy.cos(α), sy.sin(Θ) * sy.sin(α), a * sy.cos(Θ)]
    row2 = [sy.sin(Θ), sy.cos(Θ) * sy.cos(α), -sy.cos(Θ) * sy.sin(α), a * sy.sin(Θ)]
    row3 = [0, sy.sin(α), sy.cos(α), d]
    row4 = [0, 0, 0, 1]
    return sy.Matrix([row1, row2, row3, row4])

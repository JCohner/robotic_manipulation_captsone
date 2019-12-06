import modern_robotics as mr
import numpy as np
from NextState import NextState
from pprint import pprint

e_int = np.zeros((4,4))

#X - Tse, Xd; Tse_d; Xd_next - Tse_d,next; 
def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
	global e_int
	#find desired twist 
	Vd = mr.se3ToVec(1/dt * mr.MatrixLog6(np.matmul(mr.TransInv(Xd),Xd_next)))
	Xerr = mr.MatrixLog6(np.matmul(mr.TransInv(X),Xd))
	e_int = e_int + Xerr * dt
	V = np.matmul(mr.Adjoint(np.matmul(mr.TransInv(X),Xd)), Vd) + mr.se3ToVec(np.matmul(Kp, Xerr)) + mr.se3ToVec(np.matmul(Ki,e_int))

	return V #twist in end effector frame

def main():
	#calculate end effector position based on chassis configuration q, and arm thetalist 
	q0 = np.array([0,0,0,0,0,0.2,-1.6,0])
	Xd = np.array([[0,0,1,0.5],
				   [0,1,0,0],
				   [-1,0,0,0.5],
				   [0,0,0,1]])
	Xd_next = np.array([[0,0,1,0.6],
				   [0,1,0,0],
				   [-1,0,0,0.3],
				   [0,0,0,1]])

	X = np.array([[0.17, 0, 0.985, 0.387],
				  [0, 1, 0 , 0],
				  [-0.985, 0, .17, 0.57],
				  [0, 0, 0, 1]])

	Kp = np.identity(4)#np.zeros((4,4))
	Ki = np.zeros((4,4))

	dt = 0.01

	V_ee =  FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)
	# thetadot_list = V_ee[3:]
	# u = V_ee[:3]
	# q_list[i] = NextState()
	print(V_ee)


if __name__ == '__main__':
	main()
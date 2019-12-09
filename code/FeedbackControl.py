import modern_robotics as mr
import numpy as np
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

	return V, mr.se3ToVec(Xerr) #twist in end effector frame


def Jacobian_in_Body_Pinv(q):
	B_list_arm = np.array([[0,0,0,0,0],
						  [0,-1,-1,-1,0],
						  [1,0,0,0,1],
						  [0,-0.5076,-0.3526,-0.2176,0],
						  [0.033,0,0,0,0],
						  [0,0,0,0,0]])

	Tb0 = np.array([[1,0,0,0.1662],
					[0,1,0,0],
					[0,0,1,0.0026],
					[0,0,0,1]])

	#to get T0e
	M0e = np.array([[1,0,0,0.033],
					[0,1,0,0],
					[0,0,1,0.6546],
					[0,0,0,1]])
	# print(q.shape)
	T0e = mr.FKinBody(M0e, B_list_arm, q[3:])

	Teb = np.matmul(mr.TransInv(T0e),mr.TransInv(Tb0))
	l = 0.47/2 
	w = .3/2
	r = 0.0475

	F6 = np.zeros((6,4))
	F6[2:5] = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
						[1,1,1,1],
						[-1,1,-1,1]])

	Ad = mr.Adjoint(Teb)
	Jbase = np.matmul(Ad,F6)

	Jarm = mr.JacobianBody(B_list_arm, q[3:])


	Je = np.concatenate((Jbase, Jarm),1)
	Je_pinv = np.linalg.pinv(Je,1e-4)

	return Je_pinv, mr.TransInv(Teb)


def main():
	#calculate end effector position based on chassis configuration q, and arm thetalist 
	q0 = np.array([0,0,0,0,0,0.2,-1.6,0])
	#X - Tse, Xd; Tse_d; Xd_next - Tse_d,next; 
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

	Kp = np.identity(4) #np.zeros((4,4))
	Ki = np.zeros((4,4))

	dt = 0.01

	V_ee, err =  FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)

	Je_pinv, other = Jacobian_in_Body_Pinv(q0)
	vels = np.matmul(Je_pinv, V_ee)
	print("V is:")
	pprint(V_ee)
	print("velocities are:")
	#vels = np.around(vels,4)
	pprint(vels)

if __name__ == '__main__':
	main()

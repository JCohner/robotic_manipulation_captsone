import modern_robotics as mr
import numpy as np
import pandas as pd

def to_csv(p_list):
	df = pd.DataFrame(p_list)
	df.to_csv("eggs.csv", header=False, index=False)

#q - config 12 x 1, thetadot, thetadot - control vector 9 x 1, dt - timestep, omega_max - max angular velocity of arm joints and wheels 
def NextState(q, thetadot, dt, omega_max):
	#arm angles
	for i in range(thetadot.shape[0]):
		x = abs(thetadot[i])
		if x > omega_max:
			thetadot[i] = thetadot[i]/float(x) * omega_max
	q_new = np.zeros(13)
	q_new[3:8] = q[3:8] + thetadot[4:] * dt
	q_new[8:12] = q[8:12] + thetadot[:4] * dt
	l = 0.47/2 
	w = .3/2
	H_mat = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
	q_new[0:3] = 0.0475/4. * np.matmul(H_mat, thetadot[:4]) * dt + q[0:3]
	#return the new configuration
	return q_new

def main():
	dt = 0.01
	t_total = 1
	tvec = np.arange(0,t_total,dt)
	its = tvec.shape[0]

	omega_max = 5
	q_list = np.zeros((its,13))
	q0 = np.array([0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) #x_chassis, y_chassis, phi_chassis, arm_theta1, arm_theta2, arm_theta3, arm_theta4, arm_theta5, wheel_theta1, wheel_theta2, wheel_theta3, wheel_theta4 
	q_list[0] = q0
	u_list = np.array([0,0,0,0,0, 10,10,10,10]) #arm_theta1, arm_theta2, arm_theta3, arm_theta4, arm_theta4, wheel_theta1, wheel_theta2, wheel_theta3, wheel_theta4
	for i in range(1, its):
		q_list[i] = NextState(q_list[i-1], u_list, dt, omega_max)
	# NextState(q_list[0], u_list, dt, omega_max)
	to_csv(q_list)

if __name__ == '__main__':
	main()
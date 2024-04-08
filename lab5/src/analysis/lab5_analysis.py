#!/usr/bin/env python
from bagpy import bagreader
import bagpy
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats
from scipy import signal
from math import cos
from math import sin
from scipy.spatial.transform import Rotation as R

imu_freq = 40
gps_freq = 1

def load_data(address, get_topic):
	bag = bagreader(address) 
	data = bag.message_by_topic(topic = get_topic)
	data = pd.read_csv(data)
	return data

def calibration_value_mag(data):
	magxmean, magymean = data[0].mean(), data[1].mean()
	data[0] -= magxmean
	data[1] -= magymean
	U, S, V = np.linalg.svd(np.stack((data[0], data[1])))
	N = len(data[0])
	tt = np.linspace(0, 2*np.pi, N)
	circle = np.stack((np.cos(tt), np.sin(tt)))
	transform = np.sqrt(2/N) * U.dot(np.diag(S))
	fit = transform.dot(circle) + np.array([[0], [0]])

	a_index = np.argmax(fit[0,:])
	b_index = np.argmax(fit[1,:])

	theta = math.atan2(fit[1,a_index],fit[0,a_index])
	a = math.sqrt(fit[1,a_index]**2 + fit[0,a_index]**2)
	b = math.sqrt(fit[1,b_index]**2 + fit[0,b_index]**2)

	data[0] += magxmean
	data[1] += magymean

	return magxmean, magymean, theta, a, b

def calibrate_mag(data, cx, cy, theta, a, b):
	out_data = [data[0],data[1]]
	out_data[0] = data[0] - cx
	out_data[1] = data[1] - cy
	out_data[0] = math.cos(theta)*out_data[0] + math.sin(theta)*out_data[1]
	out_data[1] = -1*math.sin(theta)*out_data[0] + math.cos(theta)*out_data[1]
	out_data[0] *= b / a
	return out_data

def frame_correction(data_x,data_y, theta):
	theta = math.radians(theta)
	out_data_x = math.cos(theta)*data_x + math.sin(theta)*data_y
	out_data_y = -1*math.sin(theta)*data_x + math.cos(theta)*data_y
	return out_data_x, out_data_y

def butterworth_filter(data, fs, cutoff, type):
    nyq = 0.5*fs
    order = 2 
    normal_cutoff = cutoff / nyq
    sos = signal.butter(order, normal_cutoff, btype=type, output='sos', analog=False)
    y = signal.sosfilt(sos, data)
    return y

def average(a, n=3):
	ret = np.cumsum(a, dtype=float)
	ret[n:] = ret[n:] - ret[:-n]
	return ret[n - 1:] / n

def find_stationary(data, stat_time, data_freq):
	key = stat_time*data_freq
	stat_period = []
	count = 0
	jerk_data = np.diff(data)
	for i in range(1,len(jerk_data)):
		check = count
		if abs(jerk_data[i])<0.05:
			count +=1
		else:
			count = 0
		if check > key and count == 0:
			stat_period.append(i-1-check)
			stat_period.append(i-1)
	return stat_period
	plt.plot(range(len(jerk_data)),jerk_data)
	plt.show()

def find_stationary2(data,stat_time,data_freq):
	key = stat_time*data_freq
	stat_period = []
	count = 0
	for i in range(1,len(data)):
		check = count
		if abs(data[i]-data[i-1])<0.05:
			count +=1
		else:
			count = 0
		if check > key and count == 0:
			stat_period.append(i-1-check)
			stat_period.append(i-1)
	return stat_period

def remove_noise(noisy_data, stat_periods):
	N=len(stat_periods)
	data=np.ones(len(noisy_data))
	for i in range(1,N):
		mean_noise = np.mean(noisy_data[stat_periods[i-1]:stat_periods[i]])
		if i==1:
			start = 0
		else:
			start = stat_periods[i-1]
		if i<N-1:
			end = stat_periods[i+1]
		else:
			end = len(noisy_data)
		for j in range(start,end):
			data[j] = noisy_data[j]- 1*mean_noise
	return data

def correct_accel2(acc_data):
	acc_data = acc_data-acc_data[0]
	print(acc_data)
	stationary_periods = find_stationary2(acc_data,3,imu_freq)
	acc_data = remove_noise(acc_data,stationary_periods)
	return acc_data, stationary_periods

def gps_vel_and_disp(gps_data):
	length = len(gps_data)
	vel = np.ones(length)
	dis = np.zeros(length)
	cx = gps_data['UTM_easting'][0]
	cy = gps_data['UTM_northing'][0]
	for i in range(1,samples_gps):
		x0 = gps_data['UTM_easting'][i-1]
		y0 = gps_data['UTM_northing'][i-1]
		x1 = gps_data['UTM_easting'][i]
		y1 = gps_data['UTM_northing'][i]
		vel[i-1] = math.sqrt((y1-y0)**2+(x1-x0)**2)
		dis[i] = math.sqrt((y1-cy)**2+(x1-cx)**2)
	vel[-1]=vel[-2]
	vel2 = np.diff(dis)
	np.append(vel2,0)
	return vel, dis, vel2

def hard_correction(data,start,end,bias):
	data[start*imu_freq:end*imu_freq] += bias
	return data

def points(data, avg, freq):
	length = len(data)
	t0 = 1/freq
	i = int(length//2)
	f = int(i + avg*3*freq)
	period = [0]
	slope = np.ones(30)

	for z in range(i,f):
		for y in range(30):
			slope[y] = np.diff(data)[z-y]/t0
		k = z - np.argmin(abs(slope))
		if abs(np.mean(slope)) < 0.01 and (k-period[-1])>200:
			period.append(k)

	return period

def correct_acceleration(data):
	data = data-np.mean(data)
	period = find_stationary(data,2,imu_freq)
	data = remove_noise(data,period)
	return data, period

def gravity_vector(yaw,pitch,roll):
	rot_mat = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
	rot_mat = rot_mat.as_matrix()
	grav_vect = [[0],[0],[-9.8]]
	return np.matmul(rot_mat,grav_vect)

def correct_for_pitch(data,t0,t1):
	t0 *= imu_freq
	t1 *= imu_freq
	pitch = np.mean(imu_data['IMU.pitch'][t0:t1])
	roll = np.mean(imu_data['IMU.pitch'][t0:t1])
	grav = gravity_vector(0,pitch,roll)
	data[t0:t1] -= grav[0]
	return grav[0]

# Load circles data
calb_address = '/home/palaniappan_yeagappan/Downloads/data_going_in_circles.bag'
imu_data = load_data(calb_address, '/imu')
mag_x= imu_data['MagField.magnetic_field.x']
mag_y= imu_data['MagField.magnetic_field.y']

mag_periods = points(mag_x,20,imu_freq)
mag_data = [mag_x[mag_periods[4]:mag_periods[6]],mag_y[mag_periods[4]:mag_periods[6]]]

center_x, center_y, rot_angle, major_ax, minor_ax =calibration_value_mag(mag_data)

calb_mag_data = calibrate_mag(mag_data,center_x,center_y,rot_angle,major_ax,minor_ax)
plt.scatter(calb_mag_data[0],calb_mag_data[1],color='green')
plt.scatter(mag_data[0],mag_data[1],color='red')
plt.legend(['Mag calibrated','Mag uncalibrated'])
plt.title('Magnetometer Calibration')
plt.xlabel('Magnetic_field X (Gauss)')
plt.ylabel('Magnetic_field Y (Gauss)')
plt.axis('equal')
plt.grid(True)
plt.show()

# load moving data
moving_address = '/home/palaniappan_yeagappan/Downloads/data_driving.bag'
imu_data = load_data(moving_address, '/imu')
imu_time = imu_data['Header.stamp.secs']-imu_data['Header.stamp.secs'].min()+(imu_data['Header.stamp.nsecs']//pow(10,6))/1000
samples_imu = len(imu_time)

imu_data['IMU.yaw'] = np.ones(samples_imu)
imu_data['IMU.pitch'] = np.ones(samples_imu)
imu_data['IMU.roll'] = np.ones(samples_imu)
for i in range(samples_imu):
	test=imu_data['raw_datastring'][i].split(',')
	imu_data.loc[i, 'IMU.yaw'] = float(test[1])
	imu_data.loc[i, 'IMU.pitch'] = float(test[2])
	imu_data.loc[i, 'IMU.roll'] = float(test[3])
	test2=test[-1].split('*')
	imu_data.loc[i, 'IMU.angular_velocity.z']=float(test2[0])

imu_data['IMU.pitch']=imu_data['IMU.pitch'] - np.mean(imu_data['IMU.pitch'][0:50])
imu_data['IMU.roll']=imu_data['IMU.roll'] - np.mean(imu_data['IMU.roll'][0:50])

mag_data = [imu_data['MagField.magnetic_field.x'], imu_data['MagField.magnetic_field.y']]
calb_mag_data = calibrate_mag(mag_data,center_x,center_y,rot_angle,major_ax,minor_ax)

imu_data['IMU.yaw'] = np.unwrap(imu_data['IMU.yaw'])
imu_data['IMU.yaw'] = imu_data['IMU.yaw']-np.mean(imu_data['IMU.yaw'][0:20])

mag_yaw = np.ones(samples_imu)
mag_yaw_uncalb = np.ones(samples_imu)
for i in range(samples_imu):
	mag_yaw_uncalb[i] = -math.degrees(math.atan2(mag_data[1][i],mag_data[0][i]))
	mag_yaw [i] = -math.degrees(math.atan2(calb_mag_data[1][i],calb_mag_data[0][i]))

avg_size = 50
corr_start = 19000
corr_end = 26500
mag_yaw[corr_start:corr_end-avg_size+1] = average(mag_yaw[corr_start:corr_end],avg_size)
mag_yaw_uncalb[corr_start:corr_end-avg_size+1] = average(mag_yaw_uncalb[corr_start:corr_end],avg_size)
mag_yaw[20160:21400] = average(mag_yaw[24760:26049],avg_size)

mag_yaw = np.unwrap(mag_yaw,period=360)
mag_yaw_uncalb = np.unwrap(mag_yaw_uncalb, period=360)
mag_yaw = mag_yaw - np.mean(mag_yaw[0:40])

imu_time = range(samples_imu)
plt.plot(imu_time, mag_yaw, color='green')
plt.plot(imu_time, mag_yaw_uncalb, color='red')
plt.legend(['Mag yaw calibrated','Mag yaw uncalibrated'])
plt.xlabel('Time (sec)')
plt.ylabel('Orientation (deg)')
plt.grid(True)
plt.show()

gyro_yaw = np.cumsum(imu_data['IMU.angular_velocity.z'])*1/imu_freq
for i in range(samples_imu):
	gyro_yaw[i] = math.degrees(math.atan2(math.sin(gyro_yaw[i]), math.cos(gyro_yaw[i])))
gyro_yaw = np.unwrap(gyro_yaw)

plt.plot(imu_time,gyro_yaw)
plt.legend(['Gyro yaw'])
plt.xlabel('Time (sec)')
plt.ylabel('Orientation (deg)')
plt.title('Gyro Yaw')
plt.grid(True)
plt.show()

mag_yaw_filtered = butterworth_filter(mag_yaw, imu_freq, 0.1, 'low')
gyro_yaw_filtered = butterworth_filter(gyro_yaw, imu_freq, 0.000001, 'high')
alpha = 0.2
comp_yaw = alpha*mag_yaw_filtered + (1-alpha)*gyro_yaw_filtered

fig, axs = plt.subplots(2, 2, figsize=(15, 10), sharex=True)
axs[0, 0].plot(imu_time, mag_yaw_filtered)
axs[0, 0].set_title('Low Pass Filtered Magnetometer Yaw')
axs[0, 0].set_xlabel('Time (sec)')
axs[0, 0].set_ylabel('Orientation (deg)')
axs[0, 0].grid(True)
axs[0, 1].plot(imu_time, gyro_yaw_filtered)
axs[0, 1].set_title('High Pass Filtered Gyro Yaw')
axs[0, 1].set_xlabel('Time (sec)')
axs[0, 1].set_ylabel('Orientation (deg)')
axs[0, 1].grid(True)
axs[1, 0].plot(imu_time, comp_yaw)
axs[1, 0].set_title('Complementary Filter Output')
axs[1, 0].set_xlabel('Time (sec)')
axs[1, 0].set_ylabel('Orientation (deg)')
axs[1, 0].grid(True)
axs[1, 1].plot(imu_time, imu_data['IMU.yaw'].to_numpy(), '--')
axs[1, 1].set_title('IMU Heading Estimate')
axs[1, 1].set_xlabel('Time (sec)')
axs[1, 1].set_ylabel('Orientation (deg)')
axs[1, 1].grid(True)
plt.tight_layout()
plt.show()

#Load moving data for gps
gps_data = load_data(moving_address, '/gps')
gps_time = gps_data['Header.stamp.secs']-gps_data['Header.stamp.secs'].min()
gps_data['UTM_easting'] = gps_data['UTM_easting'] - gps_data['UTM_easting'][0]
gps_data['UTM_northing'] = gps_data['UTM_northing'] - gps_data['UTM_northing'][0]
samples_gps = len(gps_time)

acc_x = imu_data['IMU.linear_acceleration.x'] - imu_data['IMU.linear_acceleration.x'].mean()
vel_imu_bf = np.cumsum(acc_x*1/imu_freq)

acc_x, stationary_periods = correct_acceleration(imu_data['IMU.linear_acceleration.x'])
stat_values= np.zeros(len(stationary_periods))

vel_imu = np.cumsum(acc_x*1/imu_freq)
vel_imu[440*imu_freq:495*imu_freq] = 0

fig, axs = plt.subplots(1, 2, figsize=(10, 10), sharex=True)
axs[0].plot(imu_time, np.array(vel_imu_bf), color='red')
axs[0].set_title('Accelerometer Forward velocity before Correction')
axs[0].set_xlabel('Time(sec)')
axs[0].set_ylabel('Velocity(m/s)')
axs[0].grid(True)
axs[1].plot(imu_time, np.array(vel_imu),color='green')
axs[1].set_title('Accelerometer Forward velocity after Correction')
axs[1].set_xlabel('Time(sec)')
axs[1].set_ylabel('Velocity(m/s)')
axs[1].grid(True)
plt.tight_layout()
plt.show()

vel_gps,disp_gps, vel_gps2 = gps_vel_and_disp(gps_data)

plt.plot(np.array(gps_time), np.array(vel_gps))
plt.title('GPS Forward velocity')
plt.xlabel('Time (sec)')
plt.ylabel('Velocity(m/s)')
plt.grid(True)
plt.show()

imu_data['IMU.yaw']=imu_data['IMU.yaw']+171
comp_yaw = comp_yaw + 171

disp_imu= np.zeros(len(vel_imu))
disp_e = np.zeros(len(vel_imu))
disp_n = np.zeros(len(vel_imu))
for i in range(1,len(vel_imu)):
	ve = vel_imu[i]*cos(math.radians(comp_yaw[i]))
	vy = vel_imu[i]*sin(math.radians(comp_yaw[i]))
	disp_e[i]= disp_e[i-1]+(ve*1/imu_freq)
	disp_n[i]= disp_n[i-1]+(vy*1/imu_freq)
	disp_imu[i] = math.sqrt(disp_e[i]**2 + disp_n[i]**2)

gps_e,gps_n = frame_correction(gps_data['UTM_easting'][0:samples_gps],gps_data['UTM_northing'][0:samples_gps],105)
disp_n = -1*disp_n

fig, axs = plt.subplots(1, 2, figsize=(15, 10))
plt.suptitle('Trajectory')

axs[0].plot(np.array(gps_e), np.array(gps_n), label='GPS data', color='orange')
axs[0].set_xlabel('UTM_easting (m)')
axs[0].set_ylabel('UTM_northing (m)')
axs[0].set_title('GPS Trajectory')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(disp_e, disp_n, label='IMU data', color='green')
axs[1].set_title('IMU Trajectory')
axs[1].set_xlabel('UTM_easting (m)')
axs[1].set_ylabel('UTM_northing (m)')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()

Y2 = np.zeros(len(vel_imu))
for i in range(len(vel_imu)):
	Y2[i] = imu_data['IMU.angular_velocity.z'][i]*vel_imu[i]

acc_y, stationary_periods2 = correct_accel2(imu_data['IMU.linear_acceleration.y'])
acc_y = butterworth_filter(acc_y,imu_freq,1,'low')
stat_values = np.zeros(len(stationary_periods2))
plt.plot(imu_time, Y2, color='r')
plt.plot(imu_time, acc_y, color='green')
plt.legend(['Omega.X','Yobs'])
plt.xlabel('Time (sec)')
plt.ylabel('Accelerarion (m/sec^2)')
plt.show()
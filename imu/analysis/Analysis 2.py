import rosbag
import numpy as np
import matplotlib.pyplot as plt
from allantools import oadev

# data_arr = {'hr5_imu_angular_velocity_x':[], 'hr5_imu_angular_velocity_y':[], 'hr5_imu_angular_velocity_z':[]}

def read_rosbag(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    data = {'orientation_x':[], 'orientation_y':[], 'orientation_z':[], 'orientation_w':[],'magnetic_field_x': [], 'magnetic_field_y': [], 'magnetic_field_z': [], 'linear_acceleration_x': [], 'linear_acceleration_y': [], 'linear_acceleration_z': [], 'angular_velocity_x': [], 'angular_velocity_y': [], 'angular_velocity_z': [], 'timeimu':[], 'data5hr':[], 'time5hr':[]}

    for topic, msg, t in bag.read_messages(topics=[topic]):
        if (topic=='/imu'):
            data['orientation_x'].append(msg.imu.orientation.x)
            data['orientation_y'].append(msg.imu.orientation.y)
            data['orientation_z'].append(msg.imu.orientation.z)
            data['orientation_w'].append(msg.imu.orientation.w)

            data['magnetic_field_x'].append(msg.mag_field.magnetic_field.x)
            data['magnetic_field_y'].append(msg.mag_field.magnetic_field.y)
            data['magnetic_field_z'].append(msg.mag_field.magnetic_field.z)

            data['linear_acceleration_x'].append(msg.imu.linear_acceleration.x)
            data['linear_acceleration_y'].append(msg.imu.linear_acceleration.y)
            data['linear_acceleration_z'].append(msg.imu.linear_acceleration.z)

            data['angular_velocity_x'].append(msg.imu.angular_velocity.x)
            data['angular_velocity_y'].append(msg.imu.angular_velocity.y)
            data['angular_velocity_z'].append(msg.imu.angular_velocity.z)

            data['timeimu'].append(msg.header.stamp.to_sec())
        
        # if (topic=='/vectornav'):
        #     data['data5hr'].append(msg.data)
        #     data['time5hr'].append(msg.header.stamp.to_sec())
        #     temp=msg.data
        #     # read_5hr_data(temp)

    bag.close()

    for key in data:
        data[key] = np.array(data[key])

    return data

# def read_5hr_data(raw_data):
#     imu_data = raw_data.split(',')
#     temp = imu_data[-1].split('*')

#     # Convert the filter objects to lists of floats
#     angular_velocity_x = [float(val) for val in filter(str.isdigit, imu_data[10])]
#     angular_velocity_y = [float(val) for val in filter(str.isdigit, imu_data[11])]
#     angular_velocity_z = [float(val) for val in filter(str.isdigit, temp[0])]

#     # Append the converted values to data_arr
#     data_arr['hr5_imu_angular_velocity_x'].extend(angular_velocity_x)
#     data_arr['hr5_imu_angular_velocity_y'].extend(angular_velocity_y)
#     data_arr['hr5_imu_angular_velocity_z'].extend(angular_velocity_z)

def to_euler_angles(qx,qy,qz,qw):
    yaw = np.arctan2(2 * (qx*qy + qw*qz), qw**2 + qx**2 - qy**2 - qz**2)
    pitch = np.arcsin(2 * (qw*qy - qx*qz))
    roll = np.arctan2(2 * (qw*qx + qy*qz), qw**2 - qx**2 - qy**2 + qz**2)
    
    yaw = np.degrees(yaw)
    pitch = np.degrees(pitch)
    roll = np.degrees(roll)
    
    return yaw, pitch, roll

def plot_histogram(ax, data, bins, color, alpha, label, xlabel, ylabel, title):
    ax.hist(data, bins=bins, color=color, alpha=alpha, label=label)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

# def calculate_allan_variance():
#     # Calculate the Allan variance for gyro X, Y, and Z
#     av_x = oadev(data_arr['hr5_imu_angular_velocity_x'], taus='all')
#     print("done 1")
#     av_y = oadev(data_arr['hr5_imu_angular_velocity_y'], taus='all')
#     print("done 2")
#     av_z = oadev(data_arr['hr5_imu_angular_velocity_z'], taus='all')
#     print("done 3")


#     # Plot the Allan variance for each axis
#     plt.figure(figsize=(10, 6))
#     plt.loglog(av_x['taus'], av_x['adev'], label='Gyro X')
#     plt.loglog(av_y['taus'], av_y['adev'], label='Gyro Y')
#     plt.loglog(av_z['taus'], av_z['adev'], label='Gyro Z')
#     plt.xlabel('Tau (s)')
#     plt.ylabel('Allan Deviation')
#     plt.title('Allan Variance Plot for Gyro X, Y, Z')
#     plt.legend()
#     plt.grid(True)
#     plt.show()

#     # Find the index of the local minimum Allan variance for each axis
#     min_index_x = np.argmin(av_x['adev'])
#     min_index_y = np.argmin(av_y['adev'])
#     min_index_z = np.argmin(av_z['adev'])

#     # Extract rate random walk (K), angle random walk (N), and bias stability (B) parameters
#     K_x = av_x['slope'][min_index_x + 1]
#     K_y = av_y['slope'][min_index_y + 1]
#     K_z = av_z['slope'][min_index_z + 1]

#     B_x = av_x['adev'][min_index_x]
#     B_y = av_y['adev'][min_index_y]
#     B_z = av_z['adev'][min_index_z]

#     N_x = av_x['adev'][1]
#     N_y = av_y['adev'][1]
#     N_z = av_z['adev'][1]

#     # Print the extracted parameters
#     print("Rate Random Walk (K) - Gyro X:", K_x)
#     print("Rate Random Walk (K) - Gyro Y:", K_y)
#     print("Rate Random Walk (K) - Gyro Z:", K_z)

#     print("Bias Instability (B) - Gyro X:", B_x)
#     print("Bias Instability (B) - Gyro Y:", B_y)
#     print("Bias Instability (B) - Gyro Z:", B_z)

#     print("Angle Random Walk (N) - Gyro X:", N_x)
#     print("Angle Random Walk (N) - Gyro Y:", N_y)
#     print("Angle Random Walk (N) - Gyro Z:", N_z)

if __name__ == "__main__":
    data_imu = read_rosbag("/home/palaniappan_yeagappan/imu/data/data4.bag", "/imu")
    data_5hr = read_rosbag("/home/palaniappan_yeagappan/imu/data/LocationC.bag", "/vectornav")

    timeimu_start = data_imu['timeimu'][0]
    data_imu['timeimu'] -= timeimu_start
        

    plt.figure(figsize=(8,6))
    plt.plot(data_imu['timeimu'], data_imu['linear_acceleration_x'], color='red', label='Linear acceleration X', marker='o')
    plt.plot(data_imu['timeimu'], data_imu['linear_acceleration_y'], color='blue', label='Linear acceleration Y', marker='*')
    plt.plot(data_imu['timeimu'], data_imu['linear_acceleration_z'], color='green', label='Linear acceleration Z', marker='x')
    plt.title("Acceleration (m/s^2)")
    plt.xlabel("Time (sec)")
    plt.ylabel("Linear Acceleration (m/s^2)")
    plt.grid(True)
    plt.legend()
    plt.show()

    plt.figure(figsize=(8,6))
    plt.plot(data_imu['timeimu'], data_imu['angular_velocity_x'], color='red', label='Angular Velocity X', marker='o')
    plt.plot(data_imu['timeimu'], data_imu['angular_velocity_y'], color='blue', label='Angular Velocity Y', marker='*')
    plt.plot(data_imu['timeimu'], data_imu['angular_velocity_z'], color='green', label='Angular Velocity Z', marker='x')
    plt.title("Rotational rate (degrees/s)")
    plt.xlabel("Time (sec)")
    plt.ylabel("Angular Velocity (degrees/s)")
    plt.grid(True)
    plt.legend()
    plt.show()

    yaw, pitch, roll = to_euler_angles(data_imu['orientation_x'], data_imu['orientation_y'], data_imu['orientation_z'], data_imu['orientation_w'])

    plt.figure(figsize=(8,6))
    plt.plot(data_imu['timeimu'], yaw, color='red', label='Yaw', marker='o')
    plt.plot(data_imu['timeimu'], pitch, color='blue', label='Pitch', marker='*')
    plt.plot(data_imu['timeimu'], roll, color='green', label='Roll', marker='x')
    plt.title("Rotation from the VN estimation (degress)")
    plt.xlabel("Time (sec)")
    plt.ylabel("Rotation (degrees)")
    plt.grid(True)
    plt.legend()
    plt.show()

    fig, axes = plt.subplots(3, 1, figsize=(8, 18))

    plot_histogram(axes[0], yaw, bins=30, color='red', alpha=0.5, label='Yaw', xlabel='Yaw', ylabel='Frequency', title='Yaw Rotation 1D histogram')
    plot_histogram(axes[1], pitch, bins=30, color='blue', alpha=0.5, label='Pitch', xlabel='Pitch', ylabel='Frequency', title='Pitch Rotation 1D histogram')
    plot_histogram(axes[2], roll, bins=30, color='green', alpha=0.5, label='Roll', xlabel='Roll', ylabel='Frequency', title='Roll Rotation 1D histogram')

    plt.grid(True)
    plt.show()
    
    # calculate_allan_variance()

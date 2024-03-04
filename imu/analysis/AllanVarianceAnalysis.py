import pandas as pd
import numpy as np
import allantools as alan
import matplotlib.pyplot as plt

def isVNYMRinString(inputString):
    inputString = inputString.strip()  
    
    if inputString.startswith('$VNYMR'):
        return True
    elif inputString == '$VNYMR':  
        return True
    else:
        return False

df = pd.read_csv("/home/palaniappan_yeagappan/Downloads/LocationC/vectornav.csv")
data_string = df["data"].tolist() 
time = df["Time"].tolist() 

gyro_x = []
gyro_y = []
gyro_z = []
for s in range(len(data_string)):
    inputString = data_string[s].split(',')

    if isVNYMRinString(str(inputString[0])) != True:
        continue
            
    Time = time[s]
    angular_rate_x = float(inputString[10])
    angular_rate_y = float(inputString[11])
    temp = str(inputString[12].split('*')[0])
    try:
        gyro_value = float(temp)
    except ValueError:
        gyro_value = 0
    else:
        pass
    angular_rate_z = gyro_value

    gyro_x.append(angular_rate_x)
    gyro_y.append(angular_rate_y)
    gyro_z.append(angular_rate_z)

np_gyro_x = np.array(gyro_x)
np_gyro_y = np.array(gyro_y)
np_gyro_z = np.array(gyro_z)

taus_x, ad_x, ade_x, ns_x = alan.oadev(np_gyro_x, rate=40.0, data_type="phase", taus="all")
print("Checkpoint 1 Achieved")
taus_y, ad_y, ade_y, ns_y = alan.oadev(np_gyro_y, rate=40.0, data_type="phase", taus="all")
print("Checkpoint 2 Achieved")
taus_z, ad_z, ade_z, ns_z = alan.oadev(np_gyro_z, rate=40.0, data_type="phase", taus="all")
print("Checkpoint 3 Achieved")

plt.title("Allan Deviation Plot")
plt.xlabel("Time (s)")
plt.ylabel("Allan Deviation")
plt.loglog(taus2_x, ad_x, label=f'Allan deviation X', color="Red")
plt.loglog(taus2_y, ad_y, label=f'Allan deviation Y', color="Blue")
plt.loglog(taus2_z, ad_z, label=f'Allan deviation Z', color="Green")

plt.legend()
plt.show()

def extract_parameters(taus, ad):
    BI_idx = np.argmin(ad)
    BI_adev = ad[BI_idx]
    BI_tau = taus[BI_idx]
    BI = (BI_adev / 0.664) * 3600
    
    idx_1s = (np.abs(taus - 1)).argmin()
    N_adev = ad[idx_1s]
    N = N_adev * 60
    
    mid_point_start = len(taus) // 3
    mid_point_end = 2 * len(taus) // 3
    log_taus = np.log10(taus[mid_point_start:mid_point_end])
    log_ad = np.log10(ad[mid_point_start:mid_point_end])
    slope, intercept = np.polyfit(log_taus, log_ad, 1)
    if -0.6 < slope < -0.4:
        K_idx = mid_point_start + np.argmin(np.abs(slope + 0.5))
        K_adev = ad[K_idx]
        K_tau = taus[K_idx]
        K = np.sqrt(2 * (K_adev**2) * K_tau)
    else:
        K = None

    return K, N, BI

K_x, N_x, B_x = extract_parameters(taus_x, ad_x)
K_y, N_y, B_y = extract_parameters(taus_y, ad_y)
K_z, N_z, B_z = extract_parameters(taus_z, ad_z)

print("Gyro X - K:", K_x, "N:", N_x, "B:", B_x)
print("Gyro Y - K:", K_y, "N:", N_y, "B:", B_y)
print("Gyro Z - K:", K_z, "N:", N_z, "B:", B_z)

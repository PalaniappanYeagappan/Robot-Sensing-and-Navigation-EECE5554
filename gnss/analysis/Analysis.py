#!/usr/bin/env python3

import rosbag
import pandas as pd
import numpy as np
from bagpy import bagreader
import matplotlib.pyplot as plt
from scipy.stats import linregress

def process_bag_data(bag_file, topic='/gps'):
    reader = bagreader(bag_file)
    data = reader.message_by_topic(topic)
    df = pd.read_csv(data)

    easting = df['utm_easting']
    northing = df['utm_northing']
    altitude = df['altitude']
    time = df['Time']

    first_easting, first_northing = easting.iloc[0], northing.iloc[0]
    easting -= first_easting
    northing -= first_northing

    centroid_easting, centroid_northing = np.mean(easting), np.mean(northing)
    deviation_easting, deviation_northing = easting - centroid_easting, northing - centroid_northing

    processed_data = {
        'easting': easting,
        'northing': northing,
        'altitude': altitude,
        'time': time,
        'centroid_easting': centroid_easting,
        'centroid_northing': centroid_northing,
        'deviation_easting': deviation_easting,
        'deviation_northing': deviation_northing
    }

    print("Centroid Easting:", centroid_easting)
    print("Centroid Northing:", centroid_northing)
    print("Deviation in Easting:", deviation_easting)
    print("Deviation in Northing:", deviation_northing)

    return processed_data

ros_topic = '/gps'

stationary_data_uno = process_bag_data('/home/palaniappan_yeagappan/gnss/data/stationary_unocc.bag', ros_topic)
stationary_data_o = process_bag_data('/home/palaniappan_yeagappan/gnss/data/stationary_occ.bag', ros_topic)
walking_data = process_bag_data('/home/palaniappan_yeagappan/gnss/sdata/walking_unocc.bag', ros_topic)

def plot_scatter(x, y, color, label, marker):
    plt.scatter(x, y, c=color, label=label, marker=marker)

def plot_line(x, y, color, linestyle, label):
    slope, intercept, _, _, _ = linregress(x, y)
    x_fit = np.linspace(min(x), max(x), 100)
    y_fit = slope * x_fit + intercept
    plt.plot(x_fit, y_fit, color=color, linestyle=linestyle, label=label)

def plot_histogram(data, bins, color, alpha, label, xlabel, ylabel, title):
    plt.hist(data, bins=bins, color=color, alpha=alpha, label=label)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)

def plot_altitude_time(time, altitude, color, label, marker):
    plt.plot(time.values, altitude.values, color=color, label=label, marker=marker)
    plt.xlabel("Time (sec)")
    plt.ylabel("Altitude (m)")
    plt.legend()

# Plotting
plt.figure(figsize=(8, 6))
plot_scatter(stationary_data_uno['deviation_easting'], stationary_data_uno['deviation_northing'], 'red', 'Not Occluded', 'x')
plot_scatter(stationary_data_o['deviation_easting'], stationary_data_o['deviation_northing'], 'blue', 'Occluded', '*')
plt.xlabel("UTM Easting (m)")
plt.ylabel("UTM Northing (m)")
plt.legend()
plt.title("Stationary Northing vs. Easting Scatterplot")
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
plot_altitude_time(stationary_data_uno['time_data'], stationary_data_uno['altitude_data'], 'red', 'Not Occluded', 'x')
plot_altitude_time(stationary_data_o['time_data'], stationary_data_o['altitude_data'], 'blue', 'Occluded', '*')
plt.title("Stationary Altitude")
plt.grid(True)
plt.show()

plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plot_histogram(stationary_data_uno['deviation_easting'], 30, 'red', 1, 'Not Occluded', "UTM Easting (m)", "Frequency", "Stationary Easting Histogram (Not Occluded)")

plt.subplot(1, 2, 2)
plot_histogram(stationary_data_o['deviation_easting'], 30, 'blue', 1, 'Occluded', "UTM Easting (m)", "Frequency", "Stationary Easting Histogram (Occluded)")

plt.tight_layout()
plt.show()

# Euclidean distance histograms
euclidean_distance_uno = np.sqrt(stationary_data_uno['deviation_easting']**2 + stationary_data_uno['deviation_northing']**2)
euclidean_distance_o = np.sqrt(stationary_data_o['deviation_easting']**2 + stationary_data_o['deviation_northing']**2)

plt.figure(figsize=(8, 6))
plot_histogram(euclidean_distance_uno, 30, 'red', 1, 'Not Occluded', "Euclidean Distance", "Frequency", "Euclidean Distance Histogram")
plot_histogram(euclidean_distance_o, 30, 'blue', 1, 'Occluded', "Euclidean Distance", "Frequency", "Euclidean Distance Histogram")
plt.grid(True)
plt.show()

# Moving data scatterplot with line of best fit
plt.figure(figsize=(8, 6))
plot_scatter(walking_data['deviation_easting'], walking_data['deviation_northing'], 'red', 'Moving', 'o')
# plot_line(walking_data['deviation_easting'], walking_data['deviation_northing'], 'black', '--', 'Line of Best Fit')
plt.xlabel("UTM Easting (m)")
plt.ylabel("UTM Northing (m)")
plt.legend()
plt.title("Moving Data Scatterplot")
plt.grid(True)
plt.show()

# Moving data altitude vs. time plot
plt.figure(figsize=(8, 6))
plot_altitude_time(walking_data['time_data'], walking_data['altitude_data'], 'red', 'Walking Data', 'o')
plt.title("Moving Data Altitude")
plt.grid(True)
plt.show()

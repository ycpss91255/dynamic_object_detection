#!/usr/bin/env python

from sklearn.datasets import make_blobs
from sklearn.metrics import silhouette_score
from sklearn.cluster import KMeans
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sklearn.metrics import silhouette_score
from sklearn.cluster import KMeans

import rospy
import numpy as np
import time
import tf
import struct
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2

import pandas as pd

import copy





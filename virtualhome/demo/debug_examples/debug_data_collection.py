from tqdm import tqdm
import sys, os
import json

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *


if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
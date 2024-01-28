from pydoc import apropos
import numpy as np
import math
import argparse

parser = argparse.ArgumentParser(description = "single variable least-squares regression")
parser.add_argument("type")
parser.add_argument("degree")
args = parser.parse_args()

types = ["polynomial", "exponential"]

type = args.type
degree = int(args.degree)

if not type in types:
    print("Invalid regression type")
    exit()

if degree < 0:
    print("Degree must be at least 0")
    exit()

x_values = [
    13.8,
    -3.64,
    -12.68,
    -19.93,
    -23.34
]

y_values = [
    53,
    42,
    31,
    26,
    22
]

if type == "polynomial":
    A = np.mat([[]])
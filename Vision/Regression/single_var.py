from pydoc import apropos
import numpy as np
import math
import argparse

parser = argparse.ArgumentParser(description = "single variable least-squares regression")
parser.add_argument("type")
parser.add_argument("degree")
args = parser.parse_args()

types = ["poly", "exp"]

type = args.type
degree = int(args.degree)

if not type in types:
    print("Invalid regression type")
    exit()

if degree < 0:
    print("Degree must be at least 0")
    exit()

x_list = [
    13.8,
    -3.64,
    -12.68,
    -19.93,
    -23.34
]

y_list = [
    53,
    42,
    31,
    26,
    22
]

if type == "poly":
    A = np.mat([[x ** (degree - j) for j in range(degree + 1)] for x in x_list])
if type == "exp":
    A = np.mat([[math.e ** (x * (degree - j)) for j in range(degree + 1)] for x in x_list])

B = np.mat([[y] for y in y_list])

p = np.matmul(np.linalg.inv(np.matmul(np.transpose(A), A)), np.matmul(np.transpose(A), B))

print(p)

if type == "poly":
    eq_str = "y=" + "+".join([f"{p[i, 0]}x^{degree - i}" for i in range(degree + 1)])
if type == "exp":
    eq_str = "y=" + "+".join([f"{p[i, 0]}e^{degree - i}x" for i in range(degree + 1)])

print(eq_str)
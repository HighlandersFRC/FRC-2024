import json as jason
import math
import argparse

parser = argparse.ArgumentParser(description = 'Script for generating Limelight compatible JSON fmap file from a list of AprilTag poses')
parser.add_argument('in_file_name')
parser.add_argument('out_file_name')

args = parser.parse_args()

HALF_FIELD_LENGTH = 16.59128 / 2
HALF_FIELD_WIDTH = 8.211312 / 2

in_file = args.in_file_name
out_file = args.out_file_name

def deg_to_rad(deg: float):
    return (deg * math.pi) / 180

def in_to_m(inches: float):
    return inches / 39.37

with open(f'input/{in_file}') as fi:
    data_in = jason.load(fi)
    poses = data_in['poses']
    transforms = []
    for pose in poses:
        transform = [
                        math.cos(deg_to_rad(pose['pose'][3])),
                        -math.sin(deg_to_rad(pose['pose'][3])),
                        0,
                        in_to_m(pose['pose'][0]) - HALF_FIELD_LENGTH,
                        math.sin(deg_to_rad(pose['pose'][3])),
                        math.cos(deg_to_rad(pose['pose'][3])),
                        0,
                        in_to_m(pose['pose'][1]) - HALF_FIELD_WIDTH,
                        0,
                        0,
                        1,
                        in_to_m(pose['pose'][2]),
                        0,
                        0,
                        0,
                        1
                    ]
        transforms.append({
            'transform': transform,
            'id': pose['id']
        })
    data_out = {}
    data_out['fiducials'] = []
    for transform in transforms:
        data_out['fiducials'].append({
            'family': data_in['family'],
            'id': transform['id'],
            'size': data_in['size'],
            'unique': 1,
            'transform': transform['transform']
        })
    fo = open(f'output/{out_file}', 'w')
    jason.dump(data_out, fo, indent = 2)
    fo.close()
    fi.close()
    print("Finished successfully!")
#!/usr/bin/env python

import sys
import yaml

# print yaml.dump({'name': 'Silenthand Olleander', 'race': 'Human', 'traits': ['ONE_HAND', 'ONE_EYE']})

#with open("/mnt/nixbig/data/project_AA1_2_extrinsics__phone_data_recording/VOS_data_2017_12_29_01_36_48/2017_12_29_01_36_48_savedData.txt", 'r') as stream:

def parse_some_yaml(filename_):
    with open(filename_, 'r') as stream:
        try:
            print(yaml.load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            
            
if __name__ == "__main__":            
    parse_some_yaml(sys.argv[1])

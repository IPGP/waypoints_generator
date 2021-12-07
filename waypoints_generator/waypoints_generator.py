#!/usr/bin/python
# -*- coding:utf-8 -*-
import json
# Json valider
# https://jsonlint.com/


# Read JSON file
with open('drones.json') as data_file:
    data_loaded = json.load(data_file)

print(json.dumps(data_loaded, indent=2, sort_keys=False))

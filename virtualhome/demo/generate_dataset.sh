#!/bin/bash

# python generate_scene_graphs.py
# python generate_scripts.py --graph_dir example_graphs

python collect_data.py --target_classes book
python collect_data.py --target_classes toy --clean_surfaces desk kitchentable plate --seed 41 # unsure??
python collect_data.py --target_classes magazine --clean_surfaces wallshelf desk --seed 41
python collect_data.py --target_classes folder --clean_surfaces wallshelf desk
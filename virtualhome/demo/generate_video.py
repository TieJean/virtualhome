# Generate video for a program. Make sure you have the executable open
import sys

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz


script = ['<char0> [Walk] <tv> (1)', '<char0> [switchon] <tv> (1)', '<char0> [Walk] <sofa> (1)', '<char0> [Sit] <sofa> (1)', '<char0> [Watch] <tv> (1)'] # Add here your script

print('Starting Unity...')
comm = UnityCommunication(port="8080")
comm.timeout_wait = 300 

print('Starting scene...')
comm.reset()

# success, graph = comm.environment_graph()
# Example node: {'id': 11, 'category': 'Rooms', 'class_name': 'bathroom', 'prefab_name': 'PRE_ROO_Bathroom_00', 'obj_transform': {'position': [-6.385, -0.003, -0.527], 'rotation': [0.0, 0.0, 0.0, 1.0], 'scale': [1.0, 1.0, 1.0]}, 'bounding_box': {'center': [-5.135, 1.247, 0.723], 'size': [8.0, 3.0, 5.5]}, 'properties': [], 'states': []}

vidpath = "./Output/"
camera_name = "observer_camera"

comm.add_character('Chars/Female1', initial_room="kitchen")
comm.add_character_camera(position=[1.5, 1.0, 0.0], rotation=[0, -60, 0], field_view=60, name="observer_camera")
print('Generating video...')
comm.render_script(script, 
                   random_seed=100,
                   recording=True, 
                   find_solution=True, 
                   save_pose_data=True,
                   file_name_prefix=camera_name,
                   camera_mode=["observer_camera"])

utils_viz.generate_video(input_path="../../unity_output/", prefix=camera_name, output_path="../../outputs/", frame_rate=5)

# comm.reset()
# print('Generating video...')
# comm.render_script(script, 
#                    random_seed=100,
#                    recording=True, 
#                    find_solution=True, 
#                    save_pose_data=True,
#                    file_name_prefix='FIRST_PERSON',
#                    camera_mode=["FIRST_PERSON"])

print('Generated.')

from tqdm import tqdm
import sys, os

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *

debug_dir = "../../outputs"
scene_id = 4

comm = UnityCommunication(port="8080")
comm.timeout_wait = 300 

views = []
for scene_id in [scene_id]:
    comm.reset(scene_id)
    
    # We will go over the line below later
    comm.remove_terrain()
    top_view = get_scene_cameras(comm, [-2])
    views += top_view
    
view_pil = display_grid_img(views, nrows=1)
view_pil.save(os.path.join(debug_dir, "top_view.png"))

comm.reset(scene_id)

imgs_prev = get_scene_cameras(comm, [-scene_id])
view_pil = display_grid_img(imgs_prev, nrows=1)

success, graph = comm.environment_graph()
sofa = find_nodes(graph, class_name='sofa')[-1]
add_node(graph, {'class_name': 'cat', 
                   'category': 'Animals',
                   "prefab_name": "Cat_1", 
                   'id': 1000, 
                   'properties': [], 
                   'states': []})
add_edge(graph, 1000, 'ON', sofa['id'])
add_node(graph, {'class_name': 'cat', 
                   'category': 'Animals',
                   "prefab_name": "Cat_2", 
                   'id': 1001, 
                   'properties': [], 
                   'states': []})
add_edge(graph, 1001, 'ON', sofa['id'])
success, message = comm.expand_scene(graph)

# success, graph = comm.environment_graph()
# fridge = find_nodes(graph, class_name='fridge')[0]
# fridge['states'] = ['OPEN']
# success = comm.expand_scene(graph)

imgs_final = get_scene_cameras(comm, [-4])
view_pil = display_grid_img(imgs_prev+imgs_final, nrows=1)
view_pil.save(os.path.join(debug_dir, "scene4_final.png"))

comm.add_character('chars/Female2', initial_room='kitchen')
comm.add_character_camera(position=[1.5, 1.0, 0.0], rotation=[0, -60, 0], field_view=90, name="observer_camera")
s, g = comm.environment_graph()
cat_id = [node['id'] for node in g['nodes'] if node['class_name'] == 'cat'][0]

s, nc = comm.camera_count()
indices = range(nc - 6, nc)
imgs_prev = get_scene_cameras(comm, indices)
view_pil = display_grid_img(imgs_prev, nrows=2)
view_pil.save(os.path.join(debug_dir, "character_view.png"))

script = ['<char0> [Walk] <sofa> ({})'.format(sofa['id']),
          '<char0> [Find] <cat> ({})'.format(cat_id),
          '<char0> [Grab] <cat> ({})'.format(cat_id),
          '<char0> [Sit] <sofa> ({})'.format(sofa['id'])]

success, message = comm.render_script(script=script[:1],
                                      processing_time_limit=120,
                                      find_solution=False,
                                      image_width=640,
                                      image_height=480,  
                                      skip_animation=False,
                                      recording=True,
                                      save_pose_data=True,
                                      camera_mode=["observer_camera"],
                                      image_synthesis=["normal", "seg_class"],
                                      file_name_prefix='relax_0')
print("finish part I")
input_path = os.path.abspath('../../unity_output/')
output_path = os.path.abspath('../../outputs/')
utils_viz.generate_video(input_path=input_path, prefix='relax_0', output_path=output_path)

success, message = comm.render_script(script=script[1:],
                                      processing_time_limit=120,
                                      find_solution=False,
                                      image_width=640,
                                      image_height=480,  
                                      skip_animation=False,
                                      recording=True,
                                      save_pose_data=True,
                                      camera_mode=["observer_camera"],
                                      file_name_prefix='relax_1')

print("finish part II")

input_path = os.path.abspath('../../unity_output/')
output_path = os.path.abspath('../../outputs/')
utils_viz.generate_video(input_path=input_path, prefix='relax_1', output_path=output_path)
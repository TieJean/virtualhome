from tqdm import tqdm
import sys, os

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
from utils_demo import *

debug_dir = "../../outputs"
n_scenes = 2

comm = UnityCommunication(port="8080")
comm.timeout_wait = 300 

# views = []
# for scene_id in tqdm(range(n_scenes)):
#     comm.reset(scene_id)
    
#     # We will go over the line below later
#     comm.remove_terrain()
#     top_view = get_scene_cameras(comm, [-2])
#     views += top_view
    
# view_pil = display_grid_img(views, nrows=1)
# view_pil.save(os.path.join(debug_dir, "top_view.png"))

comm.reset(4)

imgs_prev = get_scene_cameras(comm, [-4])
view_pil = display_grid_img(imgs_prev, nrows=1)

success, graph = comm.environment_graph()
sofa = find_nodes(graph, class_name='sofa')[-1]
add_node(graph, {'class_name': 'cat', 
                   'category': 'Animals', 
                   'id': 1000, 
                   'properties': [], 
                   'states': []})
add_edge(graph, 1000, 'ON', sofa['id'])
success, message = comm.expand_scene(graph)

success, graph = comm.environment_graph()
fridge = find_nodes(graph, class_name='fridge')[0]
fridge['states'] = ['OPEN']
success = comm.expand_scene(graph)

imgs_final = get_scene_cameras(comm, [-4])
view_pil = display_grid_img(imgs_prev+imgs_final, nrows=1)
view_pil.save(os.path.join(debug_dir, "scene4_final.png"))

comm.add_character('chars/Female2', initial_room='kitchen')
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

success, message = comm.render_script(script=script,
                                      processing_time_limit=60,
                                      find_solution=False,
                                      image_width=320,
                                      image_height=240,  
                                      skip_animation=False,
                                      recording=True,
                                      save_pose_data=True,
                                      file_name_prefix='relax')

input_path = os.path.abspath('../../unity_output/')
output_path = os.path.abspath('../../outputs/')
utils_viz.generate_video(input_path=input_path, prefix='relax', output_path=output_path)
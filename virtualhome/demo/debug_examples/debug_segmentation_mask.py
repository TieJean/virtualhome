import json
import sys

sys.path.append('../simulation')
from unity_simulator.comm_unity import UnityCommunication
from unity_simulator import utils_viz
sys.path.append(".")
from utils_demo import *
from graph_utils import *
from ros_utils import *

def example(comm):
    with open("../../unity_output/scene4_754ab231d3_0/0/graph.json", "r") as f:
        graph = json.load(f)
    if graph is None:
        raise ValueError("Graph is None, please check the file path or content.")
    comm.reset(4)
    success, message = comm.expand_scene(graph)
    if not success:
        raise RuntimeError(f"Failed to expand scene: {message}")
    
    s, nc_before = comm.camera_count()
    prepare_pano_character_camera(comm)
    comm.add_character('chars/Female2')
    s, nc_after = comm.camera_count()
    cameras_select = list(range(nc_before, nc_after))
    pano_camera_select = cameras_select[8:14]
    
    position = [2.625752, 0.8725489, -10.35497]
    success = comm.move_character(0, position)
    if not success:
        raise RuntimeError("Failed to move character to the specified position.")

    # These images are in bgr
    (ok_img, normal_imgs) = comm.camera_image(pano_camera_select, mode="normal")
    (ok_img, cls_imgs) = comm.camera_image(pano_camera_select, mode="seg_class")
    cls_imgs = bgr_to_rgb_imgs(cls_imgs)
    prefab_classes, class_list = load_prefab_metadata("../resources/PrefabClass.json")
    
    ### Case 1 - Specifying specific class
    class_name = "book"
    target_color = semantic_cls_to_rgb(class_name, class_list)
    
    ### Case 2 - Specifying specific class
    sample_img = cls_imgs[0].reshape(-1, 3)
    # Remove black pixels
    non_black = sample_img[~np.all(sample_img == 0, axis=1)]
    assert len(non_black) > 0, "No non-black pixels found"
    # Pick a random non-black pixel
    import random
    rand_rgb = tuple(non_black[random.randint(0, len(non_black) - 1)])
    # Decode its class
    class_name = semantic_rgb_to_cls(rand_rgb, class_list)
    print(f"Randomly selected color: {rand_rgb} → Class: {class_name}")
    target_color = np.array(rand_rgb, dtype=np.uint8)
    
    masked_imgs = []
    for img in cls_imgs:
        assert img.dtype == np.uint8 and img.shape[-1] == 3, "Expected RGB uint8 image"
        match_mask = np.all(img == target_color, axis=-1)
        masked_img = np.zeros_like(img)
        masked_img[match_mask] = img[match_mask]
        masked_imgs.append(masked_img)
    view_pil = display_grid_img(normal_imgs + masked_imgs, nrows=4)
    view_pil.save("../../outputs/debug_cls_inst.png")
    
if __name__ == "__main__":
    comm = UnityCommunication(port="8080")
    comm.timeout_wait = 300 
    example(comm)
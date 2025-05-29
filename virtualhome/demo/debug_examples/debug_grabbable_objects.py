import json

if __name__ == "__main__":
    prefab_file = "../resources/PrefabClassCustomed.json"
    with open(prefab_file, "r") as f:
        prefab_data = json.load(f)['prefabClasses']
    property_file = "../resources/properties_data_unity.json"
    with open(property_file, "r") as f:
        property_data = json.load(f)
    classname_equiv_file = "../resources/class_name_equivalence.json"
    with open(classname_equiv_file, "r") as f:
        classname_equivalences = json.load(f)
        
    grabbable_objects_data = {
        k: v for k, v in property_data.items() if "GRABBABLE" in v
    }
    
    class_to_prefabs = {}
    for class_name in grabbable_objects_data:
        for entry in prefab_data:
            if entry["className"].lower() == class_name.lower():
                class_to_prefabs[class_name] = entry["prefabs"]
                break  # Assume only one match needed
            
    candidate_class_to_prefabs = {
        k: v for k, v in class_to_prefabs.items() if len(v) > 2
    }
    
    import pdb; pdb.set_trace()
import rclpy
from ros2node.api import get_node_names
import json
from jinja2 import Environment, FileSystemLoader
from ament_index_python.packages import get_package_share_directory
import os

def main():
    rclpy.init()
    setup_file_generator = rclpy.create_node('graph_config_generator')

    wants_hidden_nodes = input("Include hidden nodes? (y/n): ").strip().lower() == 'y'
    available_nodes = get_node_names(node=setup_file_generator, include_hidden_nodes=wants_hidden_nodes)

    for node in available_nodes:
        if node.name == 'graph_config_generator':
            available_nodes.remove(node)

    all_nodes: dict[str, list[dict[str, str]]] = {}
    for node in available_nodes:
        ns = node.namespace
        if ns not in all_nodes:
            all_nodes[ns] = []
        all_nodes[ns].append({"node_name": node.name, "full_name": node.full_name})
            

    print("Available namespaces:", list(all_nodes.keys())) 

    ignore_namespaces = input("Enter namespaces to ignore all related nodes (comma-separated), or leave blank: ").strip()
    ignore_namespaces_list = [ns.strip() for ns in ignore_namespaces.split(',')] if ignore_namespaces else []

    blacklisted_nodes: dict[str, list[dict[str,str]]] = {"blacklisted_nodes": []}
    for ns in ignore_namespaces_list:
        if ns in all_nodes:
            for node_dict in all_nodes[ns]:
                blacklisted_nodes["blacklisted_nodes"].append({
                    "node_name": node_dict["node_name"],
                    "namespace": ns,
                    "full_name": node_dict["full_name"],
                })
            del all_nodes[ns]

    needed_namespaces = input("Enter namespaces to include all related nodes(comma-separated), or leave blank: ").strip()
    needed_namespaces_list = [ns.strip() for ns in needed_namespaces.split(',')] if needed_namespaces else []

    needed_nodes: dict[str, list[dict[str,str]]] = {"needed_nodes": []}
    for ns in needed_namespaces_list:
        if ns in all_nodes:
            for node_dict in all_nodes[ns]:
                needed_nodes["needed_nodes"].append({
                    "node_name": node_dict["node_name"],
                    "namespace": ns,
                    "full_name": node_dict["full_name"],
                })
            del all_nodes[ns]

    for ns, node_dicts in all_nodes.items():
        print(f"Namespace: {ns}")
        for node_dict in node_dicts:
            name = node_dict["node_name"]
            full_name = node_dict["full_name"]
            include = input(f"  Include node '{name}'? (y/n): ").strip().lower()
            if include == 'n':
                blacklisted_nodes["blacklisted_nodes"].append({
                    "node_name": name,
                    "namespace": ns,
                    "full_name": full_name,
                })
            else:
                needed_nodes["needed_nodes"].append({
                    "node_name": name,
                    "namespace": ns,
                    "full_name": full_name,
                })

    def _strip_fullname(list_of_dicts: list[dict[str, str]]) -> list[dict[str, str]]:
        return [{"node_name": d["node_name"], "namespace": d["namespace"]} for d in list_of_dicts]

    final_data = {
        "needed_nodes": _strip_fullname(needed_nodes["needed_nodes"]),
        "blacklisted_nodes": _strip_fullname(blacklisted_nodes["blacklisted_nodes"]),
    }
    json_output = json.dumps(final_data, indent=4)

    graph_config_dir = "/home/dockuser/ros_ws/src/mapek/config/graph_config.json"
    with open(graph_config_dir, "w") as f:
        f.write(json_output)
        print("Wrote graph_config.json to", graph_config_dir)

    template_path = os.path.join(get_package_share_directory('experiment_setup'), 'templates')
    env = Environment(loader=FileSystemLoader(template_path))
    template = env.get_template('main_bt_template.jinja')
    data = [{"full_name": n["full_name"]} for n in needed_nodes["needed_nodes"]]
    bt_output = template.render(needed_nodes=data)

    main_bt_dir = "/home/dockuser/ros_ws/src/bt_mape_k/bts/main_bt.xml"
    with open(main_bt_dir, "w") as f:
        f.write(bt_output)
        print("Wrote bt_main.xml to", main_bt_dir)


    setup_file_generator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
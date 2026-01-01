import os
import xml.etree.ElementTree as ET

"""
Used to convert BARN world files from gazebo-classic to gazebo harmonic. 
"""

PLUGINS = [
    {"filename": "gz-sim-physics-system", "name": "gz::sim::systems::Physics"},
    {"filename": "gz-sim-user-commands-system", "name": "gz::sim::systems::UserCommands"},
    {"filename": "gz-sim-scene-broadcaster-system", "name": "gz::sim::systems::SceneBroadcaster"},
    {"filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors"},
    {"filename": "gz-sim-contact-system", "name": "gz::sim::systems::Contact"},
    {"filename": "gz-sim-imu-system", "name": "gz::sim::systems::Imu"},
    {"filename": "gz-sim-navsat-system", "name": "gz::sim::systems::NavSat"}
]

input_folder = 'BARN'
output_folder = 'ROS2_BARN'
os.makedirs(output_folder, exist_ok=True)

for filename in os.listdir(input_folder):
    if not filename.endswith('.world'):
        continue

    input_path = os.path.join(input_folder, filename)
    output_path = os.path.join(output_folder, filename)

    # 1. Load the .world file
    tree = ET.parse(input_path)
    root = tree.getroot()

    world = root.find('world')
    if world is None:
        raise ValueError("No <world> element found in the file.")

    # 2. Insert the plugin elements at the start of the world element
    for i, p in enumerate(PLUGINS):
        plugin_elem = ET.Element('plugin', attrib={"filename": p["filename"], "name": p["name"]})
        if p["name"] == "gz::sim::systems::Sensors":
            re = ET.SubElement(plugin_elem, 'render_engine')
            re.text = 'ogre2'
        world.insert(i, plugin_elem)

    # 3. Iterate over direct child models
    for model in world.findall('model'):
        name = model.get('name', '')
        if name.startswith("unit_cylinder"):
            # 4. Append element (contact sensor) to each link
            for link in model.findall('link'):
                sensor = ET.Element('sensor', attrib={'name':'sensor_contact', 'type':'contact'})
                contact = ET.SubElement(sensor, 'contact')
                ET.SubElement(contact, 'collision').text = 'collision'
                link.append(sensor)
            # 5. Append element (touch plugin) to the model
            touch_plugin = ET.Element('plugin', attrib={"filename":"gz-sim-touchplugin-system",
                                                    "name":"gz::sim::systems::TouchPlugin"})
            ET.SubElement(touch_plugin, 'target').text = 'robot'
            ET.SubElement(touch_plugin, 'namespace').text = 'robot'
            ET.SubElement(touch_plugin, 'time').text = '0.001'
            ET.SubElement(touch_plugin, 'enabled').text = 'true'
            model.append(touch_plugin)

    # 6. Save patched world file
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"Patched {filename} → {output_path}")

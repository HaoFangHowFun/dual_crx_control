"""Build driver descriptions with the selected real or mock hardware plugin."""
import math
import xml.etree.ElementTree as ET
import xacro

MOCK_INITIAL_POSITIONS = {
    'left': [0.0, 0.0, 0.0, 0.0, -math.pi / 2, 0.0],
    'right': [-math.pi / 2, 0.0, math.pi, 0.0, math.pi / 2, 0.0],
}


def arm_description(xacro_path, side, robot_ip, mock):
    description = xacro.process_file(xacro_path, mappings={
        'robot_ip': robot_ip, 'use_mock': str(mock).lower(),
        'prefix': f'{side}_', 'child_link': f'{side}_ee_mount', 'motion_control': '1',
    }).toxml()
    if not mock:
        return description
    # The driver's mock macro does not expose initial-position arguments.
    # Add ros2_control's standard initial_value parameter to the expanded XML.
    root = ET.fromstring(description)
    for index, position in enumerate(MOCK_INITIAL_POSITIONS[side], start=1):
        interface = root.find(
            f"ros2_control/joint[@name='{side}_J{index}']/state_interface[@name='position']")
        if interface is None:
            raise ValueError(f'Missing mock position interface for {side}_J{index}')
        initial = interface.find("param[@name='initial_value']")
        if initial is None:
            initial = ET.SubElement(interface, 'param', name='initial_value')
        initial.text = str(position)
    return ET.tostring(root, encoding='unicode')

"""Joint names and initial software-mock configuration (radians at interfaces)."""
SIDES = ('left', 'right')
JOINT_NAMES = {side: [f'{side}_J{i}' for i in range(1, 7)] for side in SIDES}
INITIAL_JOINTS_DEG = {
    'left': [0., 0., 0., 0., -90., 0.],
    'right': [-90., 0., 180., 0., 90., 0.],
}


def canonical_side(namespace):
    aliases = {'robot1': 'right', 'robot2': 'left', '': 'left'}
    side = aliases.get(namespace.strip('/'), namespace.strip('/'))
    if side not in SIDES:
        raise ValueError(f'Unsupported arm namespace: {namespace}')
    return side

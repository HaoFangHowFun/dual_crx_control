"""Exercise the legacy motion entry points against the common ROS joint pipeline."""
import importlib.util
from pathlib import Path
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import xacro

from dual_crx_control.interpolation_node import InterpolationNode

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('name', ['test2_j1_periotic','dual_test2_periodic',
                                  'dual_test3_sychronized_motion','dual_test3_sychronized_motion_infinite'])
def test_motion_entrypoint_uses_joint_node(name):
    rclpy.init(domain_id=179)
    xml=xacro.process_file(str(ROOT/'urdf/dual_crx.urdf.xacro')).toxml()
    params=[Parameter('robot_description',value=xml)]
    mock=load('dual_mock_robot').DualMockRobot(parameter_overrides=params)
    interpolation=InterpolationNode(parameter_overrides=params)
    executor=SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(interpolation)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    module=load(name)
    kwargs=dict(motion_period=.3,amplitude_deg=.05,joint=1,initial_hold_time=.1,
                ramp_time=.1,rate_hz=50.,plot_file='',show_plot=False)
    if not name.endswith('infinite'):
        kwargs['run_time']=.3
    if name=='dual_test2_periodic':
        kwargs['robot_namespace']='right'
    if 'dual_test3' in name:
        kwargs['robot_namespaces']=('left','right')
    motion=module.JointTest(**kwargs)
    starts={s:q.copy() for s,q in mock.positions.items()}
    try:
        deadline=time.monotonic()+4
        while not motion.target_client.available() and time.monotonic()<deadline:
            rclpy.spin_once(motion,timeout_sec=.01)
        assert motion.target_client.available()
        with patch('builtins.input',return_value=''), patch.object(motion,'plot_response'):
            if name.endswith('infinite'):
                publish=motion.publish_pair
                calls=[]
                def bounded(*args):
                    calls.append(1)
                    if len(calls)>20:
                        raise KeyboardInterrupt()
                    return publish(*args)
                with patch.object(motion,'publish_pair',side_effect=bounded):
                    motion.run_test()
            else:
                motion.run_test()
        time.sleep(.05)
        assert interpolation.command_count>40
        assert not any('forward_position_controller/commands' in p.topic_name for p in motion.publishers)
        if not name.endswith('infinite'):
            for side in starts:
                np.testing.assert_allclose(mock.positions[side],starts[side],atol=1e-6)
    finally:
        time.sleep(.03)
        executor.shutdown()
        thread.join(timeout=3)
        motion.destroy_node()
        interpolation.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()

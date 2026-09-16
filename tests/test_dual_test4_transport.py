"""Test4 transport against live joint-only interpolation, independent of path planning."""
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import xacro

from dual_crx_control.interpolation_node import InterpolationNode

ROOT=Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not (ROOT / 'scripts/dual_test4.py').exists(),
                                reason='legacy dual_test4 script is absent')


def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec)
    import sys
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def test_joint_pair_transport_and_read_only_capture():
    pytest.importorskip('pinocchio', reason='optional legacy test4 planning dependency')
    rclpy.init(domain_id=180)
    params=[Parameter('robot_description',value=xacro.process_file(str(ROOT/'urdf/dual_crx.urdf.xacro')).toxml())]
    mock=load('dual_mock_robot').DualMockRobot(parameter_overrides=params)
    interpolation=InterpolationNode(parameter_overrides=params)
    executor=SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(interpolation)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    module=load('dual_test4')
    robot=SimpleNamespace(arms=[SimpleNamespace(namespace='robot1',q_indices=np.arange(6)),
                               SimpleNamespace(namespace='robot2',q_indices=np.arange(6,12))],
                          initial_q=np.zeros(12))
    args=SimpleNamespace(rate=50.,state_timeout=.25,ready_timeout=3.)
    io=module.RobotIO(robot,args)
    reader=module.RobotIO(robot,args,read_only=True)
    try:
        io.wait_ready()
        reader.wait_ready()
        assert not reader.publishers
        with pytest.raises(RuntimeError,match='cannot publish'):
            reader.publish_pair(np.zeros(12))
        target=io.read_q()
        target[[0,6]] += .001
        io.publish_pair(target)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            io.spin()
            if np.allclose(io.read_q(),target,atol=1e-10):
                break
        np.testing.assert_allclose(io.read_q(),target,atol=1e-10)
        assert interpolation.command_count >= 3
        assert not any('forward_position_controller/commands' in p.topic_name for p in io.node.publishers)
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        reader.node.destroy_node()
        io.node.destroy_node()
        interpolation.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()

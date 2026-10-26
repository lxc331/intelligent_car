"""Offline mapping SSH boundaries, ownership and dual-save verification.

No physical car, SSH server or ROS installation is accessed by these tests.
"""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

from carapp.core.mapping_remote import (MappingRemote, MappingRemoteError, MappingSaveError,
                                       remote_command, validate_map_name, validate_remote_path)
from carapp.core.mapping_remote_scripts import REMOTE_SCRIPT, SUPERVISOR


class MappingValidationTests(unittest.TestCase):
    def test_paths_are_absolute_and_cannot_traverse(self):
        for value in ('relative', '~/maps', '/home/../etc', '/tmp/ok\nBAD', '/tmp/x\x00', 'C:\\maps'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_remote_path(value)
        self.assertEqual(validate_remote_path('/home/ubuntu/maps/'), '/home/ubuntu/maps')
        self.assertEqual(validate_remote_path('/tmp/建筑 沙盘'), '/tmp/建筑 沙盘')

    def test_map_names_work_on_both_windows_and_linux(self):
        self.assertEqual(validate_map_name('校园 沙盘-01'), '校园 沙盘-01')
        for value in ('', '../bad', 'bad/name', 'bad\\name', 'bad\nname', 'NUL', 'COM1', 'x' * 65, '.map', 'x;bad', 'x$(id)'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_map_name(value)

    def test_remote_values_are_single_quoted_json_argument(self):
        path = "/tmp/quote' ; $(touch unwanted)"
        args = shlex.split(remote_command('directories', 'host', path=path))
        self.assertEqual(args[:2], ['python3', '-c'])
        self.assertEqual(json.loads(args[-1])['path'], path)
        args = shlex.split(remote_command('directories', 'container', path=path))
        self.assertEqual(args[:7], ['sudo', '-n', 'docker', 'exec', '-u', 'ubuntu', 'ClawPi'])
        self.assertEqual(json.loads(args[-1])['path'], path)

    def test_ros_actions_cannot_execute_in_host_scope(self):
        with self.assertRaises(ValueError):
            remote_command('start', 'host')

    def test_remote_payloads_compile_and_only_start_slam(self):
        compile(REMOTE_SCRIPT, 'mapping remote', 'exec')
        compile(SUPERVISOR, 'mapping supervisor', 'exec')
        self.assertIn("'sync_slam_toolbox_node'", REMOTE_SCRIPT)
        self.assertNotIn('slam.launch.py', REMOTE_SCRIPT)
        self.assertNotIn('killall', REMOTE_SCRIPT)
        self.assertNotIn('pkill', REMOTE_SCRIPT)
        self.assertIn("os.getpgid(pid)!=pid", REMOTE_SCRIPT)
        self.assertIn("record['birth']", REMOTE_SCRIPT)
        self.assertIn('CARAPP_MAPPING_OWNER', REMOTE_SCRIPT)
        self.assertIn("st_mtime>90", SUPERVISOR)


class FakeRemote(MappingRemote):
    def __init__(self):
        super().__init__('no-network.invalid', 'test', password='DO_NOT_LEAK')
        self.operations = []
        self.payloads = {'map.pgm': b'P5\n2 2\n255\n\x00\xfe\xcd\x00',
                         'map.yaml': b'image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n'}
        self.fail_download = False
        self.fail_remote = False
        self.bad_name = False
        self.serialized = False
        self.canonical_paths = {}

    def _remote(self, operation, scope='container', **values):
        self.operations.append((operation, scope, values))
        if operation == 'canonical_directory':
            return {'path': self.canonical_paths.get(values['path'], values['path'])}
        if operation == 'save':
            self.folder = values['folder']
            result = dict(remote_dir=values['directory'] + '/' + self.folder, files=self.descriptors(),
                          serialized=self.serialized, warnings=[], scope=scope)
            if self.fail_remote:
                result['error'] = 'map was unavailable'
            if self.bad_name:
                result['files'][0]['name'] = '../escape.pgm'
            return result
        if operation == 'mkdir_unique':
            return {'path': values['path'] + '/' + values['folder']}
        if operation == 'manifest':
            return {'files': self.descriptors()}
        return {}

    def descriptors(self):
        return [dict(name=name, size=len(value), sha256=hashlib.sha256(value).hexdigest())
                for name, value in self.payloads.items()]

    def _download(self, path, scope, target, descriptor):
        self.operations.append(('download', scope, descriptor['name']))
        if self.fail_download:
            raise OSError('interrupted transfer')
        Path(target).write_bytes(self.payloads[descriptor['name']])

    def _execute(self, command, **kwargs):
        self.operations.append(('execute', command, kwargs))
        return b''


class MappingSaveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'savedMap'
        self.remote = FakeRemote()

    def tearDown(self):
        self.remote.close()
        self.temp.cleanup()

    def test_container_save_publishes_complete_portable_map_with_manifest(self):
        result = self.remote.save_map('/home/ubuntu/maps', '沙盘', self.root)
        destination = Path(result['local_dir'])
        self.assertEqual(destination.parent, self.root)
        self.assertTrue((destination / 'map.pgm').exists())
        self.assertTrue((destination / 'map.yaml').exists())
        manifest = (destination / 'manifest.json').read_text(encoding='utf-8')
        self.assertNotIn('DO_NOT_LEAK', manifest)
        self.assertEqual(json.loads(manifest)['files'], self.remote.descriptors())
        self.assertFalse(list(self.root.glob('.mapping_*')))

    def test_every_save_is_new_and_existing_maps_remain(self):
        one = self.remote.save_map('/maps', 'campus', self.root)
        two = self.remote.save_map('/maps', 'campus', self.root)
        self.assertNotEqual(one['local_dir'], two['local_dir'])
        self.assertTrue(Path(one['local_dir']).is_dir())

    def test_user_selected_symlink_directory_saves_under_its_canonical_target(self):
        self.remote.canonical_paths['/home/ubuntu/map_link'] = '/media/maps'
        result = self.remote.save_map('/home/ubuntu/map_link', 'campus', self.root)
        self.assertTrue(result['remote_dir'].startswith('/media/maps/campus_'))
        self.assertEqual(result['requested_remote_dir'], '/home/ubuntu/map_link')
        self.assertTrue(Path(result['local_dir']).exists())

    def test_download_failure_reports_remote_success_without_local_success(self):
        self.remote.fail_download = True
        with self.assertRaises(MappingSaveError) as caught:
            self.remote.save_map('/maps', 'campus', self.root)
        self.assertTrue(caught.exception.partial['remote_dir'].startswith('/maps/campus_'))
        self.assertIsNone(caught.exception.partial['local_dir'])
        self.assertFalse(list(self.root.iterdir()))

    def test_remote_failure_never_downloads_or_reports_both_complete(self):
        self.remote.fail_remote = True
        with self.assertRaises(MappingSaveError) as caught:
            self.remote.save_map('/maps', 'campus', self.root)
        self.assertIsNone(caught.exception.partial['local_dir'])
        self.assertFalse(list(self.root.iterdir()))

    def test_manifest_traversal_is_rejected_before_local_write(self):
        self.remote.bad_name = True
        with self.assertRaises(MappingSaveError):
            self.remote.save_map('/maps', 'campus', self.root)
        self.assertFalse(list(self.root.iterdir()))

    def test_nonportable_yaml_is_rejected_without_publishing(self):
        self.remote.payloads['map.yaml'] = b'image: /home/ubuntu/map.pgm\nresolution: 0.05\n'
        with self.assertRaises(MappingSaveError):
            self.remote.save_map('/maps', 'campus', self.root)
        self.assertFalse(list(self.root.iterdir()))

    def test_host_scope_uses_checked_docker_bridge_then_downloads_host_files(self):
        result = self.remote.save_map('/home/pi/maps', 'campus', self.root, scope='host')
        self.assertTrue(result['remote_dir'].startswith('/home/pi/maps/'))
        self.assertTrue(result['container_dir'].startswith('/tmp/'))
        save = next(values for op, scope, values in self.remote.operations if op == 'save')
        self.assertEqual(save['directory'], '/tmp')
        commands = [shlex.split(scope) for op, scope, values in self.remote.operations if op == 'execute']
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0][:4], ['sudo', '-n', 'docker', 'cp'])

    def test_downloads_renew_owner_lease_before_and_after_each_bounded_transfer(self):
        self.remote.save_map('/maps', 'campus', self.root)
        operations = [item[0] for item in self.remote.operations]
        for index, operation in enumerate(operations):
            if operation == 'download':
                self.assertEqual(operations[index - 1], 'heartbeat')
                self.assertEqual(operations[index + 1], 'heartbeat')
        self.assertEqual(operations[operations.index('save') - 1], 'heartbeat')

    def test_host_copies_renew_owner_lease_before_and_after_each_copy(self):
        self.remote.save_map('/home/pi/maps', 'campus', self.root, scope='host')
        operations = [item[0] for item in self.remote.operations]
        for index, operation in enumerate(operations):
            if operation == 'execute':
                self.assertEqual(operations[index - 1], 'heartbeat')
                self.assertEqual(operations[index + 1], 'heartbeat')

    def test_optional_serialized_state_copies_both_files(self):
        self.remote.payloads.update({'map.posegraph': b'graph', 'map.data': b'scans'})
        self.remote.serialized = True
        result = self.remote.save_map('/maps', 'campus', self.root)
        self.assertTrue(result['serialized'])
        self.assertTrue((Path(result['local_dir']) / 'map.posegraph').exists())
        self.assertTrue((Path(result['local_dir']) / 'map.data').exists())

    def test_input_rejected_before_remote_operation(self):
        with self.assertRaises(ValueError):
            self.remote.save_map('/tmp/../root', 'campus', self.root)
        self.assertEqual(self.remote.operations, [])


class MappingLifecycleTests(unittest.TestCase):
    @staticmethod
    def fake_channel(done_after=1):
        state = dict(polls=0, closed=False)
        def ready():
            state['polls'] += 1
            return state['polls'] >= done_after
        channel = types.SimpleNamespace(exec_command=lambda value: None, recv_ready=lambda: False,
                                        recv_stderr_ready=lambda: False, exit_status_ready=ready,
                                        recv_exit_status=lambda: 0, close=lambda: state.update(closed=True))
        transport = types.SimpleNamespace(is_active=lambda: True, open_session=lambda **kwargs: channel)
        client = types.SimpleNamespace(get_transport=lambda: transport, close=lambda: None)
        return client, state

    def test_network_guard_is_forced_initially_then_throttled_to_three_seconds(self):
        calls = []
        remote = MappingRemote('invalid', 'test', connection_guard=lambda: calls.append(True))
        remote.client, channel_state = self.fake_channel(done_after=7)
        with patch('carapp.core.mapping_remote.time.monotonic', side_effect=[0, 0, 1, 2, 3, 4, 5, 6, 7]), \
                patch('carapp.core.mapping_remote.time.sleep'):
            self.assertEqual(remote._execute('read-only'), b'')
        self.assertEqual(len(calls), 3)  # First call, then t=3 and t=6.
        self.assertTrue(channel_state['closed'])

    def test_close_is_detected_on_next_loop_without_waiting_for_network_guard(self):
        calls = []
        remote = MappingRemote('invalid', 'test', connection_guard=lambda: calls.append(True))
        remote.client, channel_state = self.fake_channel(done_after=99)
        with patch('carapp.core.mapping_remote.time.monotonic', side_effect=[0, 0, .1, .2]), \
                patch('carapp.core.mapping_remote.time.sleep', side_effect=lambda value: remote.close()):
            with self.assertRaisesRegex(MappingRemoteError, '关闭'):
                remote._execute('read-only')
        self.assertEqual(len(calls), 1)
        self.assertTrue(channel_state['closed'])

    def test_close_disconnects_without_stop_or_velocity(self):
        remote = FakeRemote()
        closed = []
        remote.client = types.SimpleNamespace(close=lambda: closed.append(True))
        remote.close()
        self.assertTrue(closed)
        self.assertEqual(remote.operations, [])

    def test_disconnected_status_stop_do_not_execute_remote_commands(self):
        remote = MappingRemote('invalid', 'test')
        self.assertFalse(remote.status()['connected'])
        self.assertFalse(remote.stop()['owned'])

    def test_connection_guard_runs_before_tcp_connection(self):
        remote = MappingRemote('invalid', 'test', connection_guard=lambda: (_ for _ in ()).throw(ConnectionError('wrong car')))
        with patch('socket.create_connection') as network:
            with self.assertRaises(ConnectionError):
                remote.connect()
        network.assert_not_called()

    def test_owner_token_is_fresh_for_each_session(self):
        self.assertNotEqual(MappingRemote('one', 'test').token, MappingRemote('one', 'test').token)

    def test_streaming_download_checks_remote_hash_and_size(self):
        remote = MappingRemote('invalid', 'test')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'map.pgm'
            descriptor = dict(name='map.pgm', size=4, sha256=hashlib.sha256(b'good').hexdigest())
            with patch.object(remote, '_execute', side_effect=lambda command, **kwargs: kwargs['sink'].write(b'evil')):
                with self.assertRaises(MappingRemoteError):
                    remote._download('/tmp/map.pgm', 'container', path, descriptor)
            path.unlink()
            with patch.object(remote, '_execute', side_effect=lambda command, **kwargs: kwargs['sink'].write(b'good')):
                remote._download('/tmp/map.pgm', 'container', path, descriptor)
            self.assertEqual(path.read_bytes(), b'good')


def remote_functions(**overrides):
    """Compile the shipped payload functions, with explicit fake I/O globals."""
    functions = [item for item in ast.parse(REMOTE_SCRIPT).body if isinstance(item, ast.FunctionDef)]
    namespace = dict(base64=__import__('base64'), hashlib=hashlib, json=json, os=os,
                     pathlib=__import__('pathlib'), re=re, signal=signal, stat=__import__('stat'),
                     subprocess=subprocess, sys=sys, time=time)
    exec(compile(ast.Module(body=functions, type_ignores=[]), 'remote payload functions', 'exec'), namespace)
    namespace.update(overrides)
    return namespace


class RemotePayloadTests(unittest.TestCase):
    def test_lightweight_heartbeat_only_touches_a_matching_owner_without_ros(self):
        branch = next(item for item in ast.parse(REMOTE_SCRIPT).body if isinstance(item, ast.If))
        heartbeat = branch.orelse[0]
        code = compile(ast.Module(body=heartbeat.body, type_ignores=[]), 'heartbeat payload', 'exec')
        for owns_process in (True, False):
            with self.subTest(owns_process=owns_process):
                touched, results = [], []
                namespace = dict(p={'token': 'a' * 32}, re=re,
                                 owned_record=lambda *a: {'pid': 1234} if owns_process else None,
                                 touch=touched.append, output=results.append)
                exec(code, namespace)
                self.assertEqual(results, [dict(owned=owns_process, running=owns_process)])
                self.assertEqual(touched, ['/tmp/carapp_slam_' + 'a' * 32] if owns_process else [])
        payload = json.loads(shlex.split(remote_command('heartbeat', token='a' * 32))[-1])
        self.assertNotIn('_script', payload)  # No ROS sourcing/re-execution for renewal.

    def test_large_json_survives_ros_source_wrapper_without_log_tail_truncation(self):
        namespace = remote_functions()
        raw = namespace['bounded_run']([sys.executable, '-c', 'import json;print(json.dumps({"topics":["x"*500]*100}))'],
                                       timeout=5, max_output=1048576, truncate=False)
        self.assertGreater(len(raw), 12000)
        self.assertEqual(len(json.loads(raw)['topics']), 100)

    def test_oversized_json_is_rejected_instead_of_returning_truncated_json(self):
        namespace = remote_functions()
        with self.assertRaisesRegex(RuntimeError, 'JSON response'):
            namespace['bounded_run']([sys.executable, '-c', 'print("x"*20000)'],
                                      timeout=5, max_output=10000, truncate=False)

    def test_active_bridge_environment_clears_stale_domain_and_rmw_settings(self):
        class Entry:
            name = '123'
            def __truediv__(self, field):
                raw = (b'/usr/bin/python3\0/opt/ros/humble/rosbridge_websocket\0' if field == 'cmdline'
                       else b'RMW_IMPLEMENTATION=rmw_actual\0')
                return types.SimpleNamespace(read_bytes=lambda: raw)
        environ = {'ROS_DOMAIN_ID': '30', 'ROS_LOCALHOST_ONLY': '1',
                   'CYCLONEDDS_URI': 'stale', 'FASTRTPS_DEFAULT_PROFILES_FILE': 'stale',
                   'RMW_IMPLEMENTATION': 'rmw_wrong', 'UNRELATED': 'preserved'}
        namespace = remote_functions(os=types.SimpleNamespace(environ=environ),
                                     pathlib=types.SimpleNamespace(Path=lambda value: types.SimpleNamespace(iterdir=lambda: [Entry()])))
        self.assertTrue(namespace['source_environment']())
        self.assertEqual(environ, {'RMW_IMPLEMENTATION': 'rmw_actual', 'UNRELATED': 'preserved'})

    def test_start_refuses_unknown_ros_environment_instead_of_guessing(self):
        namespace = remote_functions(p={'token': 'a' * 32}, op='start',
                                     graph=lambda: {'environment_confirmed': False},
                                     open=lambda *a, **k: types.SimpleNamespace(fileno=lambda: 3))
        fcntl = types.SimpleNamespace(LOCK_EX=1, flock=lambda *a: None)
        with patch.dict(sys.modules, fcntl=fcntl), self.assertRaisesRegex(RuntimeError, 'domain'):
            namespace['ros_action']()

    def test_owner_verification_checks_token_birth_and_environment(self):
        token, pid = 'a' * 32, 1234
        root = '/tmp/carapp_slam_' + token
        record = dict(token=token, root=root, pid=pid, birth='456')
        contents = {root + '/owner.json': json.dumps(record),
                    '/proc/1234/stat': '1234 (ros2) ' + ' '.join(['0'] * 19 + ['456']),
                    '/proc/1234/environ': ('CARAPP_MAPPING_OWNER=' + token).encode() + b'\0'}
        def path(*parts):
            key = '/'.join(str(value).rstrip('/') for value in parts)
            return types.SimpleNamespace(read_text=lambda: contents[key], read_bytes=lambda: contents[key])
        namespace = remote_functions(pathlib=types.SimpleNamespace(Path=path),
                                     os=types.SimpleNamespace(getpgid=lambda value: pid))
        self.assertEqual(namespace['owned_record'](root, token)['pid'], pid)
        self.assertIsNone(namespace['owned_record'](root, 'b' * 32))
        contents['/proc/1234/stat'] = '1234 (reused PID) ' + ' '.join(['0'] * 19 + ['789'])
        self.assertIsNone(namespace['owned_record'](root, token))
        contents['/proc/1234/stat'] = '1234 (ros2) ' + ' '.join(['0'] * 19 + ['456'])
        contents['/proc/1234/environ'] = b'UNRELATED=1\0'
        self.assertIsNone(namespace['owned_record'](root, token))

    def test_external_slam_is_reused_without_process_creation(self):
        results, launched = [], []
        namespace = remote_functions(p={'token': 'a' * 32}, op='start',
                                     graph=lambda: {'nodes': ['/slam_toolbox'], 'environment_confirmed': True},
                                     owned_record=lambda root, token: None,
                                     output=results.append,
                                     subprocess=types.SimpleNamespace(Popen=lambda *a, **k: launched.append(a)),
                                     open=lambda *a, **k: types.SimpleNamespace(fileno=lambda: 3))
        fcntl = types.SimpleNamespace(LOCK_EX=1, flock=lambda *a: None)
        with patch.dict(sys.modules, fcntl=fcntl):
            namespace['ros_action']()
        self.assertFalse(results[0]['owned'])
        self.assertTrue(results[0]['running'])
        self.assertEqual(launched, [])

    def test_stop_signals_only_verified_owner_process_group(self):
        results, signals = [], []
        live = [True]
        def killpg(pid, sig):
            signals.append((pid, sig)); live[0] = False
        namespace = remote_functions(p={'token': 'a' * 32}, op='stop', output=results.append,
                                     os=types.SimpleNamespace(killpg=killpg),
                                     signal=types.SimpleNamespace(SIGINT=2, SIGTERM=15, SIGKILL=9),
                                     owned_record=lambda root, token: {'pid': 1234} if live[0] else None)
        namespace['ros_action']()
        self.assertEqual(signals, [(1234, signal.SIGINT)])
        self.assertEqual(results[0]['status'], 'stopped')

    def test_stop_without_owner_does_not_signal_anything(self):
        results, signals = [], []
        namespace = remote_functions(p={'token': 'b' * 32}, op='stop', output=results.append,
                                     os=types.SimpleNamespace(killpg=lambda *a: signals.append(a)),
                                     owned_record=lambda *a: None)
        namespace['ros_action']()
        self.assertFalse(results[0]['owned'])
        self.assertEqual(signals, [])

    def test_remote_map_save_normalizes_yaml_and_hashes_actual_files(self):
        with tempfile.TemporaryDirectory() as folder:
            results = []
            def save_cli(argv, timeout):
                prefix = Path(argv[argv.index('-f') + 1])
                prefix.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xfe\xcd\x00')
                prefix.with_suffix('.yaml').write_text('image: /absolute/original.pgm\nresolution: 0.05\n')
            namespace = remote_functions(op='save',
                                         p=dict(token='a' * 32, directory=folder, name='campus', folder='unique', serialize=False),
                                         output=results.append, absolute=lambda value: value,
                                         owned_record=lambda *a: None, bounded_run=save_cli)
            namespace['ros_action']()
            result = results[0]
            self.assertNotIn('error', result)
            self.assertEqual(result['serialized'], False)
            self.assertEqual((Path(folder) / 'unique' / 'map.yaml').read_text().splitlines()[0], 'image: map.pgm')
            self.assertEqual({item['name'] for item in result['files']}, {'map.yaml', 'map.pgm'})
            for descriptor in result['files']:
                actual = Path(result['remote_dir'], descriptor['name']).read_bytes()
                self.assertEqual(descriptor['size'], len(actual))
                self.assertEqual(descriptor['sha256'], hashlib.sha256(actual).hexdigest())

    def test_remote_partial_failure_returns_the_destination_path(self):
        with tempfile.TemporaryDirectory() as folder:
            results = []
            def unavailable(*args, **kwargs):
                raise TimeoutError('no map publisher')
            namespace = remote_functions(op='save',
                                         p=dict(token='a' * 32, directory=folder, name='campus', folder='unique'),
                                         output=results.append, absolute=lambda value: value,
                                         owned_record=lambda *a: None, bounded_run=unavailable)
            namespace['ros_action']()
            self.assertIn('error', results[0])
            self.assertEqual(Path(results[0]['remote_dir']), Path(folder) / 'unique')
            self.assertFalse(results[0]['serialized'])


if __name__ == '__main__':
    unittest.main()

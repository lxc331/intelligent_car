"""Local isolated tests; no authentication or writes on a physical vehicle."""
import base64
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

from carapp.core.ssh_session import SSHSession,query_command,LIST_SCRIPT,READ_SCRIPT
from carapp.core.terminal_buffer import TerminalBuffer


class TerminalModelTests(unittest.TestCase):
    def test_progress_line_replaces_instead_of_appending(self):
        buffer=TerminalBuffer(30,8)
        buffer.feed('progress 10%\r\x1b[Kprogress 99%')
        text,_=buffer.render()
        self.assertEqual(text.splitlines()[0].strip(),'progress 99%')

    def test_ansi_cursor_and_wide_unicode(self):
        buffer=TerminalBuffer(30,8)
        buffer.feed('中文> ')
        text,cursor=buffer.render()
        self.assertIn('中文>',text)
        self.assertEqual(cursor,(1,4))
        buffer.feed('\r\x1b[2KOK')
        self.assertEqual(buffer.render()[0].splitlines()[0].strip(),'OK')

    def test_history_is_bounded(self):
        buffer=TerminalBuffer(30,8,history=20)
        buffer.feed(''.join('row%d\r\n'%i for i in range(100)))
        self.assertLessEqual(len(buffer.render()[0].splitlines()),28)
        self.assertIn('row99',buffer.render()[0])


class FileQueryTests(unittest.TestCase):
    def run_script(self,script,path):
        output=subprocess.check_output([sys.executable,'-c',script,str(path)])
        return json.loads(output)

    def test_paths_are_one_argument_and_never_shell_code(self):
        value="/tmp/a'; touch UNWANTED; $(echo bad)"
        args=shlex.split(query_command('directory',value,'host'))
        self.assertEqual(args[-1],value)
        self.assertEqual(args[:2],['python3','-c'])
        args=shlex.split(query_command('file',value,'container'))
        self.assertEqual(args[-1],value)
        self.assertEqual(args[:7],['sudo','-n','docker','exec','-u','ubuntu','ClawPi'])

    @unittest.skipIf(os.name=='nt','Remote listing script uses Linux pwd; live tested on the vehicle')
    def test_directory_query_preserves_names_and_sorts_folders_first(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'文件夹').mkdir();(root/"quote'file.txt").write_text('test')
            entries=self.run_script(LIST_SCRIPT,root)['entries']
            self.assertTrue(entries[0]['is_dir'])
            self.assertEqual({e['name'] for e in entries},{'文件夹',"quote'file.txt"})

    def test_preview_is_bounded_and_byte_exact(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'preview.txt';payload=b'A'*300000;path.write_bytes(payload)
            data=self.run_script(READ_SCRIPT,path)
            self.assertTrue(data['truncated'])
            self.assertEqual(base64.b64decode(data['data']),payload[:262144])


class SSHLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.session=SSHSession(str(Path(self.temp.name)/'known_hosts'))

    def tearDown(self):
        self.session.close();self.temp.cleanup()

    def test_disconnected_input_not_queued(self):
        self.assertFalse(self.session.send('dangerous command\r'))
        self.assertTrue(self.session.outbound.empty())

    def test_new_connection_cannot_feed_old_vehicle_worker(self):
        self.session.connected=True
        old=self.session.outbound
        self.session.send('old\r')
        self.session.disconnect()
        self.session.connected=True
        self.session.send('new\r')
        self.assertIsNot(self.session.outbound,old)
        self.assertEqual(old.get_nowait(),b'old\r')
        self.assertTrue(old.empty())
        self.assertEqual(self.session.outbound.get_nowait(),b'new\r')

    def test_old_events_are_discarded(self):
        old=self.session.generation
        self.session.disconnect()
        self.session._event(old,'data','stale output')
        self.assertTrue(self.session.events.empty())

    def test_wifi_guard_failure_never_opens_tcp(self):
        self.session.connection_guard=lambda:(_ for _ in ()).throw(ConnectionError('wrong SSID'))
        with patch('socket.create_connection') as connect:
            self.session.connect('192.168.149.1','pi','not-to-be-logged')
            deadline=time.monotonic()+2;events=[]
            while time.monotonic()<deadline:
                item=self.session.events.get(timeout=1);events.append(item)
                if item[1]=='disconnected':break
            connect.assert_not_called()
        self.assertFalse(self.session.connected)
        self.assertNotIn('not-to-be-logged',str(events))


class TerminalWidgetTests(unittest.TestCase):
    def test_build_directory_navigation_and_key_encoding(self):
        import copy
        import tkinter as tk
        from tkinter import ttk
        from carapp.config import DEFAULTS
        from carapp.features.terminal import TerminalFeature
        from carapp.model.state import CarState
        root=tk.Tk();root.withdraw()
        cfg=copy.deepcopy(DEFAULTS);cfg['terminal']['auto_connect']=False
        app=types.SimpleNamespace(cfg=cfg,state=CarState(),log=lambda text:None,root=root,
                                 _connection_guard=lambda:None)
        ctx=types.SimpleNamespace(cfg=cfg,state=app.state,app=app,log=app.log)
        feature=TerminalFeature(ctx)
        try:
            with patch('carapp.features.terminal.local_login',return_value=''):
                feature.build(root)
            self.assertEqual(feature.title,'集成终端')
            feature._listing_request=4
            feature.directory_result({'request_id':4,'scope':'host','path':'/home/pi','entries':[
                {'name':'folder','is_dir':True,'size':0,'mtime':0},
                {'name':'test.txt','is_dir':False,'size':4,'mtime':0}]})
            self.assertEqual(len(feature.tree.get_children()),2)
            sent=[]
            feature.session.send=lambda data:sent.append(data) or True
            feature.keypress(types.SimpleNamespace(state=4,keysym='c',char=''))
            feature.keypress(types.SimpleNamespace(state=0,keysym='Up',char=''))
            feature.keypress(types.SimpleNamespace(state=0,keysym='Return',char=''))
            self.assertEqual(sent,['\x03','\x1b[A','\r'])
            feature.on_vehicle_change()
            self.assertEqual(feature.tree.get_children(),())
        finally:
            feature.on_close();root.destroy()


if __name__=='__main__':unittest.main()

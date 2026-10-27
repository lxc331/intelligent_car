"""Wi-Fi dialog regression tests; no physical networking or robot commands."""
import copy
import gc
from pathlib import Path
import tempfile
import threading
import time
import tkinter as tk
import types
import unittest
from unittest.mock import Mock,patch

from carapp.config import APP_DIR,DEFAULTS,normalize_connection_config
from carapp.features.wifi_setup import WifiSetupDialog


class WifiSetupUITests(unittest.TestCase):
    def setUp(self):
        gc.collect()
        self.root=tk.Tk();self.root.withdraw()
        self.temp=tempfile.TemporaryDirectory(dir=Path(APP_DIR).parent/'.work')
        cfg=normalize_connection_config(copy.deepcopy(DEFAULTS))
        self.listeners=[]
        self.app=type('TestApp',(),{})()
        self.app.__dict__.update(root=self.root,cfg=cfg,frame_listeners=self.listeners,
            camera=None,log=Mock(),change_host=Mock(return_value=True),_post=Mock(),
            _connection_guard=Mock(return_value=None))
        self.app.ctx=types.SimpleNamespace(motion_in_use=Mock(return_value=False),add_frame_listener=self.listeners.append)
        patch('carapp.features.wifi_setup.APP_DIR',str(Path(self.temp.name)/'app')).start()
        patch('carapp.features.wifi_setup.local_password',return_value='private-login').start()
        patch.object(tk.Toplevel,'grab_set').start()
        self.save=patch('carapp.features.wifi_setup.save_config').start()
        self.dialog=WifiSetupDialog(self.root,self.app,'new wifi')
        self.dialog.wifi_password.set('private-wifi')

    def tearDown(self):
        self.root.destroy();patch.stopall();self.temp.cleanup()
        del self.dialog,self.app,self.root
        gc.collect()

    def result(self,reachable=True):
        return {'state':'connected','submitted':True,'profile_id':'HW-10CCA58E',
                'ssid':'new wifi','ip':'192.168.1.20','reachable':reachable,
                'preference_saved':True,'boot_autostart':True,'service_active':True}

    def test_password_is_masked_and_open_network_is_explicit(self):
        self.assertTrue(self.dialog.password_entry.cget('show'))
        self.dialog.open_network.set(True);self.dialog._toggle_open()
        self.assertEqual(self.dialog.password_entry.cget('state'),'disabled')

    def test_motion_blocks_submission(self):
        self.app.ctx.motion_in_use.return_value=True
        with patch.object(self.dialog,'_start') as start:self.dialog.apply()
        start.assert_not_called();self.assertIn('停止',self.dialog.status.get())

    def test_empty_password_requires_explicit_open_network_choice(self):
        self.dialog.wifi_password.set('')
        with patch.object(self.dialog,'_start') as start:self.dialog.apply()
        start.assert_not_called();self.assertIn('开放网络',self.dialog.status.get())

    def test_submission_does_not_save_secrets_or_change_local_addresses_early(self):
        before=copy.deepcopy(self.app.cfg)
        with patch.object(self.dialog,'_start') as start,patch('carapp.features.wifi_setup.configure_wifi') as configure:
            self.dialog.apply()
            work=start.call_args.args[0];work()
        self.assertEqual(configure.call_args.args[3:5],('new wifi','private-wifi'))
        self.assertEqual(configure.call_args.kwargs['login_password'],'private-login')
        self.assertEqual(self.app.cfg,before);self.save.assert_not_called()

    def test_result_updates_only_selected_profile_without_password(self):
        before=copy.deepcopy(self.app.cfg);self.dialog.auto_connect.set(False)
        self.dialog._finished(self.result(False))
        expected=copy.deepcopy(before)
        expected['cars'][0].update(sta_host='192.168.1.20',sta_ssid='new wifi')
        self.assertEqual(self.app.cfg,expected)
        self.assertNotIn('private-wifi',str(self.app.cfg));self.assertNotIn('private-login',str(self.app.cfg))
        self.app.change_host.assert_not_called()
        self.assertIn('已保存为开机首选',self.dialog.status.get())

    def test_missing_boot_confirmation_does_not_claim_saved_or_auto_connect(self):
        result=self.result();result['boot_autostart']=False
        self.dialog._finished(result)
        self.assertIn('开机首选尚未确认',self.dialog.status.get())
        self.assertNotIn('已保存为开机首选',self.dialog.status.get())
        self.app.change_host.assert_not_called()

    def test_stopped_service_does_not_claim_saved_or_auto_connect(self):
        result=self.result();result['service_active']=False
        self.dialog._finished(result)
        self.assertIn('开机首选尚未确认',self.dialog.status.get())
        self.app.change_host.assert_not_called()

    def test_temporary_ap_result_keeps_saved_boot_preference_visible(self):
        self.dialog._finished({'state':'pending','submitted':True,'preference_saved':True,
                               'boot_autostart':True,'service_active':True,
                               'message':'尚未连接，临时回退 AP'})
        self.assertIn('已设为开机首选',self.dialog.status.get())
        self.app.change_host.assert_not_called()

    def test_changed_preference_pending_does_not_claim_boot_priority(self):
        self.dialog._finished({'state':'pending','submitted':True,'preference_saved':False,
                               'boot_autostart':True,'service_active':True,
                               'message':'检测到车端配置被其他操作更改'})
        self.assertNotIn('已设为开机首选',self.dialog.status.get())

    def test_stale_vehicle_result_never_updates_current_profile(self):
        self.app.cfg['car']['profile_id']='HW-A0857935'
        before=copy.deepcopy(self.app.cfg)
        self.dialog._finished(self.result())
        self.assertEqual(self.app.cfg,before);self.save.assert_not_called()
        self.app.change_host.assert_not_called()

    def test_search_result_connects_only_once(self):
        self.dialog._connect_after_find=True
        self.dialog._found(self.result())
        self.app.change_host.assert_called_once_with('192.168.1.20','HW-10CCA58E','sta',sta_ssid='new wifi')

    def test_explicit_connect_works_with_automatic_switch_disabled(self):
        self.dialog.auto_connect.set(False);self.dialog._connect_after_find=True
        self.dialog._found(self.result())
        self.app.change_host.assert_called_once()

    def test_failed_switch_does_not_claim_connected_or_update_profile(self):
        before=copy.deepcopy(self.app.cfg)
        self.dialog._finished({'state':'uncertain','submitted':True,'message':'连接中断，等待确认'})
        self.assertEqual(self.app.cfg,before)
        self.assertIn('等待确认',self.dialog.status.get())
        self.assertEqual(self.dialog.search.cget('state'),'normal')

    def test_worker_posts_completion_instead_of_touching_tk_from_worker(self):
        thread_ids=[]
        def work():thread_ids.append(threading.get_ident());return {'state':'pending'}
        self.dialog._start(work,'_finished')
        deadline=time.monotonic()+2
        while not self.app._post.called and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(self.app._post.called)
        self.assertNotEqual(thread_ids[0],threading.get_ident())
        self.assertEqual(self.app._post.call_args.args[2],'_finished')
        self.assertEqual(self.app._post.call_args.args[3],({'state':'pending'},))

    def test_late_worker_completion_ignores_destroyed_dialog(self):
        import weakref
        from carapp.features.wifi_setup import _deliver
        reference=weakref.ref(self.dialog)
        self.dialog.destroy()
        _deliver(reference,'_finished',({'state':'connected'},))
        self.app.change_host.assert_not_called()

    def test_closing_cancels_waiting_and_clears_password_fields(self):
        self.dialog.busy=True;self.dialog.close()
        self.assertTrue(self.dialog.cancel.is_set());self.assertTrue(self.dialog.alive)
        wifi,login=self.dialog.wifi_password,self.dialog.login_password
        self.dialog.busy=False;self.dialog.close()
        self.assertEqual(wifi.get(),'')
        self.assertEqual(login.get(),'')
        self.assertIsNone(self.dialog.wifi_password)
        self.assertNotIn(self.dialog._on_frame,self.listeners)


if __name__=='__main__':unittest.main()

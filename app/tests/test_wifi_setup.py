"""Provisioning safety and transaction checks; all SSH and robot I/O is mocked."""
import base64
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, MagicMock, patch

from carapp.config import APP_DIR
from carapp.core import wifi_setup as w


PROFILE={'id':'HW-10CCA58E','name':'1 号车','host':'192.168.150.3'}
SERIAL='7b5a8de10cca58e8'


class ValidationTests(unittest.TestCase):
    def test_personal_open_unicode_and_hex_networks(self):
        for ssid,password in [('new wifi','12345678'),('中文网络',''),('x'*32,'a'*63),('hex','a'*64)]:
            w.validate_wifi(ssid,password)

    def test_invalid_values_fail_before_network_access(self):
        for ssid,password in [('', '12345678'),('中'*11,'12345678'),('x\nwrong','12345678'),
                              ('x','short'),('x','密码'*11),('x','g'*64),('x','1234567\n')]:
            with self.subTest(ssid=ssid),self.assertRaises(ValueError):w.validate_wifi(ssid,password)

    def test_credentials_go_through_stdin_not_command_text(self):
        client=Mock();stdin,stdout,stderr=Mock(),Mock(),Mock()
        client.exec_command.return_value=(stdin,stdout,stderr)
        stdout.read.return_value=b'{"ok":true}'
        stdout.channel.recv_exit_status.return_value=0
        secret='wifi-secret-$()'
        self.assertTrue(w.remote_json(client,w.WRITE_SCRIPT,{'password':secret},sudo=True)['ok'])
        self.assertNotIn(secret,client.exec_command.call_args.args[0])
        self.assertEqual(json.loads(stdin.write.call_args.args[0])['password'],secret)
        stdin.channel.shutdown_write.assert_called_once()


class RemoteWriteTests(unittest.TestCase):
    def execute(self,profile_id=PROFILE['id'],expected=None,upgrade=False,adapter_hash=None,
                enabled=True,enable_failure=False,start_failure=False,readback_mismatch=False):
        source=b'WIFI_MODE=1\nWIFI_AP_SSID="HW-10CCA58E"\n'
        saved={}
        update=Mock(side_effect=lambda path,values:saved.update(values));lock=MagicMock()
        atomic=Mock()
        module=types.SimpleNamespace(WifiOperationLock=Mock(return_value=lock),update_config_file=update,
                                     atomic_write_text=atomic,load_config_values=lambda path:{} if readback_mismatch else dict(saved))
        service={'enabled':enabled,'active':False}
        def systemctl(argv,**kwargs):
            action=argv[1]
            if action=='enable':
                if enable_failure:raise OSError('enable failed')
                service['enabled']=True
            if action in ('start','restart'):
                if start_failure:raise OSError('start failed')
                service['active']=True
            value=('enabled' if service['enabled'] else 'disabled') if action=='is-enabled' else \
                  ('active' if service['active'] else 'inactive') if action=='is-active' else ''
            return types.SimpleNamespace(stdout=value,returncode=0)
        class RemotePath:
            def __init__(self,value):self.value=value
            def read_text(self):return SERIAL+'\0'
            def read_bytes(self):return source
            def __str__(self):return self.value
        data={'profile_id':profile_id,'ssid':"new ';$(wifi)",'password':'new-secret',
              'expected_sha256':expected or hashlib.sha256(source).hexdigest()}
        if upgrade:
            data.update(adapter=base64.b64encode(b'OPEN_NETWORK_SUPPORTED = True\n').decode(),
                        adapter_expected_sha256=adapter_hash or hashlib.sha256(source).hexdigest(),restart_service=True)
        output=io.StringIO()
        with patch.dict(sys.modules,{'wifi_config_store':module}),patch('pathlib.Path',RemotePath),\
             patch('sys.stdin',io.StringIO(json.dumps(data))),patch('sys.stdout',output),patch.object(sys,'path',list(sys.path)),\
             patch('subprocess.run',side_effect=systemctl) as restart:
            exec(compile(w.WRITE_SCRIPT,'remote_wifi_write','exec'),{})
        self.atomic_write,self.restart=atomic,restart
        return json.loads(output.getvalue()),update

    def test_write_uses_lock_and_changes_only_wifi_preference(self):
        result,update=self.execute()
        self.assertTrue(result['ok'])
        args=update.call_args.args
        self.assertEqual(str(args[0]),w.CONFIG_PATH)
        self.assertEqual(args[1],{'WIFI_MODE':2,'WIFI_STA_SSID':"new ';$(wifi)",'WIFI_STA_PASSWORD':'new-secret'})
        self.assertNotIn('new-secret',json.dumps(result))

    def test_wrong_vehicle_never_writes(self):
        result,update=self.execute(profile_id='HW-A0857935')
        self.assertEqual(result['error'],'identity');update.assert_not_called()

    def test_concurrent_configuration_change_never_overwrites(self):
        result,update=self.execute(expected='changed')
        self.assertEqual(result['error'],'changed');update.assert_not_called()

    def test_upgrade_preserves_atomic_writer_and_restarts_only_wifi_service(self):
        result,update=self.execute(upgrade=True)
        self.assertTrue(result['ok']);self.assertTrue(result['restart_ok'])
        self.assertEqual(str(self.atomic_write.call_args.args[0]),w.ADAPTER_PATH)
        self.assertEqual(result['modified_files'],[w.ADAPTER_PATH,w.CONFIG_PATH])
        commands=[c.args[0] for c in self.restart.call_args_list]
        self.assertIn(['systemctl','restart','wifi.service'],commands)
        self.assertTrue(all(c[-1]=='wifi.service' for c in commands))
        update.assert_called_once()

    def test_concurrent_adapter_change_prevents_both_writes_and_restart(self):
        result,update=self.execute(upgrade=True,adapter_hash='concurrent-change')
        self.assertEqual(result['error'],'changed')
        self.atomic_write.assert_not_called();update.assert_not_called();self.restart.assert_not_called()

    def test_saved_preference_is_verified_and_disabled_service_is_enabled(self):
        result,update=self.execute(enabled=False)
        self.assertTrue(result['preference_saved'])
        self.assertTrue(result['boot_autostart']);self.assertTrue(result['service_active'])
        self.assertIn(w.BOOT_LINK_PATH,result['modified_files'])
        self.assertIn(['systemctl','enable','wifi.service'],[c.args[0] for c in self.restart.call_args_list])

    def test_enabled_service_is_started_idempotently_without_restart(self):
        result,update=self.execute()
        commands=[c.args[0] for c in self.restart.call_args_list]
        self.assertTrue(result['boot_autostart']);self.assertTrue(result['service_active'])
        self.assertIn(['systemctl','start','wifi.service'],commands)
        self.assertNotIn(['systemctl','enable','wifi.service'],commands)
        self.assertNotIn(['systemctl','restart','wifi.service'],commands)

    def test_readback_mismatch_is_not_claimed_persistent_or_activated(self):
        result,update=self.execute(readback_mismatch=True)
        update.assert_called_once();self.restart.assert_not_called()
        self.assertFalse(result['preference_saved']);self.assertFalse(result['boot_autostart'])

    def test_failed_service_enable_reports_partial_saved_state(self):
        result,update=self.execute(enabled=False,enable_failure=True)
        self.assertTrue(result['preference_saved']);self.assertFalse(result['boot_autostart'])

    def test_failed_service_start_is_not_reported_active(self):
        result,update=self.execute(start_failure=True)
        self.assertTrue(result['preference_saved']);self.assertTrue(result['boot_autostart'])
        self.assertFalse(result['service_active']);self.assertFalse(result['restart_ok'])


class ProvisioningTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=Path(APP_DIR).parent/'.work')
        self.directory=Path(self.temp.name)
        self.client=Mock();self.peer=Mock();self.now=0.
        self.open=patch.object(w,'open_vehicle',return_value=self.client).start()
        self.remote=patch.object(w,'remote_json',side_effect=self.remote_response).start()
        self.calls=[];self.ready=True;self.drop_write=False;self.cancel=threading.Event()
        self.persisted=self.boot_enabled=self.service_active=True
        self.status_persisted=self.status_boot_enabled=self.status_service_active=True
        self.status_ssid='new wifi'
        self.backup={'ok':True,'files':{w.CONFIG_PATH:{'data':base64.b64encode(b'WIFI_MODE=1').decode(),
                                                     'sha256':'backup-hash'}}}

    def tearDown(self):
        patch.stopall();self.temp.cleanup()

    def remote_response(self,client,script,payload=None,**kwargs):
        self.calls.append((script,payload))
        if script==w.BACKUP_SCRIPT:return self.backup
        if script==w.WRITE_SCRIPT:
            self.assertTrue((self.directory/'before.json').exists())
            if self.drop_write:raise OSError('disconnect after submission')
            return {'ok':True,'modified_files':[w.CONFIG_PATH],'preference_saved':self.persisted,
                    'boot_autostart':self.boot_enabled,'service_active':self.service_active}
        return {'mode':2,'preferred_ssid':self.status_ssid,'ssid':'new wifi' if self.ready else '',
                'connected':self.ready,'ip':'192.168.1.20' if self.ready else '', 'fallback_ap':not self.ready,
                'preference_saved':self.status_persisted,'boot_autostart':self.status_boot_enabled,
                'service_active':self.status_service_active}

    def sleep(self,seconds):self.now+=seconds

    def configure(self,**overrides):
        password=overrides.pop('wifi_password','wifi-private')
        options=dict(username='pi',login_password='ssh-private',known_hosts=str(self.directory/'hosts'),
                     directory=self.directory,cancel=self.cancel,wait_seconds=4,
                     clock=lambda:self.now,sleep=self.sleep)
        options.update(overrides)
        return w.configure_wifi(PROFILE,'192.168.150.3','wired','new wifi',password,**options)

    def test_success_backups_before_write_and_checks_new_address(self):
        result=self.configure()
        self.assertEqual((result['state'],result['ip'],result['reachable']),('connected','192.168.1.20',True))
        self.assertEqual(self.open.call_args_list[-1].args[0],'192.168.1.20')
        self.assertEqual([s for s,p in self.calls].count(w.WRITE_SCRIPT),1)
        record=(self.directory/'operation.json').read_text(encoding='utf-8')
        self.assertNotIn('wifi-private',record);self.assertNotIn('ssh-private',record)
        self.assertTrue(result['preference_saved']);self.assertTrue(result['boot_autostart'])

    def test_persistence_failure_is_not_connection_success(self):
        self.persisted=False
        result=self.configure()
        self.assertEqual(result['state'],'pending');self.assertFalse(result['preference_saved'])
        self.assertFalse(any(s==w.STATUS_SCRIPT for s,p in self.calls))

    def test_autostart_failure_is_not_connection_success(self):
        self.boot_enabled=False
        result=self.configure()
        self.assertEqual(result['state'],'pending');self.assertFalse(result['boot_autostart'])
        self.assertFalse(any(s==w.STATUS_SCRIPT for s,p in self.calls))

    def test_inactive_service_is_not_connection_success(self):
        self.service_active=False
        result=self.configure()
        self.assertEqual(result['state'],'pending');self.assertFalse(result['service_active'])

    def test_concurrent_change_clears_old_saved_receipt(self):
        self.status_ssid='different network'
        result=self.configure()
        self.assertEqual(result['state'],'pending')
        self.assertFalse(result['preference_saved'])
        self.assertIn('其他操作更改',result['message'])

    def test_later_service_disable_revokes_boot_confirmation(self):
        self.status_boot_enabled=False
        result=self.configure()
        self.assertEqual(result['state'],'pending')
        self.assertFalse(result['boot_autostart'])
        self.assertIn('未设置开机自启',result['message'])

    def test_later_service_stop_revokes_running_confirmation(self):
        self.status_service_active=False
        result=self.configure()
        self.assertEqual(result['state'],'pending')
        self.assertFalse(result['service_active'])
        self.assertIn('服务已停止',result['message'])

    def test_lost_acknowledgement_is_uncertain_and_never_replayed(self):
        self.drop_write=True
        result=self.configure()
        self.assertEqual(result['state'],'uncertain');self.assertTrue(result['submitted'])
        self.assertEqual([s for s,p in self.calls].count(w.WRITE_SCRIPT),1)

    def test_failed_network_keeps_sta_preference_without_rewriting_ap(self):
        self.ready=False
        result=self.configure()
        self.assertEqual(result['state'],'pending')
        self.assertTrue(result['submitted'])
        self.assertEqual([s for s,p in self.calls].count(w.WRITE_SCRIPT),1)

    def test_connected_car_but_unreachable_pc_reports_ip_without_false_success(self):
        self.open.side_effect=[self.client,w.WifiSetupError('unreachable')]
        result=self.configure()
        self.assertEqual(result['state'],'connected');self.assertFalse(result['reachable'])
        self.assertEqual(result['ip'],'192.168.1.20')

    def test_cancellation_before_submission_never_writes(self):
        original=self.remote_response
        def read(*args,**kwargs):
            result=original(*args,**kwargs)
            if args[1]==w.BACKUP_SCRIPT:self.cancel.set()
            return result
        self.remote.side_effect=read
        result=self.configure()
        self.assertEqual(result['state'],'cancelled');self.assertFalse(result['submitted'])
        self.assertFalse(any(s==w.WRITE_SCRIPT for s,p in self.calls))

    def test_backup_failure_never_submits_configuration(self):
        with patch.object(w,'_record',side_effect=PermissionError('backup denied')):
            with self.assertRaises(PermissionError):self.configure()
        self.assertFalse(any(s==w.WRITE_SCRIPT for s,p in self.calls))

    def test_missing_recovery_service_never_writes(self):
        self.backup={'ok':False,'error':'recovery_missing'}
        with self.assertRaises(w.WifiSetupError):self.configure()
        self.assertFalse(any(s==w.WRITE_SCRIPT for s,p in self.calls))

    def test_wrong_vehicle_does_not_try_other_addresses_or_write(self):
        self.open.side_effect=w.WifiSetupError('档案不符')
        with self.assertRaises(w.WifiSetupError):self.configure()
        self.open.assert_called_once();self.remote.assert_not_called()

    def test_open_wifi_upgrades_only_recognized_old_adapter_and_requests_wifi_restart(self):
        self.backup['files'][w.ADAPTER_PATH]={'data':base64.b64encode(b'# older adapter').decode(),
                                             'sha256':w.PRE_OPEN_ADAPTER_SHA256}
        result=self.configure(wifi_password='')
        payload=next(p for s,p in self.calls if s==w.WRITE_SCRIPT)
        self.assertTrue(payload['restart_service'])
        self.assertEqual(payload['adapter_expected_sha256'],w.PRE_OPEN_ADAPTER_SHA256)
        self.assertIn('OPEN_NETWORK_SUPPORTED = True',base64.b64decode(payload['adapter']).decode())
        self.assertIn(w.ADAPTER_PATH,result['possible_modified_files'])

    def test_open_wifi_does_not_overwrite_an_unrecognized_adapter(self):
        self.backup['files'][w.ADAPTER_PATH]={'data':base64.b64encode(b'# customized').decode(),'sha256':'customized'}
        with self.assertRaises(w.WifiSetupError):self.configure(wifi_password='')
        self.assertFalse(any(s==w.WRITE_SCRIPT for s,p in self.calls))

    def test_open_wifi_preserves_an_already_supported_adapter(self):
        self.backup['files'][w.ADAPTER_PATH]={'data':base64.b64encode(b'OPEN_NETWORK_SUPPORTED = True').decode(),'sha256':'supported'}
        self.configure(wifi_password='')
        payload=next(p for s,p in self.calls if s==w.WRITE_SCRIPT)
        self.assertTrue(payload['restart_service']);self.assertNotIn('adapter',payload)


class HostIdentityTests(unittest.TestCase):
    def setUp(self):
        import paramiko
        self.temp=tempfile.TemporaryDirectory(dir=Path(APP_DIR).parent/'.work')
        self.hosts=Path(self.temp.name)/'hosts'
        self.client=Mock();self.keys=paramiko.HostKeys()
        self.client.get_host_keys.return_value=self.keys
        self.client_factory=patch('paramiko.SSHClient',return_value=self.client).start()
        self.socket=patch('socket.create_connection').start()
        self.remote=patch.object(w,'remote_json',return_value={'profile_id':PROFILE['id']}).start()

    def tearDown(self):patch.stopall();self.temp.cleanup()

    def connect(self,**kw):
        return w.open_vehicle('192.168.1.20',PROFILE['id'],'pi','secret',str(self.hosts),**kw)

    def test_identity_mismatch_is_not_trusted_or_saved(self):
        self.remote.return_value={'profile_id':'HW-A0857935'}
        with self.assertRaises(w.WifiSetupError):self.connect()
        self.client.save_host_keys.assert_not_called();self.client.close.assert_called_once()

    def test_ap_guard_rejects_before_any_connection(self):
        with self.assertRaises(w.WifiSetupError):
            self.connect(guard=Mock(side_effect=ConnectionError('wrong SSID')))
        self.socket.assert_not_called()

    def test_verified_profile_key_is_pinned_at_new_dhcp_address(self):
        import paramiko
        key=paramiko.RSAKey.generate(1024)
        self.keys.add(PROFILE['id']+'@192.168.150.3',key.get_name(),key)
        self.connect()
        self.assertEqual(self.keys.lookup(PROFILE['id']+'@192.168.1.20')[key.get_name()],key)
        self.assertIsInstance(self.client.set_missing_host_key_policy.call_args.args[0],paramiko.RejectPolicy)

    def test_discovery_requires_previously_verified_profile_before_password_auth(self):
        with self.assertRaises(w.WifiSetupError):
            w.find_wifi(PROFILE,'new',['192.168.1.20'],username='pi',login_password='secret',known_hosts=str(self.hosts))
        self.socket.assert_not_called()


if __name__=='__main__':unittest.main()

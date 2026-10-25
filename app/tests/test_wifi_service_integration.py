"""Extract vendor service entry points without importing its GPIO side effects."""
import ast
from pathlib import Path
import types
import unittest
from unittest.mock import Mock, patch

SOURCE = Path(__file__).resolve().parents[2] / 'tools/vehicle_network/wifi_service.py'


def function(name, namespace):
    node = next(n for n in ast.parse(SOURCE.read_text(encoding='utf-8')).body
                if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),namespace)
    return namespace[name]


class ServiceEntryTests(unittest.TestCase):
    def test_automatic_fallback_cannot_persist_ap_or_delete_config(self):
        namespace = {'logger': Mock(), 'update_config_file': Mock(), 'clear_directory_contents': Mock()}
        function('persist_ap_fallback', namespace)('connection unavailable')
        namespace['update_config_file'].assert_not_called()
        namespace['clear_directory_contents'].assert_not_called()

    def test_both_manual_ap_and_sta_enter_coordinator(self):
        for mode in (1,2):
            namespace = {'WIFI_MODE': mode, 'setup_client_mode': Mock()}
            function('WIFI_MGR',namespace)()
            namespace['setup_client_mode'].assert_called_once()

    def test_worker_exception_retries_with_redacted_error(self):
        adapter = Mock()
        adapter.coordinator.return_value.run.side_effect = [RuntimeError('secret-do-not-log'), KeyboardInterrupt()]
        logger, clock = Mock(), Mock()
        namespace = {'logger':logger,'time':clock}
        module = types.SimpleNamespace(WifiRecoveryAdapter=Mock(return_value=adapter))
        with patch.dict('sys.modules', {'wifi_recovery_adapter':module}):
            with self.assertRaises(KeyboardInterrupt):
                function('setup_client_mode',namespace)()
        clock.sleep.assert_called_once_with(5)
        self.assertNotIn('secret-do-not-log',str(logger.mock_calls))

    def test_disabled_led_does_not_touch_gpio(self):
        led=Mock()
        namespace = {'WIFI_LED':False,'led2':led}
        function('set_led',namespace)(2,1,0,0)
        self.assertEqual(led.mock_calls,[])

    def test_network_worker_is_not_conditional_on_led(self):
        main = next(n for n in ast.parse(SOURCE.read_text(encoding='utf-8')).body
                    if isinstance(n,ast.If) and '__main__' in ast.unparse(n.test))
        conditionals = [ast.unparse(n.test) for n in ast.walk(main) if isinstance(n,ast.If)]
        self.assertNotIn('WIFI_LED',conditionals)
        phone = next(n for n in ast.parse(SOURCE.read_text(encoding='utf-8')).body
                     if isinstance(n,ast.ClassDef) and n.name=='PhoneServer')
        self.assertTrue(any(isinstance(n,ast.Assign) and any(isinstance(t,ast.Name)
                        and t.id=='allow_reuse_address' for t in n.targets) and n.value.value
                        for n in phone.body))

    def test_reset_handler_cannot_block_server_shutdown(self):
        tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
        phone = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PhoneServer')
        self.assertEqual(ast.unparse(phone.bases[0]), 'socketserver.ThreadingTCPServer')
        self.assertTrue(any(isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
                        and t.id == 'daemon_threads' for t in n.targets) and n.value.value
                        for n in phone.body))
        self.assertTrue(any(isinstance(n, ast.Import) and any(a.name == 'socket' for a in n.names)
                            for n in tree.body))


if __name__=='__main__':
    unittest.main()

"""STA integration regression checks; no Tk window and no physical networking."""
import copy
import types
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS, normalize_connection_config
from carapp.core.connection_profiles import TRANSPORT_LABELS
from carapp.main import App
from carapp.model.state import CarState
from carapp.ui.topbar import TopBar


class Variable:
    def __init__(self, value=''):
        self.value = value
    def get(self):
        return self.value
    def set(self, value):
        self.value = value


class STAAppTests(unittest.TestCase):
    def setUp(self):
        self.cfg = normalize_connection_config(copy.deepcopy(DEFAULTS))
        self.cfg['cars'][1].update(sta_host='192.168.137.192', sta_ssid='Win11_Meng_plus')
        self.cfg['tracking_pick']['calibrations']['keep'] = {'x': .5, 'y': .7}
        self.app = App.__new__(App)
        self.app.cfg, self.app.state = self.cfg, CarState()
        self.app.log = Mock()
        self.app.ctx = types.SimpleNamespace(motion_in_use=Mock(return_value=False))
        self.feature = types.SimpleNamespace(stop_all=Mock(), on_vehicle_change=Mock(), tracking=None)
        self.app.features = [self.feature]
        self.app.topbar = types.SimpleNamespace(sync_selection=Mock())
        self.app._start_bridge, self.app.start_camera = Mock(), Mock()
        self.app.cam_topic = '/image_raw'

    def test_three_modes_save_independent_endpoints(self):
        with patch('carapp.main.save_config') as save:
            for mode, host in [('sta','192.168.137.192'),('wifi','192.168.149.1'),('wired','192.168.150.2')]:
                self.assertTrue(self.app.change_host(host, 'HW-A0857935', mode))
                self.assertEqual(self.cfg['car']['transport'], mode)
                self.assertEqual(self.cfg['car']['profile_id'], 'HW-A0857935')
            self.assertEqual(save.call_count, 3)
        car = self.cfg['cars'][1]
        self.assertEqual((car['host'], car['wifi_host'], car['sta_host']),
                         ('192.168.150.2','192.168.149.1','192.168.137.192'))
        self.assertEqual(self.cfg['tracking_pick']['calibrations']['keep'], {'x': .5, 'y': .7})

    def test_invalid_sta_has_no_session_side_effects(self):
        before = copy.deepcopy(self.cfg)
        with patch('carapp.main.save_config') as save:
            self.assertFalse(self.app.change_host('http://wrong/path', 'HW-A0857935', 'sta'))
        self.assertEqual(self.cfg, before)
        self.feature.stop_all.assert_not_called()
        self.feature.on_vehicle_change.assert_not_called()
        self.app._start_bridge.assert_not_called()
        save.assert_not_called()

    def test_public_motion_query_blocks_switch(self):
        self.app.ctx.motion_in_use.return_value = True
        before = copy.deepcopy(self.cfg)
        self.assertFalse(self.app.change_host('192.168.137.192', 'HW-A0857935', 'sta'))
        self.assertEqual(self.cfg, before)
        self.feature.on_vehicle_change.assert_not_called()
        self.feature.stop_all.assert_not_called()

    def test_sta_does_not_use_ap_ssid_guard(self):
        self.cfg['car'].update(profile_id='HW-A0857935', transport='sta')
        self.assertIsNone(self.app._connection_guard())
        self.cfg['car']['transport'] = 'wifi'
        with patch('carapp.main.require_wifi') as require:
            self.app._connection_guard()()
        require.assert_called_once_with('HW-A0857935')

    def bar(self, transport='sta'):
        bar = types.SimpleNamespace(app=self.app, cars=self.cfg['cars'],
            var_transport=Variable(TRANSPORT_LABELS[transport]),
            var_host=Variable('192.168.137.193'), var_sta_ssid=Variable('new_wifi'),
            var_car=Variable(TopBar._car_label(self.cfg['cars'][1])), sync_selection=Mock())
        bar._car_label = TopBar._car_label
        bar._selected_transport = lambda: TopBar._selected_transport(bar)
        return bar

    def test_apply_and_discovery_keep_sta_identity(self):
        self.app.change_host = Mock(return_value=True)
        bar = self.bar()
        TopBar._apply(bar)
        self.app.change_host.assert_called_with('192.168.137.193','HW-A0857935','sta',sta_ssid='new_wifi')
        TopBar._use(bar, '192.168.137.194')
        self.app.change_host.assert_called_with('192.168.137.194','HW-A0857935','sta',sta_ssid='new_wifi')

    def test_missing_sta_address_cancel_keeps_active_selection(self):
        bar = self.bar()
        bar.winfo_toplevel = lambda: None
        self.cfg['cars'][1]['sta_host'] = ''
        self.app.change_host = Mock()
        with patch('carapp.ui.topbar.simpledialog.askstring', return_value=None):
            TopBar._pick_car(bar, bar.var_car.get())
        self.app.change_host.assert_not_called()
        bar.sync_selection.assert_called_once()

    def test_cli_infers_saved_sta_address(self):
        from carapp import main as module
        with patch.object(module,'load_config',return_value=self.cfg), patch.object(module,'App') as constructor:
            module.main(['--host','192.168.137.192'])
        loaded = constructor.call_args.args[0]
        self.assertEqual((loaded['car']['transport'],loaded['car']['profile_id']),('sta','HW-A0857935'))

    def test_cli_explicit_sta_selects_profile(self):
        from carapp import main as module
        with patch.object(module,'load_config',return_value=self.cfg), patch.object(module,'App') as constructor:
            module.main(['--host','192.168.137.199','--transport','sta','--car','HW-A0857935'])
        loaded = constructor.call_args.args[0]
        self.assertEqual(loaded['cars'][1]['sta_host'],'192.168.137.199')
        self.assertEqual(loaded['cars'][1]['host'],'192.168.150.2')

    def test_cli_rejects_explicit_vehicle_address_conflict(self):
        from carapp import main as module
        with patch.object(module,'load_config',return_value=self.cfg), patch.object(module,'App') as constructor, \
                patch('sys.stderr'):
            with self.assertRaises(SystemExit):
                module.main(['--host','192.168.137.192','--car','HW-10CCA58E'])
        constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()

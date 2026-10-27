"""Page scheduling without Tk, sockets or vehicle access."""
import queue
import types
import unittest
from unittest.mock import Mock

from carapp.main import App
from carapp.model.state import CarState


class PageTickDispatchTests(unittest.TestCase):
    def app(self):
        app = App.__new__(App)
        app._closing = False
        app._ui_queue, app._log_queue, app.cam_q = queue.Queue(), queue.Queue(), queue.Queue(1)
        app.state, app.camera = CarState(), None
        app.topbar = types.SimpleNamespace(update_state=Mock())
        app.logpanel = types.SimpleNamespace(add=Mock())
        app.log = Mock()
        app.frame_listeners = []
        app.root = types.SimpleNamespace(after=Mock())
        app.features = [types.SimpleNamespace(on_background_tick=Mock(), on_tick=Mock(),
                                             on_activate=Mock(), on_deactivate=Mock()) for _ in range(3)]
        app._active_feature = app.features[1]
        return app

    def test_hidden_pages_keep_background_work_without_rendering(self):
        app = self.app()
        app._tick()
        for page in app.features:
            page.on_background_tick.assert_called_once()
        app.features[1].on_tick.assert_called_once()
        app.features[0].on_tick.assert_not_called()
        app.features[2].on_tick.assert_not_called()
        app.topbar.update_state.assert_called_once()
        app.root.after.assert_called_once_with(100, app._tick)

    def test_frame_listeners_still_receive_shared_latest_frame(self):
        app = self.app()
        listeners = [Mock(), Mock()]
        app.frame_listeners = listeners
        app.cam_q.put((123., b'camera frame'))
        app._tick()
        for listener in listeners:
            self.assertEqual(listener.call_args.args[:2], (123., b'camera frame'))
        self.assertTrue(app.cam_q.empty())

    def test_switch_deactivates_old_page_and_renders_new_page(self):
        app = self.app()
        old = app._active_feature
        app.nb = types.SimpleNamespace(index=lambda value: 2)
        app._tab_changed()
        old.on_deactivate.assert_called_once()
        app.features[2].on_activate.assert_called_once()
        app._tick()
        old.on_tick.assert_not_called()
        app.features[2].on_tick.assert_called_once()

    def test_background_processing_precedes_current_page_render(self):
        app = self.app()
        order = []
        for index, page in enumerate(app.features):
            page.on_background_tick.side_effect = lambda state, i=index: order.append(('background', i))
            page.on_tick.side_effect = lambda state, i=index: order.append(('render', i))
        app._tick()
        self.assertEqual(order, [('background', 0), ('background', 1), ('background', 2), ('render', 1)])


if __name__ == '__main__':
    unittest.main()

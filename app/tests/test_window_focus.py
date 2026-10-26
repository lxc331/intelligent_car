"""Regression checks for native ttk popdowns and focus-loss stop behavior."""

import tkinter as tk
from tkinter import ttk
import types
import unittest
from unittest.mock import Mock

from carapp.main import App
from carapp.ui.focus import window_has_focus


class FocusWindow:
    """Supply a focus path without changing the user's actual desktop focus."""

    def __init__(self, root, path):
        self.root = root
        self.path = path
        self.tk = types.SimpleNamespace(call=self.call)

    def __str__(self):
        return str(self.root)

    def call(self, *args):
        if args == ('focus',):
            return self.path
        return self.root.tk.call(*args)


class WindowFocusTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.combo = ttk.Combobox(self.root, values=['wired', 'STA'], state='readonly')
        self.popup = str(self.root.tk.call('ttk::combobox::PopdownWindow', str(self.combo)))
        self.popup_list = self.popup + '.f.l'

    def tearDown(self):
        self.root.destroy()

    def app(self, path):
        app = App.__new__(App)
        app.root = FocusWindow(self.root, path)
        app._closing = False
        app._had_window_focus = True
        app._focus_timer = 'pending-check'
        self.stop = Mock()
        app.features = [types.SimpleNamespace(stop_all=self.stop)]
        return app

    def test_native_popdown_reproduces_python_lookup_failure_but_is_internal(self):
        with self.assertRaises(KeyError) as error:
            self.root.nametowidget(self.popup_list)
        self.assertEqual(error.exception.args, ('popdown',))
        self.assertTrue(window_has_focus(FocusWindow(self.root, self.popup_list)))

    def test_callback_keeps_popup_focus_without_stopping_or_raising(self):
        app = self.app(self.popup_list)
        app._check_window_focus()
        self.stop.assert_not_called()
        self.assertTrue(app._had_window_focus)
        self.assertIsNone(app._focus_timer)

    def test_ordinary_child_keeps_window_focus(self):
        self.assertTrue(window_has_focus(FocusWindow(self.root, str(self.combo))))

    def test_no_focus_still_stops_once(self):
        app = self.app('')
        app._check_window_focus()
        app._check_window_focus()
        self.stop.assert_called_once_with()
        self.assertFalse(app._had_window_focus)

    def test_separate_toplevel_still_stops(self):
        dialog = tk.Toplevel(self.root)
        dialog.withdraw()
        entry = tk.Entry(dialog)
        app = self.app(str(entry))
        app._check_window_focus()
        self.stop.assert_called_once_with()

    def test_other_windows_combobox_popup_still_stops(self):
        dialog = tk.Toplevel(self.root)
        dialog.withdraw()
        combo = ttk.Combobox(dialog, values=['one'])
        popup = str(self.root.tk.call('ttk::combobox::PopdownWindow', str(combo)))
        app = self.app(popup + '.f.l')
        app._check_window_focus()
        self.stop.assert_called_once_with()

    def test_destroyed_focus_widget_still_stops_without_tcl_error(self):
        app = self.app(self.popup_list)
        self.combo.destroy()
        app._check_window_focus()
        self.stop.assert_called_once_with()

    def test_closing_skips_focus_lookup_and_stop(self):
        app = self.app(self.popup_list)
        app._closing = True
        app.root.tk.call = Mock(side_effect=AssertionError('should not query focus'))
        app._check_window_focus()
        self.stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()

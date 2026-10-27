"""Read held Windows keys without capturing input from other applications."""
import ctypes
import sys


def key_is_down(key):
    """Return physical key state, or None when the platform cannot report it."""
    if sys.platform != "win32" or len(key) != 1 or not key.isascii():
        return None
    return bool(ctypes.windll.user32.GetAsyncKeyState(ord(key.upper())) & 0x8000)

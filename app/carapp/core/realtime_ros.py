"""Deliver sensor snapshots without Tk, scoped to the current ROS connection.

Callbacks must only update bounded, thread-safe data. The session lock spans
delivery and resets so an old in-flight scan cannot repopulate a reset monitor.
"""
import threading


class RealtimeRosDelivery:
    def __init__(self):
        self._lock = threading.RLock()
        self._generation = None
        self._connected = False
        self._handlers = {}
        self._resets = ()

    def _reset(self):
        for callback in self._resets:
            callback()

    def begin(self, generation, handlers, reset_callbacks=()):
        with self._lock:
            self._generation = generation
            self._connected = False
            self._handlers = {topic: tuple(callbacks) for topic, callbacks in handlers.items()}
            self._resets = tuple(reset_callbacks)
            self._reset()

    def set_connected(self, generation, connected):
        with self._lock:
            if generation != self._generation:
                return False
            connected = bool(connected)
            if connected != self._connected:
                self._connected = connected
                self._reset()
            return True

    def handles(self, topic):
        with self._lock:
            return topic in self._handlers

    def deliver(self, generation, topic, msg, received_at):
        with self._lock:
            if generation != self._generation or not self._connected or not isinstance(msg, dict):
                return False
            callbacks = self._handlers.get(topic, ())
            for callback in callbacks:
                callback(msg, received_at)
            return bool(callbacks)

    def invalidate(self):
        with self._lock:
            self._generation = None
            self._connected = False
            self._reset()

"""Serial background operations for mapping; this module never touches Tk."""
import queue
import threading


class MappingSession:
    """Own one remote connection; detached sessions clean up their own SLAM only."""

    def __init__(self, remote):
        self.remote = remote
        self.events = queue.Queue()
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._state = dict(connected=False, running=False, owned=False, busy=False,
                           operation="", detail="未连接建图服务")
        self._thread = None
        self._active_operation = ""
        self._pending = None

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def submit(self, operation, *args, **kwargs):
        if operation not in ("connect", "start", "stop", "status", "save_map", "list_directories"):
            raise ValueError("未知建图操作")
        with self._lock:
            if self._closed.is_set():
                return False
            if operation != "connect" and not self._state["connected"]:
                return False
            if self._state["busy"]:
                # One user action may follow an in-flight background status
                # query. Do not interrupt its SSH channel or replay actions.
                if (self._active_operation == "status" and self._pending is None
                        and operation not in ("status", "connect")):
                    self._pending = (operation, args, kwargs)
                    self._state["operation"] = operation
                    return True
                return False
            self._state.update(busy=True, operation=operation)
            self._active_operation = operation
            self._thread = threading.Thread(target=self._run, args=(operation, args, kwargs),
                                            name="mapping-" + operation, daemon=True)
            self._thread.start()
        return True

    def _execute_operation(self, operation, args, kwargs):
        try:
            result = getattr(self.remote, operation)(*args, **kwargs) or {}
            with self._lock:
                if operation == "connect":
                    self._state["connected"] = True
                if operation in ("start", "stop", "status", "connect"):
                    for key in ("connected", "running", "owned"):
                        if key in result:
                            self._state[key] = bool(result[key])
                    if operation == "start":
                        self._state["running"] = result.get("running", True)
                    if operation == "stop":
                        self._state["running"] = result.get("running", False)
                    self._state["detail"] = result.get("detail", result.get("status", "操作完成"))
            self.events.put(dict(operation=operation, result=result, error=None))
        except Exception as exc:
            if operation == "start":
                try:
                    self.remote.stop()
                except Exception:
                    pass
            with self._lock:
                if operation in ("connect", "status"):
                    self._state.update(connected=False, running=False)
                if operation == "start":
                    self._state.update(running=False, owned=False)
                self._state["detail"] = str(exc)
            self.events.put(dict(operation=operation, result=getattr(exc, "partial", {}), error=str(exc)))

    def _run(self, operation, args, kwargs):
        while True:
            self._execute_operation(operation, args, kwargs)
            with self._lock:
                closing = self._closed.is_set()
                pending, self._pending = self._pending, None
                if not closing and pending and self._state["connected"]:
                    operation, args, kwargs = pending
                    self._active_operation = operation
                    self._state["operation"] = operation
                    continue
                if not closing and pending:
                    self.events.put(dict(operation=pending[0], result={},
                                         error="建图连接已断开，请重新连接后再操作"))
                self._state.update(busy=False, operation="")
                self._active_operation = ""
            if closing:
                self._cleanup()
            return

    def drain(self):
        result = []
        while True:
            try:
                result.append(self.events.get_nowait())
            except queue.Empty:
                return result

    def close_async(self):
        with self._lock:
            if self._closed.is_set():
                return
            self._closed.set()
            if self._state["busy"]:
                return  # The in-flight operation cleans up, including a late start.
            self._thread = threading.Thread(target=self._cleanup, name="mapping-cleanup", daemon=True)
            self._thread.start()

    def _cleanup(self):
        try:
            # Backend stop is ownership checked, including a late successful start.
            self.remote.stop()
        except Exception:
            pass
        finally:
            self.remote.close()
            with self._lock:
                self._state.update(connected=False, running=False, owned=False)


class MapFeed:
    """Keep only the latest map; parsing cannot delay manual stop input in Tk."""

    def __init__(self, model):
        self.model = model
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._worker = threading.Thread(target=self._run, name="mapping-map-parse", daemon=True)
        self._worker.start()

    def put(self, msg, received_at):
        with self._condition:
            if not self._closed:
                self._pending = (self.model.generation, msg, received_at)
                self._condition.notify()

    def reset(self):
        with self._condition:
            self._pending = None
            self.model.reset()

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                generation, msg, received_at = self._pending
                self._pending = None
            self.model.accept_map(msg, received_at, generation=generation)

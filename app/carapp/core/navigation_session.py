"""Background Nav2 operations, cancellation ordering and page-independent polling.

The remote adapter owns its heartbeat and velocity lease. This coordinator polls
status independently of Tk and never starts or replays a motion command itself.
"""
import queue
import threading
import time


_TERMINAL_GOALS = frozenset(("succeeded", "canceled", "cancelled", "aborted",
                             "failed", "rejected", "stopped", "idle", "error"))
_OPERATIONS = frozenset(("connect", "start", "status", "initial_pose", "plan", "navigate",
                         "cancel", "stop"))


class NavigationSession:
    """Serialize foreground work while keeping stop requests out of its queue.

    One foreground action can wait behind a background status query. Cancel
    supersedes it, revokes the remote motion lease immediately, and invalidates
    late responses before asking the remote adapter for serial confirmation.
    """

    def __init__(self, remote, *, heartbeat_interval=.5):
        self.remote = remote
        self.events = queue.Queue()
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._state = dict(connected=False, running=False, ready=False, owned=False,
                           busy=False, operation="", motion_active=False,
                           goal_state="idle", detail="未连接导航服务")
        self._epoch = 0
        self._active_operation = ""
        self._pending = None
        self._cancel_pending = None
        self._thread = None
        self._poll_interval = max(.05, float(heartbeat_interval))
        self._poller = threading.Thread(target=self._poll, name="navigation-status", daemon=True)
        self._poller.start()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def submit(self, operation, *args, **kwargs):
        if operation not in _OPERATIONS:
            raise ValueError("未知导航操作")
        if operation in ("cancel", "stop"):
            return self.request_cancel(stop=operation == "stop")
        with self._lock:
            if self._closed.is_set() or self._cancel_pending:
                return False
            if operation != "connect" and not self._state["connected"]:
                return False
            if operation == "connect" and (self._state["running"] or self._state["motion_active"]):
                return False
            if operation == "start" and self._state["running"]:
                return False
            if operation == "initial_pose" and not self._state["running"]:
                return False
            if operation in ("plan", "navigate") and not self._state["ready"]:
                return False
            if operation in ("plan", "initial_pose", "navigate") and self._state["motion_active"]:
                return False
            if operation == "navigate":
                # Bind acceptance to the adapter epoch before starting a
                # worker; cancellation can otherwise overtake method entry.
                capture = getattr(self.remote, "capture_intent", None)
                if capture is not None:
                    kwargs = dict(kwargs, intent_epoch=capture())
            request = (operation, args, kwargs, self._epoch)
            if self._state["busy"]:
                if self._active_operation != "status" or self._pending or operation in ("status", "connect"):
                    return False
                self._pending = request
                self._state["operation"] = operation
            else:
                if operation == "navigate":
                    self._state.update(motion_active=True, goal_state="pending")
                self._start_locked(request)
            if operation == "navigate":
                # Occupy motion before this explicit request can reach a worker.
                self._state.update(motion_active=True, goal_state="pending")
            return True

    def _start_locked(self, request):
        self._state.update(busy=True, operation=request[0])
        self._active_operation = request[0]
        self._thread = threading.Thread(target=self._run, args=(request,),
                                        name="navigation-" + request[0], daemon=True)
        self._thread.start()

    def request_cancel(self, *, stop=False):
        """Revoke motion now; queue owner-only cancellation/shutdown confirmation."""
        with self._lock:
            if self._closed.is_set():
                return False
            # Background permission checks can request stop every Tk tick.
            # Do not invalidate the confirmation already in progress.
            if self._cancel_pending == "stop" or self._active_operation == "stop":
                return True
            if not stop and (self._cancel_pending == "cancel" or self._active_operation == "cancel"):
                return True
            self._epoch += 1
            self._drop_pending_locked("操作已取消，不会在停车后重新执行")
            # A later cancel cannot downgrade an already requested shutdown.
            operation = "stop" if (stop or self._cancel_pending == "stop"
                                    or self._active_operation == "start") else "cancel"
            self._cancel_pending = operation
            self._state.update(operation=operation, goal_state="canceling", detail="正在停止导航")
            # Call the adapter's nonblocking local revoke while holding this
            # lock, so no subsequent explicit request can overtake revocation.
            try:
                self.remote.request_cancel()
            except Exception as exc:
                self.events.put(dict(operation="revoke", result={}, error=str(exc)))
            if not self._state["busy"]:
                self._cancel_pending = None
                self._start_locked((operation, (), {}, self._epoch))
            return True

    def _drop_pending_locked(self, reason):
        pending, self._pending = self._pending, None
        if pending:
            self.events.put(dict(operation=pending[0], result={}, error=reason, canceled=True,
                                 target=self._target(pending[0], pending[1])))

    @staticmethod
    def _target(operation, args):
        return tuple(args[:3]) if operation in ("plan", "navigate", "initial_pose") else None

    def _apply_result_locked(self, operation, result):
        for key in ("connected", "running", "ready", "owned"):
            if key in result:
                self._state[key] = bool(result[key])
        if operation == "connect":
            self._state["connected"] = result.get("connected", True)
        if operation == "start":
            self._state["running"] = result.get("running", True)
        if operation == "stop":
            self._state.update(running=result.get("running", False), ready=False)
        if "goal_state" in result:
            self._state["goal_state"] = result["goal_state"]
        waiting_navigation = self._pending and self._pending[0] == "navigate"
        # A disconnected adapter can revoke its local grant but cannot claim a
        # vehicle-side stop acknowledgement. Keep existing motion reserved.
        stop_confirmed = result.get("motion_stopped") is True
        stop_unconfirmed = (result.get("motion_stopped") is False or
                            (not stop_confirmed and (result.get("connected") is False
                                                     or result.get("canceled") is False)))
        if not waiting_navigation and not stop_unconfirmed:
            if stop_confirmed:
                self._state["motion_active"] = False
            elif "motion_active" in result:
                self._state["motion_active"] = bool(result["motion_active"])
            elif operation == "navigate":
                self._state["motion_active"] = self._state["goal_state"] not in _TERMINAL_GOALS
            elif operation in ("cancel", "stop") or self._state["goal_state"] in _TERMINAL_GOALS:
                self._state["motion_active"] = False
        if operation in ("cancel", "stop"):
            self._state["goal_state"] = result.get("goal_state", "canceled")
        if not self._state["running"]:
            self._state["ready"] = False
        if not self._state["connected"]:
            self._state["ready"] = False
        self._state["detail"] = result.get("detail", result.get("status", "操作完成"))

    def _execute(self, request):
        operation, args, kwargs, epoch = request
        with self._lock:
            if epoch != self._epoch or self._closed.is_set():
                self.events.put(dict(operation=operation, result={}, error=None, stale=True,
                                     target=self._target(operation, args)))
                return
        try:
            result = getattr(self.remote, operation)(*args, **kwargs) or {}
            with self._lock:
                stale = epoch != self._epoch or self._closed.is_set()
                if not stale:
                    self._apply_result_locked(operation, result)
            self.events.put(dict(operation=operation, result=result, error=None, stale=stale,
                                 target=self._target(operation, args), received_at=time.monotonic()))
            if not stale and operation not in ("cancel", "stop") and result.get("connected") is False:
                self.request_cancel()
        except Exception as exc:
            with self._lock:
                stale = epoch != self._epoch or self._closed.is_set()
                if not stale:
                    self._state["detail"] = str(exc)
                    if operation in ("connect", "status"):
                        self._state.update(connected=False, ready=False)
                    if operation == "start":
                        self._state.update(running=False, ready=False, owned=False)
                    self._drop_pending_locked("导航连接或操作失败，请重新确认后再操作")
            self.events.put(dict(operation=operation, result=getattr(exc, "partial", {}),
                                 error=str(exc), stale=stale, target=self._target(operation, args)))
            if operation in ("start", "navigate", "status"):
                # Revoke an uncertain action even when its response was lost.
                try:
                    self.remote.request_cancel()
                except Exception:
                    pass
                if operation == "start":
                    try:
                        self.remote.stop()
                    except Exception:
                        pass
                elif not stale:
                    self.request_cancel()

    def _run(self, request):
        while True:
            self._execute(request)
            with self._lock:
                if self._closed.is_set():
                    break
                if self._cancel_pending:
                    operation, self._cancel_pending = self._cancel_pending, None
                    request = (operation, (), {}, self._epoch)
                elif self._pending and self._state["connected"]:
                    request, self._pending = self._pending, None
                else:
                    self._drop_pending_locked("导航连接已断开，排队操作已取消")
                    self._state.update(busy=False, operation="")
                    self._active_operation = ""
                    return
                self._active_operation = request[0]
                self._state["operation"] = request[0]
        self._cleanup()

    def _poll(self):
        while not self._closed.wait(self._poll_interval):
            with self._lock:
                poll = (self._state["connected"] and self._state["running"]
                        and not self._state["busy"] and not self._cancel_pending)
            if poll:
                self.submit("status")

    def drain(self):
        result = []
        while True:
            try:
                result.append(self.events.get_nowait())
            except queue.Empty:
                return result

    def close_async(self):
        """Revoke immediately; late in-flight startup cleans up only its owner."""
        with self._lock:
            if self._closed.is_set():
                return
            self._closed.set()
            self._epoch += 1
            self._drop_pending_locked("导航会话已关闭")
            self._cancel_pending = None
            self._state.update(ready=False, detail="正在关闭导航")
            try:
                self.remote.request_cancel()
            except Exception:
                pass
            if not self._state["busy"]:
                self._state.update(busy=True, operation="stop")
                self._thread = threading.Thread(target=self._cleanup, name="navigation-cleanup", daemon=True)
                self._thread.start()

    def _cleanup(self):
        error = None
        try:
            self.remote.stop()
        except Exception as exc:
            error = str(exc)
        finally:
            try:
                self.remote.close()
            except Exception as exc:
                error = error or str(exc)
            with self._lock:
                self._state.update(connected=False, running=False, ready=False, owned=False,
                                   busy=False, operation="", motion_active=False, goal_state="stopped")
            self.events.put(dict(operation="close", result={}, error=error))

"""Small prestarted lane used when shutdown cannot create another thread."""

import queue
import threading


class SafetyPublishLane:
    """Run safety publications on a worker created before shutdown pressure."""

    def __init__(self, label, error_logger=None):
        self.label = str(label)
        self._error_logger = error_logger
        self._jobs = queue.Queue()
        self._state_lock = threading.Lock()
        self._available = False
        self._worker = threading.Thread(
            target=self._run,
            name='safety-publish-lane-{}'.format(self.label),
            daemon=True,
        )
        try:
            self._worker.start()
            self._available = True
        except Exception as exc:
            self._report("could not start", exc)

    @property
    def available(self):
        with self._state_lock:
            return self._available and self._worker.is_alive()

    def submit(self, callback):
        """Queue one callable without creating a thread at submission time."""
        with self._state_lock:
            if not self._available or not self._worker.is_alive():
                return False
            try:
                self._jobs.put_nowait(callback)
                return True
            except Exception as exc:
                self._report("could not accept a job", exc)
                return False

    def close(self):
        """Let an idle lane exit; a blocked safety publication stays detached."""
        with self._state_lock:
            if not self._available:
                return
            self._available = False
            try:
                self._jobs.put_nowait(None)
            except Exception as exc:
                self._report("could not close", exc)

    def _run(self):
        while True:
            callback = self._jobs.get()
            if callback is None:
                return
            try:
                callback()
            except Exception as exc:
                self._report("job failed", exc)

    def _report(self, action, error):
        if self._error_logger is None:
            return
        try:
            self._error_logger(self.label, action, error)
        except Exception:
            pass

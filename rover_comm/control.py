"""Local controller lease shared by the GUI and SDK (not a vehicle-side lock)."""
import socket
import threading


class ControlLock:
    def __init__(self, port=8892):
        self.port = port
        self._socket = None
        self._lock = threading.Lock()

    def acquire(self):
        with self._lock:
            if self._socket is not None:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                sock.bind(("127.0.0.1", self.port))
                sock.listen(1)
            except OSError:
                sock.close()
                raise RuntimeError("Control lock busy: another controller is running") from None
            self._socket = sock

    def release(self):
        with self._lock:
            if self._socket is not None:
                self._socket.close()
                self._socket = None

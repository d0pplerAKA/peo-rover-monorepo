"""Local controller lease shared by the GUI and SDK (not a vehicle-side lock).

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import socket
import threading


class ControlLock:
    """A local TCP listener that grants one controller exclusive access.

    Both the GUI and the peo SDK bind the same loopback port (8892); whoever
    binds first "owns" the vehicle connection and the other is rejected. This
    enforces the single-client rule on the PC side.

    Attributes:
        port: the loopback TCP port used for the lease (default 8892).
        _socket: the listening socket while the lease is held (None = free).
        _lock: protects _socket across threads.
    """

    def __init__(self, port=8892):
        self.port = port
        self._socket = None
        self._lock = threading.Lock()

    def acquire(self):
        """Bind the lease port, or raise RuntimeError if another controller
        already holds it. Acquiring twice is a no-op."""
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
        """Close the listening socket and free the lease (no-op when free)."""
        with self._lock:
            if self._socket is not None:
                self._socket.close()
                self._socket = None

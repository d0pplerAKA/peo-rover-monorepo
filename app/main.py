"""
PEO Rover 上位机入口
运行：python -m app.main  或  python app/main.py（两种都支持）
"""
import os
import sys

# 兼容直接运行本文件（VS Code ▶ 按钮 / python app/main.py）：
# 此时 __package__ 为 None，sys.path[0] 是 app/ 目录而非项目根，
# 把项目根手动插到 sys.path 开头，from app.gui import ... 才能解析。
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtCore import qInstallMessageHandler
from PySide6.QtWidgets import QApplication

from app.gui import MainWindow


def _qt_message_filter(mode, ctx, msg: str):
    """兜底静默 Qt 图像解码噪音：坏帧已在组装层（SOI/EOI 守卫）与
    GUI 层（字节预检 + isNull）拦截，此处滤掉漏网报错防 stderr 刷屏。"""
    if msg.startswith("qt.gui.imageio."):
        return
    sys.stderr.write(msg + "\n")


_STDERR_LOG = None   # 模块级持有文件对象：防止 GC 关闭导致 fd 2 悬空（EBADF）


def _redirect_c_stderr() -> None:
    """libjpeg 的解码警告直接 fprintf(stderr)（C 运行时句柄），Python 层
    替换 sys.stderr 拦不住。OS 级把 fd 2 重定向到项目根 stderr.log：
    不刷用户屏幕、错误可回溯；GUI 自身日志走 Qt 消息系统不受影响。

    文件对象必须模块级持有，否则 GC 关闭 → fd 2 悬空 → EBADF 刷屏。
    """
    global _STDERR_LOG
    log_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "stderr.log")
    try:
        _STDERR_LOG = open(log_path, "a", encoding="utf-8", buffering=1)
        os.dup2(_STDERR_LOG.fileno(), 2)
    except OSError:
        pass   # 重定向失败不阻止启动


def main():
    qInstallMessageHandler(_qt_message_filter)
    _redirect_c_stderr()
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


'''
import peo
peo.connect()
peo.delay(500)
peo.cam.set_resolution("QVGA")
peo.cam.on()

while True:
	peo.delay(100)
'''

"""
app.i18n — 轻量双语支持（中文 / English），运行时切换，无需 Qt Linguist

用法：
    from app.i18n import tr, set_language, current_language
    label.setText(tr("btn_connect"))       # 按当前语言取文本
    set_language("en"); window._retranslate()   # 切换语言后刷新 UI
"""
T = {
    "zh": {
        "app_title": "PEO Rover 上位机（PySide6）",
        "grp_conn": "连接（笔记本热点）",
        "ip": "IP:",
        "btn_connect": "连接",
        "btn_disconnect": "断开",
        "btn_video_on": "开启视频",
        "btn_video_off": "关闭视频",
        "status": "状态:",
        "conn_no": "● 未连接",
        "conn_yes": "● 已连接",
        "grp_sensor": "传感器仪表盘",
        "dist_front": "前距", "dist_left": "左距", "dist_right": "右距",
        "spd_col": "M{}速", "cur_col": "M{}流",
        "imu": "IMU",
        "cmd_ack": "命令回显",
        "btn_stop": "急停 STOP_ALL",
        "grp_console": "控制台",
        "video_wait": "等待视频流（TCP 8890）",
        "log_ready": "就绪。开启笔记本热点后输入 peo_rover.local 连接",
        "log_conn_ok": "已连接 ",
        "log_conn_fail": "连接失败: ",
        "log_disconn": "连接断开（ESP32 断连保护将自动急停）",
        "log_sent": "已发 ",
        "log_send_fail": "发送失败（未连接）",
        "log_ack": "ACK: ",
        "log_sc_stop": "⚠ 快捷键急停（空格）",
        "perf_cpu0": "CPU0",
        "perf_cpu1": "CPU1",
        "perf_ram": "RAM",
        "lock_manual": "手操",
        "lock_auto": "自主",
        "lock_tooltip": "控制权锁：手操（手柄控制）/ 自主（上位机控制）",
        "cam_res": "分辨率",
        "log_cam_res": "切换相机分辨率 → {}",
        "console_clear": "清空",
        "console_cleared": "控制台已清空。",
        "conn_connecting": "连接中…",
        "invalid": "无效",
        "st_ok": "OK",
        "st_err": "错误",
        "tab1_manual": '手动监控',
        "tab2_ide": '组员 IDE',
        "btn_open": '打开脚本',
        "btn_save": '保存',
        "btn_save_as": '另存为',
        "file_default": '默认代码 · 未保存',
        "dlg_save_as": '保存脚本',
        "log_save_ok": "已保存: ",
        "log_save_fail": "保存失败: ",
        "btn_run": '运行',
        "btn_stop_script": '停止',
        "ide_placeholder": '在此编写/打开组员 Python 脚本（import peo；示例见 examples/）',
        "shell_placeholder": '实时 shell：输入 peo.led(True) 回车即执行',
        "shell_locked": '[shell 已锁：脚本运行中，先停止脚本]',
        "ide_ready": '就绪：脚本将接管连接（GUI 已断开）',
        "ide_running": '▶ 脚本运行中…',
        "ide_done": '脚本结束',
        "ide_stopping": '⏹ 请求停止（脚本在下一个 delay 退出）…',
        "ide_empty": '编辑区为空，请先编写或打开脚本',
        "dlg_open": '打开组员脚本',
        "log_tab2_disconn": "已切到组员 IDE：GUI 连接自动断开，控制权交给脚本",
        "log_open_fail": "打开脚本失败: ",
    },
    "en": {
        "app_title": "PEO Rover Ground Station (PySide6)",
        "grp_conn": "Connection (laptop hotspot)",
        "ip": "IP:",
        "btn_connect": "Connect",
        "btn_disconnect": "Disconnect",
        "btn_video_on": "Start Video",
        "btn_video_off": "Stop Video",
        "status": "Status:",
        "conn_no": "● Disconnected",
        "conn_yes": "● Connected",
        "grp_sensor": "Sensor Dashboard",
        "dist_front": "Front", "dist_left": "Left", "dist_right": "Right",
        "spd_col": "M{} spd", "cur_col": "M{} cur",
        "imu": "IMU",
        "cmd_ack": "Cmd ACK",
        "btn_stop": "E-STOP STOP_ALL",
        "grp_console": "Console",
        "video_wait": "Waiting for video stream (TCP 8890)",
        "log_ready": "Ready. Turn on the laptop hotspot, then connect via peo_rover.local",
        "log_conn_ok": "Connected to ",
        "log_conn_fail": "Connection failed: ",
        "log_disconn": "Disconnected (ESP32 auto E-STOP on link loss)",
        "log_sent": "Sent ",
        "log_send_fail": "Send failed (not connected)",
        "log_ack": "ACK: ",
        "log_sc_stop": "⚠ E-STOP via shortcut (Space)",
        "perf_cpu0": "CPU0",
        "perf_cpu1": "CPU1",
        "perf_ram": "RAM",
        "lock_manual": "Manual",
        "lock_auto": "Auto",
        "lock_tooltip": "Control lock: Manual (controller) / Auto (ground station)",
        "cam_res": "Resolution",
        "log_cam_res": "Camera resolution → {}",
        "console_clear": "Clear",
        "console_cleared": "Console cleared.",
        "conn_connecting": "Connecting…",
        "invalid": "invalid",
        "st_ok": "OK",
        "st_err": "error",
        "tab1_manual": "Monitor",
        "app_brand": "PEO Rover · Ground Station",
        "tab2_ide": 'Team IDE',
        "btn_open": 'Open',
        "btn_save": 'Save',
        "btn_save_as": 'Save As',
        "file_default": 'Default code · unsaved',
        "dlg_save_as": 'Save Script',
        "log_save_ok": "Saved: ",
        "log_save_fail": "Save failed: ",
        "btn_run": 'Run',
        "btn_stop_script": 'Stop',
        "ide_placeholder": 'Write/open a team Python script here (import peo; see examples/)',
        "shell_placeholder": 'Live shell: type peo.led(True) and press Enter',
        "shell_locked": '[shell locked: script running — stop the script first]',
        "ide_ready": 'Ready: script will take over the connection (GUI disconnected)',
        "ide_running": '▶ Script running…',
        "ide_done": 'Script finished',
        "ide_stopping": '⏹ Stop requested (script exits at next delay)…',
        "ide_empty": 'Editor is empty — write or open a script first',
        "dlg_open": 'Open team script',
        "log_tab2_disconn": "Switched to Team IDE: GUI connection released for the script",
        "log_open_fail": "Open failed: ",
    },
}

LANG_NAMES = [("zh", "中文"), ("en", "English")]

_current = "en"    # 默认英文（GUI 主要受众 = 国际组员；用户可一键切中文）


def set_language(lang: str):
    """切换语言（"zh" / "en"），未知值忽略。"""
    global _current
    if lang in T:
        _current = lang


def current_language() -> str:
    return _current


def tr(key: str, *args) -> str:
    """取当前语言文本；key 缺失时原样返回（便于发现漏译）。"""
    s = T[_current].get(key, key)
    return s.format(*args) if args else s

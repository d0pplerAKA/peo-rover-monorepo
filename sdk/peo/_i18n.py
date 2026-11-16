"""peo._i18n — SDK 异常消息本地化（不硬依赖 GUI）。

SDK 保持独立可嵌入；当运行在 PEO Rover GUI 进程内时（app.i18n 已导入），
自动跟随 GUI 语言切换按钮（zh/en）；独立运行时默认英文（组员受众）。
任何 SDK 抛出的报错消息（NotConnected/LinkLost/ScriptStopped/CamOff/
参数校验错误/单实例锁占用）都必须经 tr() 生成，禁止硬编码单语。
"""


def tr(en: str, zh: str) -> str:
    """返回当前语言的消息。默认英文；GUI 内随语言按钮切换。"""
    try:
        import sys
        mod = sys.modules.get("app.i18n")
        if mod is not None and mod.current_language() == "zh":
            return zh
    except Exception:
        pass
    return en

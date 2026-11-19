"""peo._i18n - localizes SDK exception messages without a hard dependency
on the GUI.

The SDK stays independently embeddable. When it runs inside the PEO Rover GUI
process (app.i18n already imported), it automatically follows the GUI language
switch (zh/en); when run standalone it defaults to English (the team audience).

Every error message the SDK raises (NotConnected / LinkLost / ScriptStopped /
CamOff / argument validation / single-instance lock busy) must go through tr();
hard-coding a single language is forbidden.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""


def tr(en: str, zh: str) -> str:
    """Return the message in the current language; English by default, follows
    the GUI language button when running inside the GUI.

    Args:
        en: English message text.
        zh: Chinese message text.

    Returns:
        The message string for the active language.
    """
    try:
        import sys
        mod = sys.modules.get("app.i18n")
        if mod is not None and mod.current_language() == "zh":
            return zh
    except Exception:
        pass
    return en

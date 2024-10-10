import time
from typing import Any, Dict, NoReturn, Optional
import adsk.core
from .transforms import so3_to_euler

LOG_DEBUG = True
REFRESH_DELAY = 2.0

start_time: Optional[float] = None
last_refresh = 0.0
            
def start_log_timer() -> None:
    global start_time
    start_time = time.time()

def time_elapsed() -> float:
    assert start_time is not None
    return time.time() - start_time

def format_name(input: str):
    translation_table = str.maketrans({':':'_', '-':'_', '.':'_', ' ':'', '(':'{', ')':'}'})
    return input.translate(translation_table)

def rename_if_duplicate(input: str, in_dict: Dict[str, Any]) -> str:
    count = 0
    new_name = input
    while in_dict.get(new_name) is not None:
        new_name = f"{input}_{count}"
        count += 1
    return new_name

def convert_german(str_in):
    translation_table = str.maketrans({'ä': 'ae', 'ö': 'oe', 'ü': 'ue', 'Ä': 'Ae', 'Ö': 'Oe', 'Ü': 'Ue', 'ß': 'ss'})
    return str_in.translate(translation_table)

viewport: Optional[adsk.core.Viewport] = None

def log(msg: str, level: Optional[adsk.core.LogLevels] = None) -> None:
    if not LOG_DEBUG and msg.startswith("DEBUG"):
        return
    if level is None:
        level = adsk.core.LogLevels.InfoLogLevel
        if msg.startswith("WARN"):
            level = adsk.core.LogLevels.WarningLogLevel
        elif msg.startswith("FATAL") or msg.startswith("ERR"):
            level = adsk.core.LogLevels.ErrorLogLevel
    adsk.core.Application.log(msg, level)
    if viewport is not None:
        global last_refresh
        if time.time() >= last_refresh + REFRESH_DELAY:
            viewport.refresh()
            last_refresh = time.time()
    print(msg)

def fatal(msg: str) -> NoReturn:
    log_msg = "FATAL ERROR: " + msg
    if start_time is not None:
        log_msg += f"\n\tTime Elapsed: {time_elapsed():.1f}s."
    log(log_msg)
    raise RuntimeError(msg)

def mat_str(m: adsk.core.Matrix3D) -> str:
    return f"xyz={m.translation.asArray()} rpy={so3_to_euler(m)}"
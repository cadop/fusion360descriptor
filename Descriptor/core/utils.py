from typing import Any, Dict, NoReturn, Optional
import adsk.core

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

def log(msg: str, level: Optional[adsk.core.LogLevels] = None) -> None:
    if level is None:
        level = adsk.core.LogLevels.InfoLogLevel
        if msg.startswith("WARN"):
            level = adsk.core.LogLevels.WarningLogLevel
        elif msg.startswith("FATAL") or msg.startswith("ERR"):
            level = adsk.core.LogLevels.ErrorLogLevel
    adsk.core.Application.log(msg, level)
    print(msg)

def fatal(msg: str) -> NoReturn:
    log("FATAL ERROR: " + msg)
    raise RuntimeError(msg)
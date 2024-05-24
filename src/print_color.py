from enum import Enum

class BColors:
    """
    ANSI escape sequences for terminal text coloring.
    """
    TITLE = '\x1b[6;30;42m'
    GC = '\x1b[5;30;43m'
    DC = '\x1b[6;30;41m'
    NC = '\x1b[0;30;47m'
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    VOL = '\x1b[6;31;47m'

class Color(Enum):
    TITLE = 'title'
    BLUE = 'blue'
    CYAN = 'cyan'
    GREEN = 'green'
    UNDERLINE = 'underline'
    WARNING = 'warning'
    FAIL = 'fail'
    HEADER = 'header'
    BOLD = 'bold'
    VOL = 'vol'
    GC = 'gc'
    DC = 'dc'
    NC = 'nc'

COLOR_MAP = {
    Color.TITLE: BColors.TITLE,
    Color.BLUE: BColors.OKBLUE,
    Color.CYAN: BColors.OKCYAN,
    Color.GREEN: BColors.OKGREEN,
    Color.UNDERLINE: BColors.UNDERLINE,
    Color.WARNING: BColors.WARNING,
    Color.FAIL: BColors.FAIL,
    Color.HEADER: BColors.HEADER,
    Color.BOLD: BColors.BOLD,
    Color.VOL: BColors.VOL,
    Color.GC: BColors.GC,
    Color.DC: BColors.DC,
    Color.NC: BColors.NC
}

def print_colored(text: str, color: Color = Color.TITLE) -> None:
    """
    Prints the given text in the specified color.

    Parameters:
    - text (str): The text to print.
    - color (Color): The color enum member. Defaults to Color.TITLE.
    """
    color_code = COLOR_MAP.get(color, BColors.ENDC)
    print(f"{color_code}{text}{BColors.ENDC}")
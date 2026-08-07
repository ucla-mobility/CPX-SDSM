# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import sys
import logging

def setup_logging(name: str, level: str = "INFO"):
    """
    Sets up a simple, colored logger for ETX examples.
    Works on macOS, Linux, and Windows.
    """
    # Standard ANSI colors (30-series) work best on macOS
    CYAN = "\033[36m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    RED = "\033[31m"
    BOLD_RED = "\033[1;31m"
    RESET = "\033[0m"

    LEVEL_COLORS = {
        logging.DEBUG: CYAN,
        logging.INFO: GREEN,
        logging.WARNING: YELLOW,
        logging.ERROR: RED,
        logging.CRITICAL: BOLD_RED
    }

    class ColorFormatter(logging.Formatter):
        def format(self, record):
            color = LEVEL_COLORS.get(record.levelno, RESET)
            # Time | Level | Name:Func:Line - Message
            fmt = f"{color}%(asctime)s | %(levelname)-8s | %(name)s:%(funcName)s:%(lineno)d - %(message)s{RESET}"
            formatter = logging.Formatter(fmt, datefmt='%Y-%m-%d %H:%M:%S')
            return formatter.format(record)

    # Configure the logger
    logger = logging.getLogger(name)
    logger.setLevel(level.upper())

    # Clear existing handlers to avoid duplicates on re-run
    if logger.hasHandlers():
        logger.handlers.clear()

    # Create console handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(ColorFormatter())
    logger.addHandler(handler)

    # Mute background noise
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("paho").setLevel(logging.WARNING)

    # Don't send logs to the root logger
    logger.propagate = False

    return logger
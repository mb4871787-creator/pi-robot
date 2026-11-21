"""
config_loader.py
----------------
Loads and validates the YAML configuration file.
Provides a singleton-style accessor used across all nodes.
"""

import yaml
import os
import logging
from pathlib import Path

_config: dict = {}
_config_path: str = ""

logger = logging.getLogger(__name__)


def load_config(path: str = None) -> dict:
    """
    Load configuration from YAML file.
    Falls back to default path relative to project root.
    """
    global _config, _config_path

    if path is None:
        # Resolve relative to this file's location
        root = Path(__file__).parent.parent
        path = str(root / "config" / "config.yaml")

    _config_path = path

    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r") as f:
        _config = yaml.safe_load(f)

    logger.info(f"Configuration loaded from: {path}")
    return _config


def get_config() -> dict:
    """Return the loaded config dict. Loads default if not yet loaded."""
    global _config
    if not _config:
        load_config()
    return _config


def get(section: str, key: str = None, default=None):
    """
    Convenience accessor.
    Usage:
        get("serial", "port")        -> "/dev/ttyUSB0"
        get("serial")                -> {port: ..., baud_rate: ...}
        get("serial", "missing", 0)  -> 0
    """
    cfg = get_config()
    section_data = cfg.get(section, {})
    if key is None:
        return section_data
    return section_data.get(key, default)

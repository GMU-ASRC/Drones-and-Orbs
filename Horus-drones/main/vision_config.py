#!/usr/bin/env python3
import os

from cage_detector import CageParams

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "vision_config.yaml")


def load(path=DEFAULT_PATH):
    try:
        import yaml
        with open(path) as handle:
            document = yaml.safe_load(handle) or {}
    except Exception as e:
        return {}, f"built-in defaults ({type(e).__name__})"
    flat = {}
    for section, body in document.items():
        if section != "provenance" and isinstance(body, dict):
            flat.update(body)
    return flat, os.path.basename(path)


def build_params(overrides):
    params = CageParams()
    for key, value in overrides.items():
        if hasattr(params, key):
            setattr(params, key, type(getattr(params, key))(value))
    return params


def params_from(path=DEFAULT_PATH):
    overrides, source = load(path)
    return build_params(overrides), overrides, source

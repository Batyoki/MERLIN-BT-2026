import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from arm_bench.visualization import prepare_rviz_environment


def test_prepare_rviz_environment_removes_problematic_qt_plugin_path(monkeypatch):
    monkeypatch.setenv("QT_PLUGIN_PATH", "/tmp/cv2/qt/plugins")
    monkeypatch.setenv("QT_QPA_PLATFORM_PLUGIN_PATH", "/tmp/bad/plugins")

    env = prepare_rviz_environment()

    assert "QT_PLUGIN_PATH" not in env
    assert "QT_QPA_PLATFORM_PLUGIN_PATH" not in env
    assert env["QT_QPA_PLATFORM"] == "xcb"

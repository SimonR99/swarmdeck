import pytest

from adapters.onboard_mapping import (
    capture_provider_name,
    mapping_authority_mode,
    selected_capture_provider,
)


def test_mapping_authority_mode_is_validated(monkeypatch):
    monkeypatch.delenv("SWARMDECK_MAPPING_AUTHORITY", raising=False)
    assert mapping_authority_mode() == "central"
    monkeypatch.setenv("SWARMDECK_MAPPING_AUTHORITY", " OnBoard ")
    assert mapping_authority_mode() == "onboard"
    monkeypatch.setenv("SWARMDECK_MAPPING_AUTHORITY", "server-ish")
    with pytest.raises(ValueError, match="SWARMDECK_MAPPING_AUTHORITY"):
        mapping_authority_mode()


def test_capture_provider_selection_is_explicit_and_safe_by_default(monkeypatch):
    monkeypatch.delenv("SWARMDECK_CAPTURE_PROVIDER", raising=False)
    assert capture_provider_name() == "unknown"
    assert selected_capture_provider().spec.name == "unknown"
    monkeypatch.setenv("SWARMDECK_CAPTURE_PROVIDER", " FAST_LIVO2 ")
    assert capture_provider_name() == "fast_livo2"
    assert selected_capture_provider().spec.name == "fast_livo2"
    monkeypatch.setenv("SWARMDECK_CAPTURE_PROVIDER", "best")
    with pytest.raises(ValueError, match="SWARMDECK_CAPTURE_PROVIDER"):
        capture_provider_name()

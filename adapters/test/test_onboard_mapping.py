import pytest

from adapters.onboard_mapping import mapping_authority_mode
def test_mapping_authority_mode_is_validated(monkeypatch):
    monkeypatch.delenv("SWARMDECK_MAPPING_AUTHORITY", raising=False)
    assert mapping_authority_mode() == "central"
    monkeypatch.setenv("SWARMDECK_MAPPING_AUTHORITY", " OnBoard ")
    assert mapping_authority_mode() == "onboard"
    monkeypatch.setenv("SWARMDECK_MAPPING_AUTHORITY", "server-ish")
    with pytest.raises(ValueError, match="SWARMDECK_MAPPING_AUTHORITY"):
        mapping_authority_mode()

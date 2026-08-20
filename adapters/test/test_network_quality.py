from adapters.network_quality import parse_link_quality


WIRELESS = """Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
wlp2s0: 0000   49.  -61.  -256        0      0      0      2      0        0
wlan1:  0000   70.  -39.  -256        0      0      0      0      0        0
"""


def test_parse_link_quality_selects_named_interface():
    assert parse_link_quality(WIRELESS, "wlp2s0") == {
        "interface": "wlp2s0",
        "quality_pct": 70.0,
        "rssi_dbm": -61.0,
    }


def test_parse_link_quality_auto_selects_first_interface():
    assert parse_link_quality(WIRELESS, "auto")["interface"] == "wlp2s0"


def test_parse_link_quality_handles_unsigned_dbm_and_missing_interface():
    unsigned = WIRELESS.replace("-61.", "195.")
    assert parse_link_quality(unsigned, "wlp2s0")["rssi_dbm"] == -61.0
    assert parse_link_quality(WIRELESS, "missing0") is None

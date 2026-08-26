from adapters.network_quality import (
    parse_link_quality,
    ping_to_quality_pct,
    ping_to_rssi_dbm,
    probe_ping_latency,
    read_link_quality,
)


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


def test_ping_to_quality_pct_bounds_and_scaling():
    assert ping_to_quality_pct(10.0) == 100.0
    assert ping_to_quality_pct(5.0) == 100.0
    assert ping_to_quality_pct(200.0) == 0.0
    assert ping_to_quality_pct(300.0) == 0.0
    assert ping_to_quality_pct(105.0) == 50.0
    assert ping_to_quality_pct(-1.0) == 0.0
    assert ping_to_quality_pct(float("nan")) == 0.0


def test_ping_to_rssi_dbm_mapping():
    assert ping_to_rssi_dbm(100.0) == -50.0
    assert ping_to_rssi_dbm(50.0) == -70.0
    assert ping_to_rssi_dbm(0.0) == -90.0


def test_read_link_quality_uniform_ping():
    res = read_link_quality("auto", host="127.0.0.1")
    assert res is not None
    assert res["interface"] == "ping"
    assert "quality_pct" in res
    assert "rssi_dbm" in res
    assert "ping_ms" in res
    assert 0.0 <= res["quality_pct"] <= 100.0


def test_read_link_quality_explicit_target():
    res = read_link_quality("ping:127.0.0.1")
    assert res is not None
    assert res["interface"] == "ping"
    assert "quality_pct" in res
    assert "rssi_dbm" in res
    assert "ping_ms" in res



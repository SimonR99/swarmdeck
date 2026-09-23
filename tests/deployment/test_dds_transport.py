"""Simulation DDS transport and namespace-sharing contracts."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
COMPOSE = ROOT / "deploy/compose/docker-compose.yml"
PROFILE = "/app/deploy/dds/fastdds_large_data.xml"


def test_sim_and_peers_share_ipc_and_large_data_profile():
    services = yaml.safe_load(COMPOSE.read_text())["services"]
    sim = services["sim"]
    assert sim["ipc"] == "shareable"
    assert sim["shm_size"] == "2gb"
    for name in ("sim", "peer0", "peer1", "peer2", "peer3"):
        service = services[name]
        assert service["environment"]["FASTRTPS_DEFAULT_PROFILES_FILE"] == PROFILE
        assert "FASTDDS_BUILTIN_TRANSPORTS" not in service["environment"]
        assert "../dds:/app/deploy/dds:ro" in service["volumes"]
        if name != "sim":
            assert service["ipc"] == "service:sim"
            assert service["network_mode"] == "service:sim"
            assert service["depends_on"]["sim"]["condition"] == "service_started"
    overlay = ROOT / "deploy/compose/docker-compose.planning-test.yml"
    assert "FASTDDS_BUILTIN_TRANSPORTS" not in overlay.read_text()


def test_mgg_joins_the_same_sim_transport():
    service = yaml.safe_load(COMPOSE.read_text())["services"]["mgg"]
    assert service["ipc"] == "service:sim"
    assert service["network_mode"] == "service:sim"
    assert service["environment"]["FASTRTPS_DEFAULT_PROFILES_FILE"] == PROFILE
    assert "FASTDDS_BUILTIN_TRANSPORTS" not in service["environment"]
    assert "../dds:/app/deploy/dds:ro" in service["volumes"]
    assert service["depends_on"]["sim"]["condition"] == "service_started"


def test_large_data_profile_keeps_udp_for_remote_participants():
    ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    root = ET.parse(ROOT / "deploy/dds/fastdds_large_data.xml").getroot()
    descriptors = root.findall(".//dds:transport_descriptor", ns)
    transports = {item.find("dds:transport_id", ns).text: item for item in descriptors}
    shm = transports["swarmdeck_shm"]
    assert shm.find("dds:type", ns).text == "SHM"
    assert int(shm.find("dds:segment_size", ns).text) >= 16777216
    assert transports["swarmdeck_udp"].find("dds:type", ns).text == "UDPv4"
    participant = root.find(".//dds:participant", ns)
    assert participant.attrib["is_default_profile"] == "true"
    assert participant.find(".//dds:useBuiltinTransports", ns).text == "false"
    assert {
        item.text
        for item in participant.findall(".//dds:userTransports/dds:transport_id", ns)
    } == set(transports)
    udp = ET.parse(ROOT / "deploy/dds/fastdds_udp_only.xml").getroot()
    assert [item.text for item in udp.findall(".//dds:type", ns)] == ["UDPv4"]

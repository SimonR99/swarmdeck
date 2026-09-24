"""The server image copies every repository module its tests and scripts import.

Host pytest finds modules through pytest.ini's pythonpath, so a module moved
out of a copied directory (as tools/ and swarmdeck_peer were) still passes
here while `make docker-test` and the image's scripts fail to import it.
"""

import ast
import configparser
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
DOCKERFILE = REPO / "deploy/docker/Dockerfile.server"
IMPORTERS = ("server/tests", "scripts")
# `deploy` is imported only by checkout-structure tests (test_sim_up,
# test_simulation_reset), which read compose and launcher files the
# production server image deliberately does not ship.
NOT_IN_IMAGE = {"deploy"}


def _roots() -> list[Path]:
    config = configparser.ConfigParser()
    config.read(REPO / "pytest.ini")
    return [REPO / entry for entry in config["pytest"]["pythonpath"].split()]


def _module_file(dotted: str, roots: list[Path]) -> Path | None:
    parts = dotted.split(".")
    for root in roots:
        base = root.joinpath(*parts)
        for candidate in (base.with_suffix(".py"), base / "__init__.py"):
            if candidate.is_file():
                return candidate
    return None


def _imported_modules() -> set[str]:
    names: set[str] = set()
    for directory in IMPORTERS:
        for path in (REPO / directory).rglob("*.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Import):
                    names.update(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.level == 0:
                    names.add(node.module)
                    names.update(f"{node.module}.{alias.name}" for alias in node.names)
    return names


def _copied_sources() -> list[Path]:
    text = DOCKERFILE.read_text().replace("\\\n", " ")
    sources = []
    for line in text.splitlines():
        words = line.split()
        if words[:1] != ["COPY"]:
            continue
        paths = [word for word in words[1:] if not word.startswith("--")]
        sources.extend(REPO / source for source in paths[:-1])
    return sources


def test_every_repository_module_imported_by_tests_and_scripts_is_copied():
    if not DOCKERFILE.is_file():
        pytest.skip("checkout-only test: the Dockerfile is not in the image")
    roots, sources = _roots(), _copied_sources()
    missing = set()
    for dotted in _imported_modules():
        if dotted.split(".")[0] in NOT_IN_IMAGE:
            continue
        module = _module_file(dotted, roots)
        if module is None:
            continue  # third-party, or a name imported from a module
        if not any(module == source or source in module.parents for source in sources):
            missing.add(str(module.relative_to(REPO)))
    assert not missing, f"Dockerfile.server does not copy {sorted(missing)}"

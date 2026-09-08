"""The normal simulation launcher must actually start its exploration sidecar."""
import json
import os
from pathlib import Path
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('render', ['software', 'gpu', 'dri'])
def test_sim_up_starts_mgg_with_matching_scenario(tmp_path, render):
    docker = tmp_path / 'docker'
    docker.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['CAPTURE'], 'w') as stream:
    json.dump({'args': sys.argv[1:], 'config': os.environ.get('SWARMDECK_CONFIG')}, stream)
''')
    docker.chmod(0o755)
    capture = tmp_path / 'call.json'
    env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}', CAPTURE=str(capture))
    subprocess.run([str(REPO / 'scripts/sim-up'), '--render', render,
                    '--scenario', 'bistro', '--drift', '--no-build'],
                   env=env, check=True, capture_output=True)
    call = json.loads(capture.read_text())
    assert str(REPO / 'deploy/compose/docker-compose.mgg.yml') in call['args']
    assert call['args'][-1] == 'mgg'
    assert call['config'] == '/app/configs/4robot_bistro.yaml'

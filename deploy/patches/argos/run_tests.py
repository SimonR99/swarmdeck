"""Run the native regressions against an existing ARGoS Jolt build."""

import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent


def compile_test(name: str, build: Path, output: Path) -> Path:
    """Compile with the library's ABI flags, including profiling and SIMD."""
    jolt = build / "_deps/joltphysics-build"
    flags = {}
    for line in (jolt / "CMakeFiles/Jolt.dir/flags.make").read_text().splitlines():
        if " = " in line:
            key, value = line.split(" = ", 1)
            flags[key] = shlex.split(value)
    options = flags["CXX_DEFINES"] + [
        flag for flag in flags["CXX_FLAGS"] if flag != "-fno-exceptions"
    ]
    binary = output / name
    subprocess.run(
        [
            "c++",
            *options,
            "-O2",
            f"-I{build / '_deps/joltphysics-src'}",
            str(HERE / f"{name}.cpp"),
            str(jolt / "libJolt.a"),
            "-pthread",
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("argos_build", type=Path)
    parser.add_argument(
        "--bistro", type=Path, help="Optional local bistro_exterior.glb"
    )
    args = parser.parse_args()
    build = args.argos_build.resolve()
    with tempfile.TemporaryDirectory(prefix="swarmdeck-jolt-tests-") as work:
        output = Path(work)
        steps = compile_test("test_steps", build, output)
        subprocess.run([str(steps)], check=True)
        if args.bistro:
            # NumPy and the scenario loader are only needed for the mesh fixture.
            from extract_bistro_road import extract_road

            road = output / "road.bin"
            extract_road(args.bistro.resolve(), road)
            contacts = compile_test("test_bistro_contacts", build, output)
            command = [str(contacts), str(road)]
            if subprocess.run([*command, "legacy"]).returncode != 1:
                raise RuntimeError("Bistro fixture did not reproduce the legacy snag")
            subprocess.run(command, check=True)
            subprocess.run([*command, "wall"], check=True)


if __name__ == "__main__":
    main()

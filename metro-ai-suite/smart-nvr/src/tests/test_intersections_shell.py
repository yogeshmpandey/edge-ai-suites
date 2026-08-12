# SPDX-FileCopyrightText: (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Tests for the shell side of the intersections configuration.

``scripts/parse-intersections.sh`` is the dependency-free YAML reader used by
``setup.sh`` to build the Frigate camera blocks, so it is exercised here through
subprocess calls together with the setup helpers that consume it.
"""

import pathlib
import re
import shutil
import subprocess

import pytest
import yaml

ANSI = re.compile(r"\x1b\[[0-9;]*m")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
PARSER = REPO_ROOT / "scripts" / "parse-intersections.sh"
SETUP = REPO_ROOT / "setup.sh"

FULL_CONFIG = """
intersections:
  - id: si1
    name: Main Street and 1st Ave     # trailing comment is ignored
    ip: 10.0.0.11
    cameras:
      - name: si1-camera1
      - name: si1-camera2
        url: rtsp://10.0.0.21:8555/camera2
      - name: si1-camera3
      - name: si1-camera4
        url: rtsp://10.0.0.11:8554/custom-path
    enabled: true

  - name: "Broadway and 5th"
    ip: 10.0.0.12
    cameras:
      - name: si2-camera1
      - name: si2-camera2
      - name: si2-camera3
      - name: si2-camera4

  - id: si3
    name: Disabled intersection
    ip: 10.0.0.13
    enabled: false
"""


def write_config(tmp_path, content, name="intersections.yaml"):
    path = tmp_path / name
    path.write_text(content)
    return path


def run_parser(config_path, port="8554"):
    return subprocess.run(
        ["bash", str(PARSER), str(config_path), port],
        capture_output=True,
        text=True,
    )


def records(result, kind):
    return [line.split("|") for line in result.stdout.splitlines() if line.startswith(f"{kind}|")]


class TestParser:
    def test_parses_intersections_and_cameras(self, tmp_path):
        result = run_parser(write_config(tmp_path, FULL_CONFIG))
        assert result.returncode == 0

        assert [(r[1], r[2], r[3], r[4], r[5]) for r in records(result, "I")] == [
            ("si1", "Main Street and 1st Ave", "10.0.0.11", "1883", "true"),
            ("si2", "Broadway and 5th", "10.0.0.12", "1883", "true"),
            ("si3", "Disabled intersection", "10.0.0.13", "1883", "false"),
        ]

    def test_camera_defaults_and_overrides(self, tmp_path):
        result = run_parser(write_config(tmp_path, FULL_CONFIG))
        si1 = [r for r in records(result, "C") if r[1] == "si1"]

        assert [(r[2], r[3]) for r in si1] == [
            ("si1-camera1", "rtsp://10.0.0.11:8554/camera1"),
            ("si1-camera2", "rtsp://10.0.0.21:8555/camera2"),
            ("si1-camera3", "rtsp://10.0.0.11:8554/camera3"),
            ("si1-camera4", "rtsp://10.0.0.11:8554/custom-path"),
        ]

    def test_id_derived_from_camera_names(self, tmp_path):
        result = run_parser(write_config(tmp_path, FULL_CONFIG))
        assert [r[2] for r in records(result, "C") if r[1] == "si2"] == [
            f"si2-camera{n}" for n in range(1, 5)
        ]

    def test_intersection_without_cameras_gets_four_defaults(self, tmp_path):
        result = run_parser(write_config(tmp_path, FULL_CONFIG))
        si3 = [r for r in records(result, "C") if r[1] == "si3"]
        assert [(r[2], r[3]) for r in si3] == [
            (f"si3-camera{n}", f"rtsp://10.0.0.13:8554/camera{n}") for n in range(1, 5)
        ]

    def test_default_rtsp_port_is_applied(self, tmp_path):
        config = "intersections:\n  - id: si1\n    ip: 10.0.0.11\n"
        result = run_parser(write_config(tmp_path, config), port="9999")
        assert all(r[3].startswith("rtsp://10.0.0.11:9999/") for r in records(result, "C"))

    def test_keys_after_camera_block_belong_to_the_intersection(self, tmp_path):
        config = (
            "intersections:\n"
            "  - id: si1\n"
            "    ip: 10.1.1.1\n"
            "    cameras:\n"
            "      - name: si1-camera1\n"
            "    mqtt_port: 8883\n"
            "    enabled: false\n"
        )
        result = run_parser(write_config(tmp_path, config))
        assert records(result, "I")[0][4:6] == ["8883", "false"]

    @pytest.mark.parametrize(
        "content,expected_rc",
        [
            ("intersections: []\n", 2),
            ("# only comments\n", 2),
            ("intersections:\n  - id: si1\n    name: no ip\n", 3),
        ],
    )
    def test_exit_codes(self, tmp_path, content, expected_rc):
        assert run_parser(write_config(tmp_path, content)).returncode == expected_rc

    def test_missing_file_exits_with_one(self, tmp_path):
        assert run_parser(tmp_path / "absent.yaml").returncode == 1

    def test_missing_ip_reports_the_intersection(self, tmp_path):
        result = run_parser(write_config(tmp_path, "intersections:\n  - id: si7\n    name: broken\n"))
        assert "si7" in result.stderr and "missing ip" in result.stderr

    def test_repo_template_has_no_entries(self):
        template = REPO_ROOT / "resources" / "broker-config" / "intersections.yaml"
        assert template.exists()
        assert run_parser(template).returncode == 2

    @pytest.mark.parametrize(
        "config",
        [
            # PyYAML style: list items aligned with the key that owns them
            "intersections:\n"
            "- id: si1\n"
            "  name: Main\n"
            "  ip: 10.0.0.11\n"
            "  cameras:\n"
            "  - name: si1-camera1\n"
            "    url: rtsp://10.0.0.11:8554/camera1\n"
            "- id: si2\n"
            "  name: Broadway\n"
            "  ip: 10.0.0.12\n",
            # Indented list items
            "intersections:\n"
            "  - id: si1\n"
            "    name: Main\n"
            "    ip: 10.0.0.11\n"
            "    cameras:\n"
            "      - name: si1-camera1\n"
            "        url: rtsp://10.0.0.11:8554/camera1\n"
            "  - id: si2\n"
            "    name: Broadway\n"
            "    ip: 10.0.0.12\n",
        ],
        ids=["pyyaml-style", "indented-style"],
    )
    def test_supports_both_yaml_indentation_styles(self, tmp_path, config):
        result = run_parser(write_config(tmp_path, config))
        assert result.returncode == 0, result.stderr
        assert [(r[1], r[3]) for r in records(result, "I")] == [
            ("si1", "10.0.0.11"),
            ("si2", "10.0.0.12"),
        ]
        assert [r[2] for r in records(result, "C") if r[1] == "si1"] == ["si1-camera1"]

    def test_explicit_null_values_fall_back_to_defaults(self, tmp_path):
        config = (
            "intersections:\n"
            "  - id: si1\n"
            "    ip: 10.0.0.11\n"
            "    cameras:\n"
            "      - name: si1-camera1\n"
            "        url: null\n"
        )
        result = run_parser(write_config(tmp_path, config))
        assert records(result, "C")[0][3] == "rtsp://10.0.0.11:8554/camera1"

    def test_reads_a_file_written_by_the_backend(self, tmp_path):
        """The backend rewrites this file via the /brokers/ API — both must agree."""
        from service import intersection_config

        source = write_config(tmp_path, FULL_CONFIG, name="source.yaml")
        written = tmp_path / "written.yaml"
        intersection_config.save_intersections(
            str(written), intersection_config.load_intersections(str(source))
        )

        from_source = run_parser(source)
        from_written = run_parser(written)
        assert from_written.returncode == 0, from_written.stderr

        assert sorted(r[1:] for r in records(from_source, "I")) == sorted(
            r[1:] for r in records(from_written, "I")
        )
        assert sorted(r[1:] for r in records(from_source, "C")) == sorted(
            r[1:] for r in records(from_written, "C")
        )


def run_setup_snippet(tmp_path, snippet, env=None, stdin=""):
    """Source setup.sh in a subshell and run helper functions against tmp files."""
    script = f"source {SETUP} help >/dev/null 2>&1\n{snippet}\n"
    environment = {
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
        "HOME": str(tmp_path),
        "FRIGATE_CONFIG_FILE": str(tmp_path / "config.yml"),
        **(env or {}),
    }
    result = subprocess.run(
        ["bash", "-c", script],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        input=stdin,
        env=environment,
    )
    result.stdout = ANSI.sub("", result.stdout)
    result.stderr = ANSI.sub("", result.stderr)
    return result


class TestSetupIntegration:
    def test_frigate_config_is_generated_from_intersections(self, tmp_path):
        config = write_config(tmp_path, FULL_CONFIG)
        result = run_setup_snippet(
            tmp_path,
            "resolve_intersections false || exit 1\ngenerate_scenescape_config || exit 1",
            env={
                "INTERSECTIONS_CONFIG_PATH": str(config),
                "INTERSECTIONS_AUTO_CONFIRM": "true",
                "FRIGATE_CONFIG_FILE": str(tmp_path / "config.yml"),
            },
        )
        assert result.returncode == 0, result.stderr

        generated = yaml.safe_load((tmp_path / "config.yml").read_text())
        cameras = generated["cameras"]

        # disabled intersections are left out of the Frigate config
        assert sorted(cameras) == [f"si1-camera{n}" for n in range(1, 5)] + [
            f"si2-camera{n}" for n in range(1, 5)
        ]
        paths = [cameras[name]["ffmpeg"]["inputs"][0]["path"] for name in sorted(cameras)[:4]]
        assert paths == [
            "rtsp://10.0.0.11:8554/camera1",
            "rtsp://10.0.0.21:8555/camera2",
            "rtsp://10.0.0.11:8554/camera3",
            "rtsp://10.0.0.11:8554/custom-path",
        ]

    def test_summary_reports_the_number_of_intersections(self, tmp_path):
        config = write_config(tmp_path, FULL_CONFIG)
        result = run_setup_snippet(
            tmp_path,
            "resolve_intersections false",
            env={
                "INTERSECTIONS_CONFIG_PATH": str(config),
                "INTERSECTIONS_AUTO_CONFIRM": "true",
            },
        )
        assert "Found 3 preconfigured intersection(s)" in result.stdout
        assert "si1 - Main Street and 1st Ave @ 10.0.0.11 (4 cameras)" in result.stdout

    def test_missing_config_fails_without_seeding(self, tmp_path):
        result = run_setup_snippet(
            tmp_path,
            "resolve_intersections false",
            env={"INTERSECTIONS_CONFIG_PATH": str(tmp_path / "absent.yaml")},
        )
        assert result.returncode != 0
        assert "No intersections configured" in result.stdout

    def test_invalid_config_reports_the_error(self, tmp_path):
        config = write_config(tmp_path, "intersections:\n  - id: si7\n    name: broken\n")
        result = run_setup_snippet(
            tmp_path,
            "resolve_intersections false",
            env={"INTERSECTIONS_CONFIG_PATH": str(config)},
        )
        assert result.returncode != 0
        assert "si7" in result.stdout and "missing ip" in result.stdout

    def test_single_node_seeds_a_local_intersection(self, tmp_path):
        config = tmp_path / "seeded.yaml"
        result = run_setup_snippet(
            tmp_path,
            "resolve_intersections true",
            env={
                "INTERSECTIONS_CONFIG_PATH": str(config),
                "INTERSECTIONS_AUTO_CONFIRM": "true",
            },
        )
        assert result.returncode == 0, result.stderr

        seeded = yaml.safe_load(config.read_text())["intersections"]
        assert seeded[0]["id"] == "si1"
        assert [c["name"] for c in seeded[0]["cameras"]] == [f"si1-camera{n}" for n in range(1, 5)]
        assert len({c["url"] for c in seeded[0]["cameras"]}) == 4

    def test_declining_the_prompt_aborts_with_guidance(self, tmp_path):
        config = write_config(tmp_path, FULL_CONFIG)
        script = tmp_path / "answer_no.sh"
        script.write_text(
            f"source {SETUP} help >/dev/null 2>&1\nresolve_intersections false\necho RC=$?\n"
        )
        result = subprocess.run(
            ["script", "-qec", f"bash {script}", "/dev/null"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            input="N\n",
            env={
                "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
                "HOME": str(tmp_path),
                "INTERSECTIONS_CONFIG_PATH": str(config),
                "FRIGATE_CONFIG_FILE": str(tmp_path / "config.yml"),
            },
        )
        output = ANSI.sub("", result.stdout)
        assert "Would you like to use them? (Y/N)" in output
        assert "RC=1" in output
        assert "Update the file and re-run" in output


@pytest.mark.skipif(shutil.which("shellcheck") is None, reason="shellcheck not installed")
def test_shell_scripts_pass_shellcheck():
    result = subprocess.run(
        ["shellcheck", "-S", "warning", str(SETUP), str(PARSER)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout

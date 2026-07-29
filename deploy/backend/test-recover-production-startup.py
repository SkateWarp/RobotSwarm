#!/usr/bin/env python3

import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
RECOVERY_SCRIPT = HERE / "recover-production-startup.sh"
SERVICE_UNIT = HERE / "robotswarm-production-startup-recovery.service"


FAKE_COMMAND = r"""#!/usr/bin/env python3
import json
import os
from pathlib import Path
import re
import sys

state_path = Path(os.environ["ROBOTSWARM_FAKE_STATE"])
state = json.loads(state_path.read_text(encoding="utf-8"))
command = Path(sys.argv[0]).name
arguments = sys.argv[1:]

def save():
    state_path.write_text(json.dumps(state), encoding="utf-8")

def record(*parts):
    state.setdefault("calls", []).append(list(parts))
    save()

if command == "ip":
    state["ip_checks"] = state.get("ip_checks", 0) + 1
    record("ip", *arguments)
    available_after = state.get("ip_available_after", 0)
    if state["ip_checks"] > available_after:
        for index, address in enumerate(state.get("ips", []), start=2):
            print(
                f"{index}: ens18    inet {address}/24 brd 10.0.0.255 "
                "scope global ens18"
            )
    raise SystemExit(0)

if command != "docker":
    raise SystemExit(f"unexpected fake command: {command}")

if arguments[0] == "info":
    record("docker", "info")
    if not state.get("docker_available", True):
        raise SystemExit(1)
    print("28.0.0")
    raise SystemExit(0)

if not state.get("docker_available", True):
    record("docker", *arguments)
    raise SystemExit(1)

if arguments[0] == "inspect":
    template = arguments[2]
    name = arguments[3]
    container = state["containers"].get(name)
    if container is None:
        raise SystemExit(1)

    record("docker", "inspect", name)
    if template in container.get("failing_inspect_templates", []):
        raise SystemExit(1)
    if template == "{{.Id}}":
        print(container.get("id", f"fake-{name}"))
    elif "com.docker.compose.project" in template:
        print(f"{container['project']}|{container['service']}")
    elif template == "{{.State.Status}}":
        print(container["status"])
    elif ".Config.Healthcheck" in template:
        print("yes" if container.get("healthcheck", True) else "no")
    elif ".State.Health" in template:
        print(container.get("health", "unknown"))
    elif ".HostConfig.PortBindings" in template:
        match = re.search(r'PortBindings "([^"]+)"', template)
        if not match:
            raise SystemExit("port was not present in inspect template")
        for address in container.get("bindings", {}).get(match.group(1), []):
            print(address)
    else:
        raise SystemExit(f"unexpected inspect template: {template}")
    raise SystemExit(0)

if arguments[:2] == ["container", "ls"]:
    record("docker", *arguments)
    if state.get("container_list_available", True) is False:
        raise SystemExit(1)
    name_filter = arguments[arguments.index("--filter") + 1]
    match = re.fullmatch(r"name=\^/([^$]+)\$", name_filter)
    if not match:
        raise SystemExit(f"unexpected container name filter: {name_filter}")
    name = match.group(1)
    if name in state["containers"]:
        print(name)
    raise SystemExit(0)

if arguments[0] == "start":
    name = arguments[1]
    container = state["containers"][name]
    if container["status"] not in ("created", "exited"):
        raise SystemExit(1)
    container["status"] = "running"
    record("docker", "start", name)
    print(name)
    raise SystemExit(0)

raise SystemExit(f"unexpected docker arguments: {arguments}")
"""


def container(service, status="exited", *, healthcheck=True, health="healthy"):
    bindings = {}
    if service == "media":
        bindings["8554/tcp"] = ["10.0.0.126"]
    elif service == "backend":
        bindings["44336/tcp"] = ["10.0.0.126"]

    return {
        "project": "robotswarm",
        "service": service,
        "status": status,
        "healthcheck": healthcheck,
        "health": health,
        "bindings": bindings,
    }


def normal_state(status="exited"):
    return {
        "containers": {
            "db_prod": container("db", status),
            "media_prod": container("media", status),
            "backend_prod": container("backend", status),
        },
        "ips": ["10.0.0.126"],
        "docker_available": True,
        "calls": [],
    }


class RecoveryTestCase(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.state_path = self.root / "state.json"

        fake = self.bin / "fake-command"
        fake.write_text(FAKE_COMMAND, encoding="utf-8")
        fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
        (self.bin / "docker").symlink_to(fake)
        (self.bin / "ip").symlink_to(fake)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def run_recovery(self, state, **environment):
        self.state_path.write_text(json.dumps(state), encoding="utf-8")
        variables = os.environ.copy()
        variables.update(
            {
                "PATH": f"{self.bin}:{variables['PATH']}",
                "ROBOTSWARM_FAKE_STATE": str(self.state_path),
                "ROBOTSWARM_NETWORK_WAIT_SECONDS": "0",
                "ROBOTSWARM_HEALTH_WAIT_SECONDS": "0",
                "ROBOTSWARM_POLL_SECONDS": "0",
            }
        )
        variables.update(environment)

        result = subprocess.run(
            ["bash", str(RECOVERY_SCRIPT)],
            cwd=HERE,
            env=variables,
            text=True,
            capture_output=True,
            check=False,
            timeout=10,
        )
        final_state = json.loads(self.state_path.read_text(encoding="utf-8"))
        return result, final_state

    @staticmethod
    def start_calls(state):
        return [
            call[2]
            for call in state["calls"]
            if call[:2] == ["docker", "start"]
        ]

    def test_starts_stopped_containers_in_the_required_order(self):
        result, state = self.run_recovery(normal_state())

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            self.start_calls(state),
            ["db_prod", "media_prod", "backend_prod"],
        )
        self.assertIn("production containers are ready", result.stdout)

    def test_does_not_restart_containers_that_are_already_running(self):
        result, state = self.run_recovery(normal_state(status="running"))

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.start_calls(state), [])

    def test_waits_for_the_address_derived_from_the_bindings(self):
        initial = normal_state()
        initial["ip_available_after"] = 2
        result, state = self.run_recovery(
            initial,
            ROBOTSWARM_NETWORK_WAIT_SECONDS="2",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertGreaterEqual(state["ip_checks"], 3)
        self.assertEqual(
            self.start_calls(state),
            ["db_prod", "media_prod", "backend_prod"],
        )

    def test_fails_closed_when_a_compose_label_does_not_match(self):
        initial = normal_state()
        initial["containers"]["media_prod"]["project"] = "other-project"
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 78)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("does not belong", result.stderr)

    def test_reports_a_temporary_failure_when_docker_is_not_ready(self):
        initial = normal_state()
        initial["docker_available"] = False
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("Docker is not ready", result.stderr)

    def test_treats_an_ambiguous_identity_inspection_as_temporary(self):
        initial = normal_state()
        initial["containers"]["db_prod"]["failing_inspect_templates"] = [
            "{{.Id}}"
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn(
            ["docker", "container", "ls", "--all", "--filter",
             "name=^/db_prod$", "--format", "{{.Names}}"],
            state["calls"],
        )
        self.assertIn(
            "lists db_prod but could not inspect its identity",
            result.stderr,
        )

    def test_treats_a_failed_compose_label_inspection_as_temporary(self):
        initial = normal_state()
        initial["containers"]["db_prod"]["failing_inspect_templates"] = [
            '{{index .Config.Labels "com.docker.compose.project"}}'
            '|{{index .Config.Labels "com.docker.compose.service"}}'
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("could not inspect the Compose identity", result.stderr)

    def test_treats_a_failed_binding_inspection_as_temporary(self):
        initial = normal_state()
        initial["containers"]["media_prod"]["failing_inspect_templates"] = [
            '{{with index .HostConfig.PortBindings "8554/tcp"}}'
            '{{range .}}{{println .HostIp}}{{end}}{{end}}'
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("could not inspect the 8554/tcp binding", result.stderr)

    def test_reports_a_missing_required_container_only_after_confirmation(self):
        initial = normal_state()
        del initial["containers"]["db_prod"]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 78)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("required container db_prod does not exist", result.stderr)

    def test_treats_a_failed_existence_confirmation_as_temporary(self):
        initial = normal_state()
        del initial["containers"]["db_prod"]
        initial["container_list_available"] = False
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn(
            "could not confirm whether db_prod exists",
            result.stderr,
        )

    def test_reports_a_temporary_failure_when_the_address_is_absent(self):
        initial = normal_state()
        initial["ips"] = []
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("was not assigned", result.stderr)

    def test_accepts_a_running_container_without_a_healthcheck(self):
        initial = normal_state()
        initial["containers"]["media_prod"]["healthcheck"] = False
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            self.start_calls(state),
            ["db_prod", "media_prod", "backend_prod"],
        )

    def test_does_not_restart_an_unhealthy_running_container(self):
        initial = normal_state(status="running")
        initial["containers"]["media_prod"]["health"] = "unhealthy"
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("did not become healthy", result.stderr)

    def test_fails_closed_when_healthcheck_inspection_fails(self):
        initial = normal_state()
        initial["containers"]["media_prod"]["failing_inspect_templates"] = [
            "{{if .Config.Healthcheck}}yes{{else}}no{{end}}"
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), ["db_prod", "media_prod"])
        self.assertIn("could not inspect the healthcheck", result.stderr.lower())
        self.assertNotIn("production containers are ready", result.stdout)

    def test_treats_a_failed_state_inspection_as_temporary(self):
        initial = normal_state()
        initial["containers"]["db_prod"]["failing_inspect_templates"] = [
            "{{.State.Status}}"
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("could not inspect the state", result.stderr)

    def test_treats_a_failed_health_inspection_as_temporary(self):
        initial = normal_state(status="running")
        initial["containers"]["db_prod"]["failing_inspect_templates"] = [
            "{{if .State.Health}}{{.State.Health.Status}}"
            "{{else}}unknown{{end}}"
        ]
        result, state = self.run_recovery(initial)

        self.assertEqual(result.returncode, 75)
        self.assertEqual(self.start_calls(state), [])
        self.assertIn("could not inspect the health", result.stderr)

    def test_service_is_scoped_to_startup_recovery(self):
        unit = SERVICE_UNIT.read_text(encoding="utf-8")
        script = RECOVERY_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("Requires=docker.service", unit)
        self.assertIn("After=docker.service", unit)
        self.assertIn("Type=oneshot", unit)
        self.assertIn("RemainAfterExit=yes", unit)
        self.assertIn("Restart=on-failure", unit)
        self.assertIn("RestartPreventExitStatus=78", unit)
        self.assertIn(
            "ExecStart=/usr/local/sbin/robotswarm-recover-production-startup",
            unit,
        )
        self.assertNotIn("docker compose", unit)
        self.assertNotIn("EnvironmentFile", unit)
        self.assertNotIn("docker compose", script)
        self.assertNotIn(".Config.Env", script)
        self.assertNotIn("EnvironmentFile", script)


if __name__ == "__main__":
    unittest.main(verbosity=2)

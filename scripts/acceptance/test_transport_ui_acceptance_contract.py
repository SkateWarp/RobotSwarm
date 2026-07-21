#!/usr/bin/env python3
"""Offline contract tests for the visible CollaborativeTransport UI smoke."""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import importlib.util
import inspect
import json
import re
import sys
import tempfile
import threading
import types
import unittest
import uuid
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("robotswarm-transport-ui-e2e.py")
REPOSITORY = SCRIPT.parents[2]


def load_harness():
    if "websocket" not in sys.modules:
        websocket = types.ModuleType("websocket")
        websocket.create_connection = None
        sys.modules["websocket"] = websocket
    if "PIL" not in sys.modules:
        pil = types.ModuleType("PIL")
        pil.Image = types.SimpleNamespace(Image=object)
        sys.modules["PIL"] = pil
    specification = importlib.util.spec_from_file_location(
        "robotswarm_transport_ui_e2e",
        SCRIPT,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("could not load the transport UI smoke")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


HARNESS = load_harness()
SESSION_ID = uuid.UUID("11111111-1111-4111-8111-111111111111")
TASK_ID = "22222222-2222-4222-8222-222222222222"
COMMAND_ID = "44444444-4444-4444-8444-444444444444"
WORKER_ID = "33333333-3333-4333-8333-333333333333"
ROSTER = {"tb3_0", "tb3_1", "tb3_2", "tb3_3"}
CREATED_AT = "2026-07-20T10:00:00Z"


def passing_task():
    robots = ["tb3_0", "tb3_1", "tb3_2", "tb3_3"]
    return {
        "id": TASK_ID,
        "sessionId": str(SESSION_ID),
        "type": "CollaborativeTransport",
        "state": "Completed",
        "progress": 1,
        "parameters": {
            "target_x": -3,
            "target_y": -4,
            "config": {
                "target_x": -3,
                "target_y": -4,
                "transport_planner": "grf",
            },
        },
        "result": {
            "transport": {
                "phase": "DONE",
                "searching_robot_count": 0,
                "all_pushers_confirmed": True,
                "useful_contributor_count": 4,
                "useful_contributor_ids": robots,
                "discovery": {
                    "event": "payload_found",
                    "task_id": TASK_ID,
                    "announced": True,
                    "finder": robots[0],
                    "notified_robots": robots[1:],
                },
            }
        },
        "error": None,
        "outcomeState": "Succeeded",
        "outcomeReason": None,
        "createdAt": CREATED_AT,
        "updatedAt": "2026-07-20T10:00:00Z",
    }


def passing_samples():
    return [
        {
            "state": "Running",
            "phase": "SEARCH",
            "searchingRobotCount": 4,
            "usefulContributorCount": 0,
            "allPushersConfirmed": False,
        },
        {
            "state": "Running",
            "phase": "PUSH",
            "searchingRobotCount": 0,
            "usefulContributorCount": 4,
            "allPushersConfirmed": True,
        },
        {
            "state": "Completed",
            "phase": "DONE",
            "searchingRobotCount": 0,
            "usefulContributorCount": 4,
            "allPushersConfirmed": True,
        },
    ]


def start_probe_document(conflicts=0):
    click_epoch = 1784541600000
    responses = []
    for index in range(conflicts):
        responses.append({
            "attempt": index + 1,
            "status": 409,
            "requestEpochMs": click_epoch + 20 + index * 200,
            "responseEpochMs": click_epoch + 30 + index * 200,
            "gestureToRequestMs": 20 + index * 200,
            "keyFingerprint": "a1b2c3d4e5f60718",
            "conflictCode": "serialization_conflict",
            "conflictRetryable": True,
            "taskId": None,
            "commandId": None,
            "taskType": None,
            "taskState": None,
            "commandType": None,
            "commandState": None,
            "taskCreatedAt": None,
            "commandCreatedAt": None,
            "taskCommandCorrelated": False,
            "targetAccepted": False,
        })
    accepted_request = click_epoch + 20 + conflicts * 200
    responses.append({
        "attempt": conflicts + 1,
        "status": 202,
        "requestEpochMs": accepted_request,
        "responseEpochMs": accepted_request + 15,
        "gestureToRequestMs": accepted_request - click_epoch,
        "keyFingerprint": "a1b2c3d4e5f60718",
        "conflictCode": None,
        "conflictRetryable": False,
        "taskId": TASK_ID,
        "commandId": COMMAND_ID,
        "taskSessionId": str(SESSION_ID),
        "commandSessionId": str(SESSION_ID),
        "commandTaskRunId": TASK_ID,
        "taskCreatedAt": CREATED_AT,
        "commandCreatedAt": CREATED_AT,
        "taskType": "CollaborativeTransport",
        "taskState": "Queued",
        "commandType": "StartTask",
        "commandState": "Pending",
        "taskCommandCorrelated": True,
        "targetAccepted": True,
    })
    return {
        "attempts": conflicts + 1,
        "completed": conflicts + 1,
        "physicalClickCount": 1,
        "clicks": [{
            "atEpochMs": click_epoch,
            "atPerformanceMs": 100,
            "trusted": True,
            "button": 0,
            "detail": 1,
        }],
        "logicalTaskCount": 1,
        "logicalCommandCount": 1,
        "responses": responses,
    }


def ros_documents(*, movement=True):
    discovery = {
        "event": "payload_found",
        "task_id": TASK_ID,
        "announced": True,
        "finder": "tb3_0",
        "notified_robots": ["tb3_1", "tb3_2", "tb3_3"],
    }
    paths = {name: 0.02 if movement else 0.0 for name in ROSTER}
    base = {
        "task_id": TASK_ID,
        "paused": False,
        "search_path_length_m": paths,
        "current_useful_pusher_count": 0,
        "current_useful_pusher_ids": [],
        "useful_contributor_count": 0,
        "useful_contributor_ids": [],
        "all_pushers_confirmed": False,
        "discovery": None,
    }
    documents = []
    for observed_at, phase, progress in (
        (1.0, "SEARCH", 0.00),
        (1.2, "SEARCH", 0.01),
        (1.4, "APPROACH", 0.05),
        (2.0, "PUSH", 0.20),
        (2.3, "PUSH", 0.205),
        (2.6, "PUSH", 0.215),
        (2.8, "DONE", 1.0),
    ):
        item = copy.deepcopy(base)
        item.update(observed_at=observed_at, phase=phase, progress=progress)
        item["searching_robot_count"] = 4 if phase == "SEARCH" else 0
        if phase in {"APPROACH", "PUSH", "DONE"}:
            item["discovery"] = copy.deepcopy(discovery)
        if phase == "PUSH":
            item["current_useful_pusher_count"] = 4
            item["current_useful_pusher_ids"] = sorted(ROSTER)
            item["useful_contributor_count"] = 4
            item["useful_contributor_ids"] = sorted(ROSTER)
            item["all_pushers_confirmed"] = True
        documents.append(item)
    return documents


def roster_response():
    return {
        "authenticated": True,
        "status": 200,
        "body": [
            {
                "runtimeId": f"tb3_{index}",
                "ordinal": index,
                "namespace": f"/tb3_{index}",
                "state": "Ready",
            }
            for index in range(4)
        ],
    }


class SequenceCdp:
    def __init__(self, values):
        self.values = list(values)
        self.expressions = []

    def evaluate(self, expression, **_kwargs):
        self.expressions.append(expression)
        if len(self.values) > 1:
            return self.values.pop(0)
        return self.values[0]


class FakeUi:
    def __init__(self, values):
        self.cdp = SequenceCdp(values)


class DockerFixture(HARNESS.DockerProof):
    def __init__(self, documents):
        super().__init__("docker", threading.Event())
        self.documents = documents

    def _resource_ids(self, kind, _session_id, *, interruptible):
        del interruptible
        return [self.documents[f"{kind}_id"]]

    def _inspect(self, kind, identifier):
        del identifier
        return copy.deepcopy(self.documents[kind])


def docker_documents(commit="a" * 40):
    container_id = "b" * 64
    network_id = "c" * 64
    image_id = "sha256:" + "d" * 64
    worker_id = WORKER_ID
    name = f"robotswarm-{SESSION_ID.hex}"
    network_name = f"{name}-net"
    labels = {
        HARNESS.MANAGED_LABEL: "true",
        HARNESS.SESSION_LABEL: str(SESSION_ID),
        HARNESS.WORKER_LABEL: worker_id,
        HARNESS.IMAGE_VERSION_LABEL: f"{commit}+{'d' * 12}",
    }
    return {
        "container_id": container_id,
        "network_id": network_id,
        "container": {
            "Name": "/" + name,
            "Image": image_id,
            "State": {"Running": True, "StartedAt": "2026-07-20T09:55:00Z"},
            "Config": {"Image": image_id, "Labels": labels},
            "NetworkSettings": {
                "Networks": {
                    network_name: {
                        "NetworkID": network_id,
                        "IPAddress": "172.24.0.2",
                    }
                }
            },
        },
        "image": {
            "Id": image_id,
            "Config": {"Labels": {HARNESS.IMAGE_REVISION_LABEL: commit}},
        },
        "network": {
            "Name": network_name,
            "Internal": True,
            "Labels": {
                HARNESS.MANAGED_LABEL: "true",
                HARNESS.SESSION_LABEL: str(SESSION_ID),
                HARNESS.WORKER_LABEL: worker_id,
            },
            "Containers": {container_id: {"Name": name}},
        },
    }


class TransportUiAcceptanceContractTests(unittest.TestCase):
    def test_scenario_is_fixed_and_versioned(self):
        self.assertEqual(HARNESS.SCHEMA_VERSION, 1)
        self.assertEqual(HARNESS.ROBOT_COUNT, 4)
        self.assertEqual((HARNESS.TARGET_X, HARNESS.TARGET_Y), (-3.0, -4.0))
        self.assertEqual(HARNESS.PLANNER, "grf")
        parsed = HARNESS.parse_utc_timestamp(
            "2026-07-20T10:00:00.987654321Z", "Docker"
        )
        self.assertEqual(parsed.microsecond, 987654)

    def test_cli_requires_explicit_mutating_inputs(self):
        required = {
            action.dest
            for action in HARNESS.build_parser()._actions
            if getattr(action, "required", False)
        }
        self.assertTrue(
            {
                "execute_production",
                "deployment_commit",
                "credentials",
                "chrome",
                "profile_root",
                "output",
            }.issubset(required)
        )

    def test_preflight_rejects_missing_authorization_and_short_sha(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            chrome = root / "chrome.exe"
            chrome.touch()
            output_root = root / "private"
            output_root.mkdir(mode=0o700)
            values = dict(
                execute_production=False,
                deployment_commit="a" * 40,
                chrome=chrome,
                profile_root=root,
                cdp_port=9352,
                ready_timeout=10,
                viewer_timeout=10,
                task_timeout=10,
                cleanup_timeout=10,
                video_seconds=5,
                output=output_root / "report.json",
                url=HARNESS.DEFAULT_URL,
            )
            with self.assertRaisesRegex(HARNESS.TransportSmokeError, "execute-production"):
                HARNESS.validate_options(argparse.Namespace(**values))
            values["execute_production"] = True
            values["deployment_commit"] = "abc123"
            with self.assertRaisesRegex(HARNESS.TransportSmokeError, "full lowercase"):
                HARNESS.validate_options(argparse.Namespace(**values))

    def test_account_a_credentials_keep_strict_0600_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "credentials.env"
            path.write_text(
                "TEST_A_ID=1\nTEST_A_EMAIL=a@example.invalid\nTEST_A_PASSWORD=secret-a\n"
                "TEST_B_ID=2\nTEST_B_EMAIL=b@example.invalid\nTEST_B_PASSWORD=secret-b\n",
                encoding="utf-8",
            )
            path.chmod(0o600)
            with mock.patch.object(HARNESS.VISIBLE.stat, "S_IMODE", return_value=0o600):
                account = HARNESS.VISIBLE.read_credentials(path)["A"]
            self.assertEqual(account["email"], "a@example.invalid")
            path.chmod(0o644)
            with mock.patch.object(HARNESS.VISIBLE.stat, "S_IMODE", return_value=0o644):
                with self.assertRaisesRegex(HARNESS.VISIBLE.DriverError, "0600"):
                    HARNESS.VISIBLE.read_credentials(path)

    def test_harness_never_imports_concurrent_matrix_or_loaded_runner(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("robotswarm-ros-matrix-e2e", source.replace(
            'Path("/tmp/robotswarm-ros-matrix-e2e.lock")', ""
        ))
        self.assertNotIn('HERE / "robotswarm-loaded-n4-e2e.py"', source)

    def test_github_actions_only_compiles_and_tests_the_offline_contract(self):
        workflows = list((REPOSITORY / ".github" / "workflows").glob("*.y*ml"))
        self.assertTrue(workflows)
        for workflow in workflows:
            contents = workflow.read_text(encoding="utf-8")
            self.assertIsNone(
                re.search(
                    rf"{re.escape(SCRIPT.name)}\s+--execute-production",
                    contents,
                ),
                msg=f"production smoke unexpectedly executed by {workflow.name}",
            )

    def test_visible_chrome_contract_forbids_headless_and_gpu_disable(self):
        launch_source = inspect.getsource(HARNESS.VISIBLE.OwnedChrome.launch)
        self.assertIn('forbidden = ("--headless", "--disable-gpu")', launch_source)
        source = inspect.getsource(HARNESS.run_smoke)
        self.assertEqual(source.count("VISIBLE.OwnedChrome("), 1)
        self.assertNotIn("--headless", source)
        self.assertNotIn("--disable-gpu", source)

    def test_docker_resource_list_arguments_use_two_exact_labels(self):
        proof = HARNESS.DockerProof("docker", threading.Event())
        calls = []

        def fake_run(arguments, **_kwargs):
            calls.append(arguments)
            return HARNESS.ProcessOutput(0, "", "")

        proof.run = fake_run
        self.assertEqual(proof._resource_ids("container", SESSION_ID, interruptible=False), [])
        self.assertEqual(proof._resource_ids("network", SESSION_ID, interruptible=False), [])
        self.assertEqual(calls[0][:4], ["ps", "-a", "--no-trunc", "--quiet"])
        self.assertEqual(calls[0].count("--filter"), 2)
        self.assertEqual(calls[1][:4], ["network", "ls", "--no-trunc", "--quiet"])
        self.assertEqual(calls[1].count("--filter"), 2)

    def test_docker_proof_requires_immutable_revision_and_internal_exclusive_network(self):
        proof = DockerFixture(docker_documents())
        handle, evidence = proof.verify_session(SESSION_ID, "a" * 40)
        self.assertEqual(handle.identifier, "b" * 64)
        self.assertEqual(handle.worker_identifier, WORKER_ID)
        self.assertLess(handle.started_at, dt.datetime(2026, 7, 20, 10, tzinfo=dt.timezone.utc))
        self.assertTrue(all(evidence.values()))
        self.assertNotIn(str(SESSION_ID), json.dumps(evidence))

        bad = docker_documents()
        bad["network"]["Internal"] = False
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "private network"):
            DockerFixture(bad).verify_session(SESSION_ID, "a" * 40)

        bad = docker_documents()
        bad["image"]["Config"]["Labels"][HARNESS.IMAGE_REVISION_LABEL] = "e" * 40
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "revision"):
            DockerFixture(bad).verify_session(SESSION_ID, "a" * 40)

    def test_ui_start_has_one_physical_click_and_explicit_hit_test_barrier(self):
        source = inspect.getsource(HARNESS.run_smoke)
        self.assertEqual(source.count("ui.click_button(START_BUTTON)"), 1)
        barrier = inspect.getsource(HARNESS.require_start_barrier)
        self.assertIn("document.elementFromPoint", barrier)
        self.assertIn("MuiBackdrop-root", barrier)
        self.assertIn("centerHitTest", barrier)
        self.assertIn("overlayFree", barrier)

    def test_target_fields_are_real_number_inputs_with_native_events(self):
        source = inspect.getsource(HARNESS.fill_labeled_number)
        self.assertIn("HTMLInputElement.prototype", source)
        self.assertIn("new Event('input'", source)
        self.assertIn("new Event('change'", source)
        run_source = inspect.getsource(HARNESS.run_smoke)
        self.assertIn('"Destino X (m)"', run_source)
        self.assertIn('"Destino Y (m)"', run_source)

    def test_xhr_probe_accepts_zero_through_three_exact_conflicts_then_one_202(self):
        probe_source = inspect.getsource(HARNESS.install_start_probe)
        self.assertIn("String(name).toLowerCase() === 'idempotency-key'", probe_source)
        self.assertNotIn("headers: new Map", probe_source)
        for conflicts in range(4):
            with self.subTest(conflicts=conflicts):
                evidence = HARNESS.read_start_probe(
                    FakeUi([start_probe_document(conflicts)]), SESSION_ID,
                    timeout=0.1,
                    quiet_period=0,
                )
                self.assertEqual(evidence.report["physicalClicks"], 1)
                self.assertEqual(
                    evidence.report["retryableSerializationConflicts"], conflicts
                )
                self.assertEqual(evidence.report["logicalTaskRuns"], 1)
                self.assertTrue(evidence.report["delayedRereadStable"])
                serialized = json.dumps(evidence.report)
                self.assertNotIn(TASK_ID, serialized)
                self.assertNotIn(COMMAND_ID, serialized)
                self.assertEqual(
                    evidence.report["idempotency"]["fingerprint"],
                    "a1b2c3d4e5f60718",
                )

    def test_xhr_probe_rejects_extra_or_untrusted_physical_click(self):
        extra = start_probe_document()
        extra["clicks"].append(copy.deepcopy(extra["clicks"][0]))
        extra["physicalClickCount"] = 2
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "one logical"):
            HARNESS.read_start_probe(
                FakeUi([extra]), SESSION_ID, timeout=0.02, quiet_period=0
            )

        untrusted = start_probe_document()
        untrusted["clicks"][0]["trusted"] = False
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "trusted primary"):
            HARNESS.read_start_probe(
                FakeUi([untrusted]), SESSION_ID, timeout=0.02, quiet_period=0
            )

    def test_xhr_probe_rejects_non_exact_conflict_or_changed_idempotency_key(self):
        for mutation in (
            "legacy", "not_retryable", "changed_key", "wrong_order", "identity",
        ):
            document = start_probe_document(2)
            if mutation == "legacy":
                document["responses"][0]["conflictCode"] = None
            elif mutation == "not_retryable":
                document["responses"][0]["conflictRetryable"] = False
            elif mutation == "changed_key":
                document["responses"][1]["keyFingerprint"] = "fedcba9876543210"
            elif mutation == "wrong_order":
                document["responses"][0]["status"] = 202
            else:
                document["responses"][0]["taskSessionId"] = str(SESSION_ID)
            with self.subTest(mutation=mutation):
                with self.assertRaises(HARNESS.TransportSmokeError):
                    HARNESS.read_start_probe(
                        FakeUi([document]), SESSION_ID, timeout=0.02, quiet_period=0
                    )

    def test_xhr_probe_rejects_409_only_and_two_accepted_responses(self):
        conflict_only = start_probe_document(1)
        conflict_only["responses"] = conflict_only["responses"][:1]
        conflict_only.update(
            attempts=1,
            completed=1,
            logicalTaskCount=0,
            logicalCommandCount=0,
        )
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "one logical"):
            HARNESS.read_start_probe(
                FakeUi([conflict_only]), SESSION_ID, timeout=0.01, quiet_period=0
            )

        two_accepted = start_probe_document()
        second = copy.deepcopy(two_accepted["responses"][0])
        second["attempt"] = 2
        second["requestEpochMs"] += 100
        second["responseEpochMs"] += 100
        second["gestureToRequestMs"] += 100
        two_accepted["responses"].append(second)
        two_accepted["attempts"] = 2
        two_accepted["completed"] = 2
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "conflicts followed by one 202"):
            HARNESS.read_start_probe(
                FakeUi([two_accepted]), SESSION_ID, timeout=0.02, quiet_period=0
            )

    def test_xhr_probe_rejects_late_request_and_noncausal_timeline(self):
        settled = start_probe_document()
        late = copy.deepcopy(settled)
        late["physicalClickCount"] = 2
        late["clicks"].append(copy.deepcopy(late["clicks"][0]))
        with mock.patch.object(HARNESS.time, "sleep", return_value=None):
            with self.assertRaisesRegex(HARNESS.TransportSmokeError, "late extra"):
                HARNESS.read_start_probe(
                    FakeUi([settled, late]), SESSION_ID,
                    timeout=0.1, quiet_period=0.01
                )

        noncausal = start_probe_document()
        noncausal["responses"][-1]["requestEpochMs"] -= 100
        noncausal["responses"][-1]["gestureToRequestMs"] = -100
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "timeline"):
            HARNESS.read_start_probe(
                FakeUi([noncausal]), SESSION_ID, timeout=0.02, quiet_period=0
            )

    def test_task_watcher_is_installed_before_the_only_click_and_polls_at_100ms(self):
        source = inspect.getsource(HARNESS.run_smoke)
        self.assertLess(source.index("install_task_watcher"), source.index("ui.click_button(START_BUTTON)"))
        watcher = inspect.getsource(HARNESS.install_task_watcher)
        self.assertIn("setInterval(poll, 100)", watcher)
        self.assertIn("MAXIMUM_TASK_DOCUMENT", watcher)
        self.assertIn("discoveryCorrelated", watcher)
        self.assertIn("stateRegression", watcher)
        self.assertIn("phaseRegression", watcher)
        self.assertIn("criticalSamples", watcher)

    def test_task_watcher_fails_closed_on_a_phase_regression(self):
        watch = {
            "error": None,
            "states": ["Accepted", "Running"],
            "phases": ["SEARCH", "APPROACH", "SEARCH"],
            "samples": [],
            "criticalSamples": [],
            "stateRegression": False,
            "phaseRegression": True,
        }
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "automaton regressed"):
            HARNESS.read_task_watcher(FakeUi([watch]))

    def test_authoritative_roster_comes_from_the_exact_session_endpoint(self):
        ui = FakeUi([roster_response()])
        roster = HARNESS.authoritative_robot_roster(ui, SESSION_ID)
        self.assertEqual(roster, ROSTER)
        self.assertIn(f"/api/sessions/{SESSION_ID}/robots", ui.cdp.expressions[0])

        for mutation in ("count", "duplicate", "namespace", "state"):
            response = roster_response()
            if mutation == "count":
                response["body"].pop()
            elif mutation == "duplicate":
                response["body"][3]["runtimeId"] = "tb3_2"
                response["body"][3]["namespace"] = "/tb3_2"
            elif mutation == "namespace":
                response["body"][0]["namespace"] = "/wrong"
            else:
                response["body"][0]["state"] = "Removed"
            with self.subTest(mutation=mutation):
                with self.assertRaises(HARNESS.TransportSmokeError):
                    HARNESS.authoritative_robot_roster(FakeUi([response]), SESSION_ID)

    def test_private_post_task_worker_and_container_correlation_is_causal(self):
        click_epoch = dt.datetime(2026, 7, 20, 10, tzinfo=dt.timezone.utc).timestamp() * 1000
        start = HARNESS.StartProbeEvidence(
            report={},
            task_id=TASK_ID,
            command_id=COMMAND_ID,
            task_created_at=CREATED_AT,
            command_created_at=CREATED_AT,
            click_epoch_ms=click_epoch,
            request_epoch_ms=click_epoch + 10,
            response_epoch_ms=click_epoch + 50,
        )
        container = HARNESS.ContainerHandle(
            "b" * 64,
            WORKER_ID,
            dt.datetime(2026, 7, 20, 9, 55, tzinfo=dt.timezone.utc),
        )
        evidence = HARNESS.correlate_private_start(
            start, passing_task(), SESSION_ID, container
        )
        self.assertTrue(all(evidence.values()))
        self.assertNotIn(TASK_ID, json.dumps(evidence))
        self.assertNotIn(WORKER_ID, json.dumps(evidence))

        wrong_task = passing_task()
        wrong_task["id"] = str(SESSION_ID)
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "do not match"):
            HARNESS.correlate_private_start(start, wrong_task, SESSION_ID, container)
        late_container = HARNESS.ContainerHandle(
            "b" * 64,
            WORKER_ID,
            dt.datetime(2026, 7, 20, 10, 1, tzinfo=dt.timezone.utc),
        )
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "established before"):
            HARNESS.correlate_private_start(
                start, passing_task(), SESSION_ID, late_container
            )

    def test_private_ros_observer_proves_strict_automaton_and_sustained_push(self):
        evidence = HARNESS.validate_ros_transport_observation(
            ros_documents(), TASK_ID, ROSTER
        )
        self.assertEqual(
            evidence["strictAutomaton"],
            ["SEARCH", "APPROACH", "PUSH", "DONE"],
        )
        self.assertTrue(evidence["discoveryDuringApproach"])
        self.assertTrue(evidence["search"]["movementDirectlyObserved"])
        self.assertGreaterEqual(evidence["push"]["sustainedSamples"], 3)
        self.assertGreaterEqual(evidence["push"]["progressGain"], 0.001)
        serialized = json.dumps(evidence)
        self.assertNotRegex(serialized, HARNESS.ROBOT_PATTERN)
        self.assertNotIn(TASK_ID, serialized)

    def test_private_ros_observer_rejects_regression_missing_notice_and_foreign_task(self):
        regression = ros_documents()
        regression.insert(3, copy.deepcopy(regression[0]))
        regression[3]["observed_at"] = 1.6
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "regressed"):
            HARNESS.validate_ros_transport_observation(regression, TASK_ID, ROSTER)

        notice = ros_documents()
        for item in notice:
            if item["phase"] == "APPROACH":
                item["discovery"]["notified_robots"].pop()
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "fleet-wide notice"):
            HARNESS.validate_ros_transport_observation(notice, TASK_ID, ROSTER)

        foreign = ros_documents()
        outsider = copy.deepcopy(foreign[0])
        outsider["task_id"] = str(SESSION_ID)
        foreign.insert(0, outsider)
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "another task"):
            HARNESS.validate_ros_transport_observation(foreign, TASK_ID, ROSTER)

    def test_private_ros_observer_rejects_partial_or_token_push(self):
        for mutation in ("partial", "short", "no_progress"):
            documents = ros_documents()
            pushes = [item for item in documents if item["phase"] == "PUSH"]
            if mutation == "partial":
                pushes[1]["current_useful_pusher_count"] = 3
                pushes[1]["current_useful_pusher_ids"].pop()
            elif mutation == "short":
                pushes[-1]["observed_at"] = 2.35
            else:
                for item in pushes:
                    item["progress"] = 0.2
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(HARNESS.TransportSmokeError, "sustain"):
                    HARNESS.validate_ros_transport_observation(
                        documents, TASK_ID, ROSTER
                    )

        cumulative_only = ros_documents()
        for item in cumulative_only:
            if item["phase"] == "PUSH":
                item["current_useful_pusher_count"] = 0
                item["current_useful_pusher_ids"] = []
                item["useful_contributor_count"] = 4
                item["useful_contributor_ids"] = sorted(ROSTER)
                item["all_pushers_confirmed"] = True
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "sustain"):
            HARNESS.validate_ros_transport_observation(
                cumulative_only, TASK_ID, ROSTER
            )

    def test_search_rejects_any_robot_without_enough_physical_movement(self):
        with self.assertRaisesRegex(
            HARNESS.TransportSmokeError,
            "physical movement for every robot",
        ):
            HARNESS.validate_ros_transport_observation(
                ros_documents(movement=False), TASK_ID, ROSTER
            )

    def test_terminal_contract_accepts_only_finder_three_notified_and_four_pushers(self):
        evidence = HARNESS.validate_transport_task(
            passing_task(),
            ["Queued", "Accepted", "Running", "Completed"],
            ["SEARCH", "APPROACH", "PUSH", "DONE"],
            SESSION_ID,
            passing_samples(),
            ROSTER,
        )
        self.assertEqual(evidence["discovery"]["notifiedTeammates"], 3)
        self.assertEqual(evidence["push"]["usefulContributors"], 4)
        self.assertTrue(evidence["workerCommandConsumptionProven"])
        self.assertTrue(evidence["done"])
        self.assertNotRegex(json.dumps(evidence), HARNESS.ROBOT_PATTERN)
        self.assertNotIn(TASK_ID, json.dumps(evidence))

    def test_terminal_contract_rejects_missing_state_or_phase(self):
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "Accepted"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Queued", "Running", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )

    def test_terminal_contract_rejects_persisted_state_or_phase_regression(self):
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "state history regressed"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Accepted", "Running", "Accepted", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "phase history regressed"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Accepted", "Running", "Completed"],
                ["SEARCH", "PUSH", "APPROACH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "SEARCH"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Accepted", "Running", "Completed"],
                ["APPROACH", "PUSH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )

    def test_terminal_contract_rejects_partial_notice_or_contributors(self):
        for mutation in ("notice", "contributors", "confirmation", "correlation"):
            task = passing_task()
            transport = task["result"]["transport"]
            if mutation == "notice":
                transport["discovery"]["notified_robots"] = ["tb3_1", "tb3_2"]
            elif mutation == "contributors":
                transport["useful_contributor_count"] = 3
                transport["useful_contributor_ids"] = ["tb3_0", "tb3_1", "tb3_2"]
            elif mutation == "confirmation":
                transport["all_pushers_confirmed"] = False
            else:
                transport["discovery"]["task_id"] = str(SESSION_ID)
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(HARNESS.TransportSmokeError, "finder, notice"):
                    HARNESS.validate_transport_task(
                        task,
                        ["Accepted", "Running", "Completed"],
                        ["SEARCH", "PUSH", "DONE"],
                        SESSION_ID,
                        passing_samples(),
                        ROSTER,
                    )

    def test_terminal_contract_rejects_failure_and_unexpected_collision(self):
        failed = passing_task()
        failed["error"] = "planner failed"
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "error"):
            HARNESS.validate_transport_task(
                failed,
                ["Accepted", "Running", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )
        collision = passing_task()
        collision["result"]["unexpected_collision_count"] = 1
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "collision"):
            HARNESS.validate_transport_task(
                collision,
                ["Accepted", "Running", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                passing_samples(),
                ROSTER,
            )

    def test_terminal_contract_requires_all_four_live_searchers_and_pushers(self):
        samples = passing_samples()
        samples[0]["searchingRobotCount"] = 3
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "all four robots searching"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Accepted", "Running", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                samples,
                ROSTER,
            )
        samples = passing_samples()
        samples[1]["usefulContributorCount"] = 3
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "four useful pushers"):
            HARNESS.validate_transport_task(
                passing_task(),
                ["Accepted", "Running", "Completed"],
                ["SEARCH", "PUSH", "DONE"],
                SESSION_ID,
                samples,
                ROSTER,
            )

    def test_report_sanitizer_removes_credentials_uuid_worker_and_private_ip(self):
        unsafe = {
            "message": (
                f"a@example.invalid {TASK_ID} worker: gpu-lan 10.0.0.126 secret-value"
            )
        }
        safe = HARNESS.sanitize_value(unsafe, ("secret-value",))
        serialized = json.dumps(safe)
        self.assertNotIn("a@example.invalid", serialized)
        self.assertNotIn(TASK_ID, serialized)
        self.assertNotIn("gpu-lan", serialized)
        self.assertNotIn("10.0.0.126", serialized)
        self.assertNotIn("secret-value", serialized)
        HARNESS.assert_report_safe(safe, ("secret-value",))

    def test_phase_and_final_captures_are_sanitized_and_correlated(self):
        source = inspect.getsource(HARNESS.capture_correlated_phase)
        self.assertIn("before_task", source)
        self.assertIn("after_task", source)
        self.assertIn("ui.screenshot", source)
        self.assertIn("ui.capture_video_clip", source)
        run_source = inspect.getsource(HARNESS.run_smoke)
        self.assertEqual(run_source.count("capture_correlated_phase("), 2)
        self.assertIn('"SEARCH"', run_source)
        self.assertIn('"PUSH"', run_source)
        self.assertIn('safe_capture(final_capture, "DONE")', run_source)

    def test_two_task_correlated_frames_must_change_visually(self):
        first = HARNESS.PhaseCapture([], TASK_ID, "0" * 64)
        second = HARNESS.PhaseCapture([], TASK_ID, "0" * 63 + "1")
        evidence = HARNESS.validate_visual_change(first, second)
        self.assertEqual(evidence["correlatedFrames"], 2)
        self.assertTrue(evidence["differentFrames"])

        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "did not change"):
            HARNESS.validate_visual_change(first, HARNESS.PhaseCapture([], TASK_ID, "0" * 64))
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "different tasks"):
            HARNESS.validate_visual_change(
                first,
                HARNESS.PhaseCapture([], str(SESSION_ID), "f" * 64),
            )
        with tempfile.TemporaryDirectory() as temporary:
            left = Path(temporary) / "left.png"
            right = Path(temporary) / "right.png"
            left.touch()
            right.touch()
            with mock.patch.object(
                HARNESS.VISIBLE.RobotSwarmUi,
                "_scene_difference",
                return_value=0.0,
            ):
                with self.assertRaisesRegex(HARNESS.TransportSmokeError, "did not change"):
                    HARNESS.validate_visual_change(
                        HARNESS.PhaseCapture([], TASK_ID, "0" * 64, left),
                        HARNESS.PhaseCapture([], TASK_ID, "f" * 64, right),
                    )

    def test_viewer_lease_identity_is_kept_private_until_terminal_close(self):
        lease_id = uuid.UUID("55555555-5555-4555-8555-555555555555")
        active = {
            "authenticated": True,
            "status": 200,
            "body": {
                "leaseId": str(lease_id),
                "sessionId": str(SESSION_ID),
                "isReady": True,
                "revokedAt": None,
                "command": {"state": "Completed"},
                "closeCommand": None,
            },
        }
        binding = HARNESS.ViewerBinding(Path("/tmp/private-lease"), lease_id)
        evidence = HARNESS.require_active_viewer_lease(
            FakeUi([active]), SESSION_ID, binding
        )
        self.assertTrue(all(evidence.values()))
        self.assertNotIn(str(lease_id), json.dumps(evidence))

        for mutation in ("inactive", "revoked", "pending"):
            invalid = copy.deepcopy(active)
            if mutation == "inactive":
                invalid["body"]["isReady"] = False
            elif mutation == "revoked":
                invalid["body"]["revokedAt"] = "2026-07-20T10:00:30Z"
            else:
                invalid["body"]["command"]["state"] = "Running"
            with self.subTest(mutation=mutation):
                with self.assertRaisesRegex(HARNESS.TransportSmokeError, "not active"):
                    HARNESS.require_active_viewer_lease(
                        FakeUi([invalid]), SESSION_ID, binding
                    )

        closing = copy.deepcopy(active)
        closing["body"].update(
            isReady=False,
            revokedAt="2026-07-20T10:01:00Z",
            closeCommand={"state": "Running"},
        )
        closed = copy.deepcopy(closing)
        closed["body"]["closeCommand"] = {"state": "Completed"}
        with mock.patch.object(HARNESS.time, "sleep", return_value=None):
            close_evidence = HARNESS.wait_viewer_lease_closed(
                FakeUi([closing, closed]), SESSION_ID, binding, 0.1
            )
        self.assertTrue(all(close_evidence.values()))
        self.assertNotIn(str(lease_id), json.dumps(close_evidence))

        failed = copy.deepcopy(closing)
        failed["body"]["closeCommand"] = {"state": "Failed"}
        with self.assertRaisesRegex(HARNESS.TransportSmokeError, "did not complete"):
            HARNESS.wait_viewer_lease_closed(
                FakeUi([failed]), SESSION_ID, binding, 0.01
            )

    def test_failed_chrome_launch_still_removes_only_its_owned_profile(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "owned-profile"
            profile.mkdir(mode=0o700)
            marker = profile / ".robotswarm-visible-owner.json"
            marker.write_text(
                json.dumps({
                    "runId": "run-1",
                    "label": "A",
                    "port": 9352,
                    "pid": None,
                }),
                encoding="utf-8",
            )
            chrome = types.SimpleNamespace(
                process=None,
                port=9352,
                profile=profile,
                marker=marker,
                run_id="run-1",
                label="A",
            )
            with mock.patch.object(HARNESS.VISIBLE, "port_is_free", return_value=True):
                outcome = HARNESS.cleanup_unstarted_profile(chrome)
            self.assertTrue(outcome["profileRemoved"])
            self.assertFalse(profile.exists())

            refused = Path(temporary) / "refused-profile"
            refused.mkdir(mode=0o700)
            refused_marker = refused / ".robotswarm-visible-owner.json"
            refused_marker.write_text(
                json.dumps({"runId": "other", "label": "A", "port": 9352, "pid": None}),
                encoding="utf-8",
            )
            chrome.profile = refused
            chrome.marker = refused_marker
            with mock.patch.object(HARNESS.VISIBLE, "port_is_free", return_value=True):
                outcome = HARNESS.cleanup_unstarted_profile(chrome)
            self.assertFalse(outcome["profileRemoved"])
            self.assertTrue(refused.exists())

    def test_video_gate_requires_progress_during_same_running_task(self):
        source = inspect.getsource(HARNESS.video_evidence)
        self.assertIn("require_live_video", source)
        self.assertIn('before_task.get("state") != "Running"', source)
        self.assertIn('after_task.get("state") != "Running"', source)
        self.assertEqual(HARNESS.MINIMUM_VIDEO_FPS, 27.0)

    def test_finally_proves_every_resource_and_browser_cleanup(self):
        cleanup_source = inspect.getsource(HARNESS.cleanup_run)
        for marker in (
            "viewerClosed",
            "sessionReleased",
            "workspaceReleased",
            "containerAbsent",
            "networkAbsent",
            "viewerRuntimeAbsent",
            "viewerPublisherAbsent",
            "viewerLeaseInactive",
            "viewerLeaseRevoked",
            "viewerCloseCommandTerminal",
        ):
            self.assertIn(marker, cleanup_source)
        run_source = inspect.getsource(HARNESS.run_smoke)
        self.assertIn("finally:", run_source)
        self.assertIn("chrome.close_owned()", run_source)
        self.assertIn("cdpPortReleased", run_source)
        self.assertIn("profileRemoved", run_source)
        self.assertIn("cleanup_unstarted_profile", run_source)
        self.assertIn("rosObserverExited", run_source)


if __name__ == "__main__":
    unittest.main()

"""Regression tests for the uncalibrated-camera workaround.

The PhaseSpace server reports uncalibrated cameras as an *error* event (e.g.
"1 uncalibrated cameras detected: 12") even though it still initializes and
tracks with the remaining cameras. The stock client treated that as a fatal
InitError, so the ROS nodes could never connect and no data came out at all.

These tests pin down the behaviour we want:
  * an uncalibrated-camera error never raises, whatever camera it names,
  * every other error stays fatal,
  * the uncalibrated-camera error is dropped instead of reaching the user.

Run with:  python3 -m pytest tests/ -v
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Ros_2"))

from phasespace_node_ros2 import (  # noqa: E402
    Context,
    Event,
    InitError,
    Type,
    error_text,
    is_uncalibrated_camera_error,
)

UNCALIBRATED = b"1 uncalibrated cameras detected: 12"


def test_detects_the_servers_uncalibrated_camera_message():
    assert is_uncalibrated_camera_error(UNCALIBRATED)
    # any camera id, singular or plural
    assert is_uncalibrated_camera_error(b"1 uncalibrated cameras detected: 1")
    assert is_uncalibrated_camera_error(b"2 uncalibrated cameras detected: 1, 12")
    assert is_uncalibrated_camera_error(b"1 uncalibrated camera detected: 1")
    # str payloads and empty ones
    assert is_uncalibrated_camera_error("1 uncalibrated cameras detected: 12")
    assert not is_uncalibrated_camera_error(None)


def test_other_errors_are_not_mistaken_for_camera_problems():
    for other in (b"hub failure", b"no connection", b"timed out", b"init failed", b""):
        assert not is_uncalibrated_camera_error(other)


def test_error_text_reads_bytes_and_str():
    assert error_text(UNCALIBRATED) == "1 uncalibrated cameras detected: 12"
    assert error_text("hello") == "hello"
    assert error_text(None) == ""


def _client():
    """A Context with the event tables filled in, so it can run without a server."""
    client = Context()
    client._Context__types[Type.ERROR] = {"name": "error"}
    client._Context__names[2] = {"name": "error"}
    return client


def _feed(client, data, event_id=2, initializing=True):
    """Push an ERROR event through the client's own event pipeline."""
    client._Context__properties["initializing"] = 1 if initializing else 0
    try:
        return client._Context__process_event(Event(Type.ERROR, event_id, data=data))
    finally:
        client._Context__properties.pop("initializing", None)


def test_uncalibrated_camera_error_does_not_fail_the_client():
    client = _client()
    for event_id in (0, 2):  # internal and normal event ids
        for message in (UNCALIBRATED,
                        b"1 uncalibrated cameras detected: 1",
                        b"2 uncalibrated cameras detected: 1, 12"):
            _feed(client, message, event_id=event_id)  # must not raise


def test_other_errors_stay_fatal():
    client = _client()
    for event_id in (0, 2):
        try:
            _feed(client, b"hub failure", event_id=event_id)
        except InitError:
            continue
        raise AssertionError(f"other errors must stay fatal (event id {event_id})")


def test_uncalibrated_camera_error_is_dropped_not_delivered():
    client = _client()
    _feed(client, UNCALIBRATED, initializing=False)
    delivered = []
    while True:
        event = client.nextEvent(0)
        if not event:
            break
        delivered.append(event)
    assert delivered == []

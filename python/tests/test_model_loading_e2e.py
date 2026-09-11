"""Native model I/O must leave Python free to consume controller status."""

import asyncio
import os
import shutil
import socket
import subprocess
import sys

import pytest

from par6 import config as paths
from par6._par6 import CollisionWorld, GravityModel, Preview
from par6.calibration.session import feedback_age


@pytest.mark.e2e
@pytest.mark.skipif(os.name != "posix", reason="uses a FIFO to control real file I/O")
@pytest.mark.parametrize("kind", ["gravity", "preview", "collision"])
@pytest.mark.timeout(30)
async def test_native_model_loading_preserves_status_delivery(
    kind, calibration_daemon, tmp_path
):
    original = calibration_daemon.config
    config = tmp_path / "model" / original.name
    shutil.copytree(original.parent, config.parent)
    tool = paths.config().active_gripper()
    if kind == "collision":
        source = paths.srdf_path(tool)
        fifo = config.parent / "loading.srdf"
    else:
        source = sorted((original.parent / "grippers").glob("*.toml"))[0]
        fifo = config.parent / "grippers" / source.name
        fifo.unlink()
    os.mkfifo(fifo, mode=0o600)
    decision = tmp_path / f"loading-{kind}-decision.txt"
    control, writer_control = socket.socketpair()
    control.setblocking(False)
    # The external writer keeps a genuine model read blocked until Python
    # consumes a fresh controller packet. Its deadline bounds the broken case:
    # a native loader holding the GIL cannot send that release.
    writer = subprocess.Popen(
        [
            sys.executable,
            "-c",
            """
import pathlib, signal, socket, sys
signal.alarm(12)
channel = socket.socket(fileno=int(sys.argv[1]))
channel.settimeout(3)
data = pathlib.Path(sys.argv[2]).read_bytes()
with open(sys.argv[3], 'wb', buffering=0) as stream:
    # A parser may reopen its path; only the initial read is gated.
    path = pathlib.Path(sys.argv[3])
    path.unlink()
    path.write_bytes(data)
    stream.write(data)
    channel.sendall(b'R')
    try:
        released = channel.recv(1) == b'G'
    except TimeoutError:
        released = False
    pathlib.Path(sys.argv[4]).write_text('released' if released else 'timed out')
""",
            str(writer_control.fileno()),
            str(source),
            str(fifo),
            str(decision),
        ],
        pass_fds=(writer_control.fileno(),),
    )
    writer_control.close()

    def load():
        if kind == "gravity":
            return GravityModel(str(config), str(paths.data_root()))
        if kind == "preview":
            return Preview(
                config=str(config),
                assets=str(paths.data_root()),
                package_dir=str(paths.package_search_dir()),
            )
        return CollisionWorld(
            str(paths.urdf_path(tool)), str(paths.package_search_dir()), str(fifo)
        )

    loading = None
    try:
        async with calibration_daemon.client() as client:
            assert await client.wait_ready(timeout=10)
            core = await client._ensure_core()
            before = await core.status_after(-1, 0.5)
            assert before is not None
            loading = asyncio.create_task(asyncio.to_thread(load))
            loop = asyncio.get_running_loop()
            assert await asyncio.wait_for(loop.sock_recv(control, 1), 5) == b"R"
            assert not decision.exists(), (
                "Native model loading held the GIL until the I/O deadline"
            )
            status = await core.status_after(before["seq"], 0.5)
            assert status is not None and status["seq"] != before["seq"]
            assert feedback_age(status) < 0.1
            assert not decision.exists(), "Status could not arrive during model I/O"
            await loop.sock_sendall(control, b"G")
            model = await asyncio.wait_for(loading, 10)
            assert decision.read_text() == "released"
            # The unblocked constructor must still produce a working native model.
            if kind == "gravity":
                assert len(model.gravity([0] * 6)) == 6
            elif kind == "collision":
                assert model.pair_count() > 0
            else:
                assert len(model.angles_rad()) == 6
    finally:
        control.close()
        if loading is not None:
            await asyncio.wait_for(asyncio.shield(loading), 10)
        if writer.poll() is None:
            await asyncio.to_thread(writer.wait, timeout=5)

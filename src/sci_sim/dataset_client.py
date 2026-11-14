from pathlib import Path
import os
import subprocess
import sys

from .dataset_worker import read_message, write_message


def data_python():
    configured = os.environ.get("SCI_SIM_DATA_PYTHON")
    if configured:
        return configured
    # Default sibling venv created by the installation scripts.
    sibling = Path(sys.prefix).parent / "sci-data" / "bin/python"
    if sibling.is_file():
        return str(sibling)
    return sys.executable


class DatasetClient:
    """Synchronous private pipe: one acknowledged write per simulation frame."""
    def __init__(self, **arguments):
        root = Path(arguments["root"])
        root.parent.mkdir(parents=True, exist_ok=True)
        self.log = root.with_name(root.name + "-writer.log").open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            [data_python(), "-m", "sci_sim.dataset_worker"], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.log)
        self.closed = False
        try:
            self.call("create", arguments=arguments)
        except Exception:
            self.process.terminate()
            self.process.wait(timeout=15)
            self.log.close()
            raise

    def call(self, operation, **values):
        try:
            write_message(self.process.stdin, {"operation": operation, **values})
            result = read_message(self.process.stdout)
        except (EOFError, BrokenPipeError) as error:
            raise RuntimeError("LeRobot writer exited. See the adjacent *-writer.log file.") from error
        if not result["ok"]:
            raise RuntimeError(result["error"])

    def add_frame(self, frame):
        self.call("add_frame", frame=frame)

    def save_episode(self):
        self.call("save_episode")

    def clear_episode_buffer(self):
        self.call("clear_episode_buffer")

    def finalize(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.call("finalize")
        finally:
            self.process.stdin.close()
            self.process.wait(timeout=60)
            self.process.stdout.close()
            self.log.close()

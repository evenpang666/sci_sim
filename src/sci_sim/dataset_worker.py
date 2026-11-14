"""LeRobot writer subprocess, isolated from MPlib's NumPy <2 requirement.

The binary pipe is private to the parent process. It accepts only locally generated
pickle messages; do not expose this protocol as a network service or feed untrusted input.
"""
import pickle
import struct
import sys
import traceback


def read_message(stream):
    header = stream.read(8)
    if not header:
        raise EOFError
    size = struct.unpack("!Q", header)[0]
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise EOFError("truncated private dataset pipe")
        chunks.extend(chunk)
    return pickle.loads(chunks)


def write_message(stream, value):
    data = pickle.dumps(value, protocol=5)
    stream.write(struct.pack("!Q", len(data)))
    stream.write(data)
    stream.flush()


def main():
    channel = sys.stdout.buffer
    sys.stdout = sys.stderr  # third-party logs cannot corrupt the binary response pipe
    dataset = None
    try:
        while True:
            try:
                request = read_message(sys.stdin.buffer)
            except EOFError:
                break
            operation = request["operation"]
            try:
                if operation == "create":
                    from lerobot.datasets.lerobot_dataset import LeRobotDataset
                    dataset = LeRobotDataset.create(**request["arguments"])
                elif operation == "add_frame":
                    dataset.add_frame(request["frame"])
                elif operation == "save_episode":
                    dataset.save_episode()
                elif operation == "clear_episode_buffer":
                    dataset.clear_episode_buffer()
                elif operation == "finalize":
                    dataset.finalize()
                else:
                    raise ValueError(operation)
                write_message(channel, {"ok": True})
            except Exception:
                write_message(channel, {"ok": False, "error": traceback.format_exc()})
            if operation == "finalize":
                break
    finally:
        if dataset is not None:
            dataset.finalize()


if __name__ == "__main__":
    main()

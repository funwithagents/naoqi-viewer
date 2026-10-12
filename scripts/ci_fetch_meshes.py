"""Run `naoqi-viewer fetch-meshes` under a pseudo-terminal and type `yes`, as CI's meshes job does.

fetch-meshes has no flag or variable that skips its license prompt, and refuses a stdin
that is not a terminal. CI therefore answers it like a person would: this script gives the
command a terminal, waits for the prompt, and types `yes`. Running the job is the
maintainer accepting the license for CI (specs/ci.md).

Usage: python scripts/ci_fetch_meshes.py [-- COMMAND...]   (default: naoqi-viewer fetch-meshes)
"""

import os
import pty
import select
import sys
import time

PROMPT = b"Type yes to accept the license"
TIMEOUT_S = 300


def run(command: list[str]) -> int:
    pid, fd = pty.fork()
    if pid == 0:  # the child, on the pseudo-terminal
        try:
            os.execvp(command[0], command)
        finally:
            os._exit(127)
    seen = b""
    answered = False
    deadline = time.monotonic() + TIMEOUT_S
    while True:
        if time.monotonic() > deadline:
            os.kill(pid, 9)
            print(f"\nci_fetch_meshes: timed out after {TIMEOUT_S} s", file=sys.stderr)
            os.waitpid(pid, 0)
            return 1
        ready, _, _ = select.select([fd], [], [], 1.0)
        if not ready:
            continue
        try:
            chunk = os.read(fd, 4096)
        except OSError:  # Linux reports the closed terminal as EIO
            chunk = b""
        if not chunk:
            break
        sys.stdout.buffer.write(chunk)
        sys.stdout.flush()
        seen = (seen + chunk)[-4096:]
        if not answered and PROMPT in seen:
            os.write(fd, b"yes\n")
            answered = True
    os.close(fd)
    _, status = os.waitpid(pid, 0)
    code = os.waitstatus_to_exitcode(status)
    if not answered:
        print("\nci_fetch_meshes: the license prompt never came", file=sys.stderr)
        return code or 1
    return code


def main(argv: list[str]) -> int:
    command = (
        argv[argv.index("--") + 1 :]
        if "--" in argv
        else ["naoqi-viewer", "fetch-meshes"]
    )
    return run(command)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

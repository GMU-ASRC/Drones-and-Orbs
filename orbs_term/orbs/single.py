import fcntl
import os

LOCK_PATH = os.environ.get("ORBS_LINK_LOCK", "/tmp/orbs_link.lock")


def acquire(path=LOCK_PATH):
    try:
        handle = open(path, "a+")
    except OSError:
        return None
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    try:
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
    except OSError:
        pass
    return handle


def holder(path=LOCK_PATH):
    try:
        handle = open(path, "a+")
    except OSError:
        return 0
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        try:
            handle.seek(0)
            return int(handle.read().strip() or 0)
        except (OSError, ValueError):
            return -1
        finally:
            handle.close()
    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    handle.close()
    return 0

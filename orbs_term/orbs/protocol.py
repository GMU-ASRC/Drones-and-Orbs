import json
import time

from websockets.extensions.permessage_deflate import (
    ClientPerMessageDeflateFactory, ServerPerMessageDeflateFactory)

VERSION = 1

DEFLATE = {"level": 6, "memLevel": 7}
WINDOW = 15


def client_deflate():
    return ClientPerMessageDeflateFactory(
        server_max_window_bits=WINDOW, client_max_window_bits=WINDOW,
        compress_settings=DEFLATE)


def server_deflate():
    return ServerPerMessageDeflateFactory(
        server_max_window_bits=WINDOW, client_max_window_bits=WINDOW,
        compress_settings=DEFLATE)

HELLO = "hello"
STATE = "state"
VISION = "vision"
SYSTEM = "system"
EVENT = "event"
ARCHIVE_OFFER = "archive_offer"
ARCHIVE_ACCEPT = "archive_accept"
ARCHIVE_DONE = "archive_done"
ARCHIVE_OK = "archive_ok"
ARCHIVE_FAIL = "archive_fail"
PING = "ping"
PONG = "pong"
BYE = "bye"

CHUNK_BYTES = 64 * 1024


def encode(kind, **fields):
    fields["t"] = kind
    fields.setdefault("ts", round(time.time(), 3))
    return json.dumps(fields, separators=(",", ":"))


def decode(raw):
    try:
        message = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(message, dict) or "t" not in message:
        return None
    return message


def hello(drone_id, name, version=VERSION, **extra):
    return encode(HELLO, id=drone_id, name=name, v=version, **extra)


def state(**fields):
    return encode(STATE, **fields)


def vision(**fields):
    return encode(VISION, **fields)


def system(**fields):
    return encode(SYSTEM, **fields)


def event(source, text):
    return encode(EVENT, src=source, msg=text)


def archive_offer(name, size, digest, session):
    return encode(ARCHIVE_OFFER, name=name, size=size, sha=digest,
                  session=session)


def archive_accept(name, resume_at):
    return encode(ARCHIVE_ACCEPT, name=name, at=resume_at)


def archive_done(name):
    return encode(ARCHIVE_DONE, name=name)


def archive_ok(name, path):
    return encode(ARCHIVE_OK, name=name, path=path)


def archive_fail(name, reason):
    return encode(ARCHIVE_FAIL, name=name, reason=reason)

import hashlib
import os
import time

from websockets.asyncio.server import serve

from . import protocol


class Incoming:
    def __init__(self, drone, name, size, digest, session, path):
        self.drone = drone
        self.name = name
        self.size = size
        self.digest = digest
        self.session = session
        self.path = path
        self.received = 0
        self.started = time.monotonic()
        self.handle = None

    @property
    def fraction(self):
        return self.received / self.size if self.size else 0.0

    @property
    def rate_mbit(self):
        elapsed = time.monotonic() - self.started
        if elapsed <= 0:
            return 0.0
        return self.received * 8 / elapsed / 1e6


class Server:
    def __init__(self, fleet, host, port, inbox):
        self.fleet = fleet
        self.host = host
        self.port = port
        self.inbox = inbox
        self.transfers = {}
        os.makedirs(inbox, exist_ok=True)

    async def run(self):
        async with serve(self.handle, self.host, self.port,
                         max_size=protocol.CHUNK_BYTES * 4,
                         ping_interval=20, ping_timeout=20,
                         extensions=[protocol.server_deflate()]) as server:
            await server.serve_forever()

    async def handle(self, socket):
        peer = getattr(socket, "remote_address", None)
        remote = f"{peer[0]}:{peer[1]}" if peer else "?"
        drone = None
        try:
            async for raw in socket:
                if isinstance(raw, bytes):
                    self.on_chunk(drone, raw)
                    continue
                message = protocol.decode(raw)
                if message is None:
                    continue
                if message["t"] == protocol.HELLO:
                    drone = self.fleet.join(
                        message.get("id", "?"), message.get("name", "?"),
                        remote, message.get("color"), message.get("marker"))
                    continue
                if drone is None:
                    continue
                drone.touch(len(raw))
                if message["t"] == protocol.PING:
                    await socket.send(protocol.encode(protocol.PONG))
                elif message["t"] == protocol.ARCHIVE_OFFER:
                    await self.on_offer(socket, drone, message)
                elif message["t"] == protocol.ARCHIVE_DONE:
                    await self.on_done(socket, drone, message)
                else:
                    drone.apply(message)
        except Exception as error:
            if drone is not None:
                self.fleet.note(drone, "link", f"error {type(error).__name__}")
        finally:
            if drone is not None:
                self.close_transfer(drone)
                self.fleet.leave(drone.id)

    async def on_offer(self, socket, drone, message):
        name = os.path.basename(str(message.get("name", "archive.zip")))
        size = int(message.get("size", 0))
        target = os.path.join(self.inbox, drone.tag)
        os.makedirs(target, exist_ok=True)
        path = os.path.join(target, name)
        partial = path + ".part"
        resume = os.path.getsize(partial) if os.path.exists(partial) else 0
        if resume > size:
            resume = 0
        transfer = Incoming(drone, name, size, message.get("sha", ""),
                            message.get("session", ""), path)
        transfer.received = resume
        transfer.handle = open(partial, "r+b" if resume else "wb")
        transfer.handle.seek(resume)
        self.transfers[drone.id] = transfer
        drone.archive = transfer
        self.fleet.note(drone, "archive",
                        f"receiving {name} "
                        f"({size / 1e6:.1f} MB"
                        f"{f', resume at {resume / 1e6:.1f} MB' if resume else ''})")
        await socket.send(protocol.archive_accept(name, resume))

    def on_chunk(self, drone, payload):
        if drone is None:
            return
        transfer = self.transfers.get(drone.id)
        if transfer is None or transfer.handle is None:
            return
        transfer.handle.write(payload)
        transfer.received += len(payload)
        drone.touch(len(payload))

    async def on_done(self, socket, drone, message):
        transfer = self.transfers.pop(drone.id, None)
        if transfer is None:
            await socket.send(protocol.archive_fail(
                message.get("name", "?"), "no transfer in progress"))
            return
        transfer.handle.close()
        transfer.handle = None
        partial = transfer.path + ".part"
        digest = ""
        if transfer.digest:
            hasher = hashlib.sha256()
            with open(partial, "rb") as handle:
                for block in iter(lambda: handle.read(1 << 20), b""):
                    hasher.update(block)
            digest = hasher.hexdigest()
        if transfer.digest and digest != transfer.digest:
            os.remove(partial)
            drone.archive = None
            self.fleet.note(drone, "archive", f"{transfer.name} CHECKSUM FAILED")
            await socket.send(protocol.archive_fail(transfer.name, "checksum"))
            return
        os.replace(partial, transfer.path)
        drone.archive = None
        drone.archives.append({
            "name": transfer.name, "path": transfer.path,
            "size": transfer.received, "mbit_s": transfer.rate_mbit,
            "session": transfer.session,
        })
        self.fleet.note(drone, "archive",
                        f"{transfer.name} complete, "
                        f"{transfer.received / 1e6:.1f} MB at "
                        f"{transfer.rate_mbit:.2f} Mbit/s")
        await socket.send(protocol.archive_ok(transfer.name, transfer.path))

    def close_transfer(self, drone):
        transfer = self.transfers.pop(drone.id, None)
        if transfer is not None and transfer.handle is not None:
            transfer.handle.close()
            self.fleet.note(drone, "archive",
                            f"{transfer.name} interrupted at "
                            f"{transfer.fraction * 100:.0f}%, will resume")
        drone.archive = None

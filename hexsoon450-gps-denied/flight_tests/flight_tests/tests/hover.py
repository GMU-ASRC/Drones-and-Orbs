import time

class HoverTestMixin:
    def test_hover(self, seconds, alt=None):
        if alt is None:
            alt = self.TAKEOFF_ALT
        self.publish_status(f'[TEST] Hover {seconds}s at {alt}m.')
        if not self.pre_flight(takeoff_alt=alt):
            return

        if not self.auto_takeoff(alt):
            self.post_flight()
            return

        self.publish_status('Waiting to reach altitude...')
        if not self.wait_alt(alt):
            self.drone.land()
            self.post_flight()
            return

        self.publish_status(f'Hovering for {seconds}s...')
        self.drone.set_mode('GUIDED')
        start = time.time()
        cx, cy, _ = self.drone.current_xyz()
        self.drone.set_target(cx, cy, alt)
        while not self.abort and (time.time() - start) < seconds:
            time.sleep(1.0 / self.SETPOINT_HZ)

        if self.abort:
            self.publish_status('Test aborted. Landing.')
        else:
            self.publish_status('Hover complete. Landing.')

        self.drone.land()
        self.wait_landed()
        self.publish_status(f'[TEST] Hover {seconds}s at {alt}m complete.')
        self.post_flight()

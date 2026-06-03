import time

class QuickLandTestMixin:
    def test_quick_land(self, alt=None):
        if alt is None:
            alt = self.TAKEOFF_ALT
        self.publish_status(f'[TEST] Quick takeoff to {alt}m and land.')
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

        if self.abort:
            self.publish_status('Test aborted. Landing.')
            self.drone.land()
            self.post_flight()
            return

        self.publish_status('Altitude reached. Landing immediately.')
        self.drone.land()
        self.wait_landed()
        self.publish_status('[TEST] Quick takeoff and land complete.')
        self.post_flight()

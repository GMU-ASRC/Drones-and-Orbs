import time

class MoveReturnTestMixin:
    def test_move_return(self, alt=None, dist=None):
        if alt is None:
            alt = self.TAKEOFF_ALT
        if dist is None:
            dist = self.MOVE_DISTANCE
        self.publish_status(f'[TEST] Move {dist}m at {alt}m and land.')
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

        self.drone.set_mode('GUIDED')
        cx, cy, _ = self.drone.current_xyz()
        origin_x, origin_y = cx, cy
        # Move straight forward (North / +Y axis only) to avoid diagonal drift
        target_x = cx
        target_y = cy + dist

        if self.abort:
            self.publish_status('Test aborted. Landing.')
            self.drone.land()
            self.post_flight()
            return

        self.publish_status(f'Moving to x={target_x:.1f} y={target_y:.1f}...')
        if not self.wait_reach(target_x, target_y, alt):
            self.publish_status('Move failed. Landing.')
            self.drone.land()
            self.post_flight()
            return

        self.publish_status(f'Target reached. Returning to origin x={origin_x:.1f} y={origin_y:.1f}...')
        if not self.wait_reach(origin_x, origin_y, alt):
            self.publish_status('Return failed. Landing.')
            self.drone.land()
            self.post_flight()
            return

        self.publish_status('Origin reached. Landing.')
        self.drone.land()
        self.wait_landed()
        self.publish_status(f'[TEST] Move {dist}m and return complete.')
        self.post_flight()

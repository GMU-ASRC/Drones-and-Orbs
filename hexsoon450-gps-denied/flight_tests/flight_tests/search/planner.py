class MissionPlanner:
    def __init__(self, pattern):
        self.pattern = pattern
        self.waypoints = self.pattern.generate_path()
        self.current_idx = 0

    def get_waypoints(self):
        return self.waypoints

    def has_next(self):
        return self.current_idx < len(self.waypoints)

    def get_next(self):
        if self.has_next():
            wp = self.waypoints[self.current_idx]
            self.current_idx += 1
            return wp
        return None

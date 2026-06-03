from .base import SearchPattern

class SpiralPattern(SearchPattern):
    """
    Inward Rectangular Spiral Search
    Starts at a given corner and spirals inwards to cover the bounding box.
    """
    def __init__(self, start, altitude, area_w_m, area_h_m, lane_spacing=1.0, start_corner='bl', hfov=1.3962634, overlap=0.2):
        super().__init__(start, altitude, hfov, overlap)
        self.area_w_m = area_w_m
        self.area_h_m = area_h_m
        self.lane_spacing = lane_spacing
        self.start_corner = start_corner

    def generate_path(self):
        leg_m = self.lane_spacing if self.lane_spacing else self.calculate_leg_spacing()
        if leg_m < 0.5:
            leg_m = 0.5

        min_x, max_x, min_y, max_y = self.get_bounds(self.area_w_m, self.area_h_m, self.start_corner)
        w = max_x - min_x
        h = max_y - min_y

        # Direction sequence and starting corner for each start_corner option.
        # Each leg alternates between the x-axis (horizontal) and y-axis (vertical).
        # Horizontal leg length = w - reduction, vertical = h - reduction,
        # where reduction grows by leg_m every two legs (every half-revolution).
        # This produces a continuous inward spiral with no ring-closing or diagonal jumps.
        corner_cfg = {
            'bl': {'start': (min_x, min_y), 'dirs': [( 1, 0), ( 0, 1), (-1, 0), ( 0,-1)]},
            'br': {'start': (max_x, min_y), 'dirs': [( 0, 1), (-1, 0), ( 0,-1), ( 1, 0)]},
            'tr': {'start': (max_x, max_y), 'dirs': [(-1, 0), ( 0,-1), ( 1, 0), ( 0, 1)]},
            'tl': {'start': (min_x, max_y), 'dirs': [( 0,-1), ( 1, 0), ( 0, 1), (-1, 0)]},
        }
        cfg = corner_cfg.get(self.start_corner, corner_cfg['bl'])

        x, y = cfg['start']
        dirs = cfg['dirs']
        waypoints = [(x, y)]

        reduction = 0
        dir_idx = 0

        while True:
            dx, dy = dirs[dir_idx % 4]
            leg_len = (w - reduction) if dx != 0 else (h - reduction)

            if leg_len <= 0:
                break

            x += dx * leg_len
            y += dy * leg_len
            waypoints.append((x, y))

            dir_idx += 1
            # Increase reduction every two legs (one horizontal + one vertical = half revolution)
            if dir_idx % 2 == 0:
                reduction += leg_m

        return waypoints
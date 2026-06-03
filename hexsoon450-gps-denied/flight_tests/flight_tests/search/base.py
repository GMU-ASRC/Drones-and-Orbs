import math

class SearchPattern:
    def __init__(self, start, altitude, hfov=1.3962634, overlap=0.2):
        self.start = start
        self.altitude = altitude
        self.hfov = hfov
        self.overlap = overlap

    def calculate_leg_spacing(self):
        return (2.0 * self.altitude * math.tan(self.hfov / 2.0)) * (1.0 - self.overlap)

    def get_bounds(self, area_w_m, area_h_m, start_corner):
        """
        Returns (min_x, max_x, min_y, max_y) bounds depending on which corner
        the start position represents.
        """
        sx, sy = self.start
        
        if start_corner == 'bl':
            return (sx, sx + area_w_m, sy, sy + area_h_m)
        elif start_corner == 'br':
            return (sx - area_w_m, sx, sy, sy + area_h_m)
        elif start_corner == 'tl':
            return (sx, sx + area_w_m, sy - area_h_m, sy)
        elif start_corner == 'tr':
            return (sx - area_w_m, sx, sy - area_h_m, sy)
        else:
            return (sx, sx + area_w_m, sy, sy + area_h_m) # Default to BL

    def generate_path(self):
        raise NotImplementedError

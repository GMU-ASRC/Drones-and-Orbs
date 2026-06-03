from .base import SearchPattern

class LawnmowerPattern(SearchPattern):
    """
    Lawnmower Search Pattern
    Starts at a given corner and sweeps back and forth across the area.
    """
    def __init__(self, start, altitude, area_w_m, area_h_m, lane_spacing=1.0, start_corner='bl', hfov=1.3962634, overlap=0.2):
        super().__init__(start, altitude, hfov, overlap)
        self.area_w_m = area_w_m
        self.area_h_m = area_h_m
        self.lane_spacing = lane_spacing
        self.start_corner = start_corner

    def generate_path(self):
        waypoints = [self.start]
        min_x, max_x, min_y, max_y = self.get_bounds(self.area_w_m, self.area_h_m, self.start_corner)
        
        spacing = self.lane_spacing
        
        # Determine the primary sweeping axis based on corner
        # If starting BL or TL, we sweep along Y, progress along +X
        # If starting BR or TR, we sweep along Y, progress along -X
        
        if self.start_corner in ['bl', 'tl']:
            lane_x = min_x
            x_dir = 1
            y_start = min_y if self.start_corner == 'bl' else max_y
            y_end = max_y if self.start_corner == 'bl' else min_y
        else:
            lane_x = max_x
            x_dir = -1
            y_start = min_y if self.start_corner == 'br' else max_y
            y_end = max_y if self.start_corner == 'br' else min_y

        lane = 0
        
        while min_x <= lane_x <= max_x:
            if lane % 2 == 0:
                current_y = y_start
                target_y = y_end
            else:
                current_y = y_end
                target_y = y_start

            waypoints.append((lane_x, current_y))
            waypoints.append((lane_x, target_y))

            lane += 1
            lane_x += (spacing * x_dir)

        # Ensure we cover the far edge exactly if it wasn't hit precisely by the lane spacing
        far_edge_x = max_x if x_dir == 1 else min_x
        last_lane_x = lane_x - (spacing * x_dir)
        
        if abs(far_edge_x - last_lane_x) > 0.01:
            lane_x = far_edge_x
            if lane % 2 == 0:
                current_y = y_start
                target_y = y_end
            else:
                current_y = y_end
                target_y = y_start
            waypoints.append((lane_x, current_y))
            waypoints.append((lane_x, target_y))

        return waypoints
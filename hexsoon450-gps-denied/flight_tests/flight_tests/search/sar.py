import math
from .base import SearchPattern

class SARPattern(SearchPattern):
    """
    Military Sector Search (SAR) Pattern — IAMSAR Vol. II.

    The drone's start position is the datum (last-known point). The search area
    is divided into 6 equal 60° sectors radiating from the datum. For each sector
    the drone flies outbound along the leading edge, sweeps the far arc, then
    returns to datum before rotating into the next sector. This produces 6
    triangular pie-slice sectors that together cover a full circle.

    start_corner is accepted but ignored — sector search is always datum-centred.
    Search radius = min(area_w, area_h) / 2.
    """

    NUM_SECTORS = 6

    def __init__(self, start, altitude, area_w_m, area_h_m, lane_spacing=1.0,
                 start_corner='bl', hfov=1.3962634, overlap=0.2):
        super().__init__(start, altitude, hfov, overlap)
        self.area_w_m = area_w_m
        self.area_h_m = area_h_m
        self.lane_spacing = lane_spacing

    def generate_path(self):
        cx, cy = self.start
        radius = min(self.area_w_m, self.area_h_m) / 2.0

        sector_angle = 2.0 * math.pi / self.NUM_SECTORS

        # One arc waypoint every ~10° keeps the arc visually smooth
        arc_subdivisions = max(2, round(math.degrees(sector_angle) / 10))

        waypoints = [(cx, cy)]  # Drone starts at datum

        for i in range(self.NUM_SECTORS):
            a_lead  = i * sector_angle                    # leading edge angle
            a_trail = (i + 1) * sector_angle              # trailing edge angle

            # Outbound along leading edge
            waypoints.append((
                cx + radius * math.sin(a_lead),
                cy + radius * math.cos(a_lead),
            ))

            # Sweep far arc from leading edge to trailing edge
            for j in range(1, arc_subdivisions):
                a = a_lead + j * (sector_angle / arc_subdivisions)
                waypoints.append((
                    cx + radius * math.sin(a),
                    cy + radius * math.cos(a),
                ))

            # Trailing edge
            waypoints.append((
                cx + radius * math.sin(a_trail),
                cy + radius * math.cos(a_trail),
            ))

            # Return to datum before next sector
            waypoints.append((cx, cy))

        return waypoints
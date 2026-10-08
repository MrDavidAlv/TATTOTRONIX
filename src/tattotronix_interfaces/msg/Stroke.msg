# One stroke of the pen: a polyline on the work panel, drawn from its first
# point to its last without lifting.
#
# Millimetres on the panel, from its corner nearest the arm's origin: x_mm runs
# along the arm's +x from the panel's edge at x = 0.11 m, y_mm along its +y from
# the edge at y = -0.07 m. The panel is 200 x 140 mm, so a point is in it when
# 0 <= x_mm <= 200 and 0 <= y_mm <= 140. Both arrays have one entry per point.

float64[] x_mm
float64[] y_mm

# circle_gcode_fixed.py
# Generates G-code for a circle (XY -> inverse kinematics -> S/E angles)
# Fixed import/definition/order issues so the file runs on MicroPython / CPython.
#
# Usage:
#  - import this on the Pico and call generate_circle_gcode(...) or run directly
#  - saved file will be written to "circle.gcode"

from math import cos, sin, pi, atan2, acos, degrees, radians, sqrt

# ------------------ translate (from your earlier code) ------------------
def translate(angle):
    """
    Converts an angle in degrees to the corresponding duty_u16 input.
    (Kept for completeness; not required for G-code generation.)
    """
    MIN = 1638  # 0 degrees
    MAX = 8192  # 180 degrees
    DEG = (MAX - MIN) / 180.0
    angle = max(0.0, min(180.0, angle))
    return int(angle * DEG + MIN)

# ------------------ Inverse kinematics (XY -> shoulder, elbow) -----------
def inverse_kinematics(x, y, L1=100.0, L2=100.0):
    """
    Return (alpha_deg, beta_deg) for a 2-link planar arm reaching (x,y).
    This implementation clamps values to avoid domain errors in acos.
    """
    # distance to point
    dist2 = x * x + y * y
    dist = sqrt(dist2)

    # basic reach checks
    if dist < 1e-6:
        # near zero position — return a sensible default
        return 0.0, 0.0
    if dist > (L1 + L2):
        # outside maximum reach — scale point inwards slightly
        scale = (L1 + L2 - 1e-3) / dist
        x *= scale
        y *= scale
        dist = sqrt(x * x + y * y)

    # cosine of elbow angle (law of cosines)
    cos_beta = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2)
    # clamp cos_beta to [-1,1]
    if cos_beta > 1.0:
        cos_beta = 1.0
    elif cos_beta < -1.0:
        cos_beta = -1.0

    beta = acos(cos_beta)  # elbow angle (rad)

    # shoulder angle
    # avoid singularities by computing with atan2 safely
    k1 = L1 + L2 * cos(beta)
    k2 = L2 * sin(beta)
    alpha = atan2(y, x) - atan2(k2, k1)

    return degrees(alpha), degrees(beta)

# ------------------ Forward kinematics (angles -> XY) -------------------
def forward_kinematics(alpha_deg, beta_deg, L1=100.0, L2=100.0):
    """
    Compute end-effector (x,y) from joint angles in degrees.
    Useful for simulation / verification.
    """
    a = radians(alpha_deg)
    b = radians(beta_deg)
    x = L1 * cos(a) + L2 * cos(a + b)
    y = L1 * sin(a) + L2 * sin(a + b)
    return x, y

# ------------------ Generate circle G-code ------------------------------
def generate_circle_gcode(center_x=80.0, center_y=0.0, radius=50.0,
                          steps=72, L1=100.0, L2=100.0):
    """
    Generate G-code commands to draw a circle in XY space.
    Returns a list of strings (G-code lines).
    """
    if steps < 4:
        steps = 4

    gcode = []
    gcode.append("M5")  # pen up

    # sample points around circle (closed loop)
    points_angles = []
    for i in range(steps + 1):  # +1 to close the loop
        theta = 2.0 * pi * float(i) / float(steps)
        x = center_x + radius * cos(theta)
        y = center_y + radius * sin(theta)

        # compute ik; inverse_kinematics clamps unreachable points
        a_deg, b_deg = inverse_kinematics(x, y, L1=L1, L2=L2)
        points_angles.append((a_deg, b_deg))

    # move to first point and lower pen
    first_a, first_b = points_angles[0]
    gcode.append("G1 S{:.4f} E{:.4f}".format(first_a, first_b))
    gcode.append("M3")  # pen down

    # subsequent points
    for (a_deg, b_deg) in points_angles[1:]:
        gcode.append("G1 S{:.4f} E{:.4f}".format(a_deg, b_deg))

    gcode.append("M5")   # pen up
    gcode.append("M18")  # disable servos
    return gcode

# ------------------ Save G-code to file --------------------------------
def save_gcode_to_file(gcode_lines, filename="circle.gcode"):
    """
    Save the G-code list to a file on the Pico filesystem (or local FS).
    """
    try:
        with open(filename, "w") as f:
            for line in gcode_lines:
                f.write(line + "\n")
        print("Saved G-code to '{}' ({} lines)".format(filename, len(gcode_lines)))
    except Exception as e:
        print("Error saving G-code:", e)

# ------------------ Simulate: parse G-code -> XY points -----------------
def simulate_gcode_xy(gcode_lines, L1=100.0, L2=100.0):
    """
    Convert G-code (G1 S.. E..) into XY points using forward kinematics.
    This avoids regex to be compatible with MicroPython.
    """
    pts = []
    for line in gcode_lines:
        line = line.strip()
        if not line:
            continue
        if not line.startswith("G1"):
            continue
        # naive token parse: split by spaces and look for tokens starting with S/E
        parts = line.split()
        s_val = None
        e_val = None
        for p in parts[1:]:
            if p[0] in ("S", "s") and len(p) > 1:
                # value may be right after letter, possibly with sign
                try:
                    s_val = float(p[1:])
                except:
                    s_val = None
            elif p[0] in ("E", "e") and len(p) > 1:
                try:
                    e_val = float(p[1:])
                except:
                    e_val = None
        if s_val is not None and e_val is not None:
            x, y = forward_kinematics(s_val, e_val, L1=L1, L2=L2)
            pts.append((x, y))
    return pts

# ------------------ Example main for direct run -------------------------
if __name__ == "__main__":
    # Parameters you can adjust
    CX = 80.0
    CY = 0.0
    R = 50.0
    STEPS = 180
    L1 = 100.0
    L2 = 100.0

    # generate and save
    g = generate_circle_gcode(center_x=CX, center_y=CY, radius=R,
                              steps=STEPS, L1=L1, L2=L2)
    save_gcode_to_file(g, "circle.gcode")

    # simulation / quick check (only on host that can plot)
    try:
        pts = simulate_gcode_xy(g, L1=L1, L2=L2)
        print("Simulated {} XY points (first 5):".format(len(pts)))
        for p in pts[:180]:
            print("  x={:.2f}, y={:.2f}".format(p[0], p[1]))
    except Exception as e:  ccc
        print("Simulation skipped (error):", e)

    print("Done.")

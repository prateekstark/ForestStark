"""Safety logic: clustering, tracking, and the stop / slow / hold / reroute decisions."""
import math

import numpy as np
from fs_autonomy import safety

JAAL = safety.SafetyConfig.for_robot("jaal")


def track(x, y, r=0.18, moving=False, vx=0.0, vy=0.0, tid=1, kind="object"):
    t = safety.Track(tid, x, y, r, 0.0, 0.0, x, y, vx, vy, moving, kind)
    return (t, x, y, vx, vy)


def test_scan_points_and_legs_cluster_as_one_person():
    # a lidar 3 m from two legs 0.2 m apart: one cluster of person size
    ang = np.linspace(-0.2, 0.2, 81)
    r = np.full_like(ang, np.inf)
    for leg in (-0.1, 0.1):
        hit = np.abs(3.0 * np.tan(ang) - leg) < 0.075
        r[hit] = 3.0
    pts = safety.scan_points(r, ang[0], ang[1] - ang[0], 0.3, 20.0)
    det = safety.clusters([pts])
    assert len(det) == 1
    assert abs(det[0][0] - 3.0) < 0.05 and det[0][2] < 0.3


def test_same_object_from_two_lidars_merges():
    a = np.array([[2.0, 0.0], [2.0, 0.1], [2.0, 0.2]])
    b = np.array([[2.05, 0.05], [2.05, 0.15]])
    assert len(safety.clusters([a, b])) == 1


def test_tracker_marks_walker_moving_and_stump_static():
    tr = safety.Tracker()
    for k in range(30):                         # 3 s: walker at 1.2 m/s, stump still
        t = k * 0.1
        tr.update(t, [(5.0, -2.0 + 1.2 * t, 0.18), (10.0, 0.0, 0.25)])
    walker = min(tr.tracks, key=lambda x: abs(x.x - 5.0))
    stump = min(tr.tracks, key=lambda x: abs(x.x - 10.0))
    assert walker.moving and abs(walker.vy - 1.2) < 0.2
    assert not stump.moving
    assert len(tr.tracks) == 2


def test_crate_seen_from_a_passing_robot_stays_static():
    # the visible face changes as the robot passes: the centroid drifts 0.45 m at up to 0.7 m/s, then settles
    tr = safety.Tracker()
    for k in range(40):
        t = k * 0.1
        wobble = 0.45 * math.sin(min(t, 1.0) * math.pi / 2)        # up to 0.7 m/s for a second
        tr.update(t, [(6.0 + wobble, 2.2, 0.4)])
    assert not tr.tracks[0].moving
    tr = safety.Tracker()
    for k in range(30):                          # a large object is never a person, even when it moves
        tr.update(k * 0.1, [(6.0 + 0.1 * k, 2.2, 0.7)])
    assert not tr.tracks[0].moving


def test_driving_past_a_crate_does_not_make_it_a_person():
    tr = safety.Tracker()
    for k in range(30):                          # robot passes at 1 m/s; the crate's centre drifts 0.7 m
        t = k * 0.1
        tr.update(t, [(5.0 + 0.7 * min(t, 2.0) / 2.0, 2.4, 0.3)], robot=(3.0 + t, 0.0))
    assert not tr.tracks[0].moving
    tr = safety.Tracker()
    for k in range(30):                          # a person walking at 1.3 m/s past the same robot does count
        t = k * 0.1
        tr.update(t, [(5.0, 3.0 - 1.3 * t, 0.18)], robot=(3.0 + t, 0.0))
    assert tr.tracks[0].moving


def test_pivoting_counts_as_ego_motion():
    tr = safety.Tracker()
    for k in range(30):                          # robot pivots in place; a sapling 4 m away drifts 0.7 m
        t = k * 0.1
        tr.update(t, [(4.0, 0.35 * min(t, 2.0), 0.08)], robot=(0.0, 0.0, 1.5 * t))
    assert not tr.tracks[0].moving


def test_a_track_that_jumps_is_not_a_walker():
    tr = safety.Tracker()
    for k in range(40):                          # one wrong association halfway: +0.7 m in 0.1 s, then stays
        tr.update(k * 0.1, [(6.0 + (0.7 if k >= 20 else 0.0), 2.2, 0.3)])
    assert not tr.tracks[0].moving


def test_a_surveyed_stump_never_becomes_a_person_but_a_walker_near_it_does():
    tr = safety.Tracker(static_zones=[(12.8, 3.4, 0.3)])
    for k in range(30):                          # the stump's track drifts 1 m as the robot passes
        tr.update(k * 0.1, [(12.8 - 0.035 * k, 3.4, 0.25), (8.0 + 1.2 * k * 0.1, 3.0, 0.18)])
    stump = min(tr.tracks, key=lambda x: abs(x.y - 3.4))
    walker = min(tr.tracks, key=lambda x: abs(x.y - 3.0))
    assert stump.kind == "surveyed" and not stump.moving
    assert walker.moving and walker.kind == "object"           # walked to within a metre of the stump


def test_tracker_labels_fleet_peers():
    tr = safety.Tracker()
    for k in range(20):
        tr.update(k * 0.1, [(4.0 + 0.1 * k, 0.0, 0.2)], peers=[(4.0 + 0.1 * k, 0.5)])
    assert tr.tracks[0].kind == "robot"


def test_person_ahead_stops_static_ahead_slows():
    A = safety.assess([track(1.45 + 2.0, 0.0, moving=True)], (1.0, 0.0, 0.0), JAAL)
    assert A.scale == 0.0 and A.reason == "person_in_path"
    A = safety.assess([track(1.45 + 3.0, 0.0)], (1.0, 0.0, 0.0), JAAL)
    assert 0.2 < A.scale < 1.0 and A.reason == "obstacle"
    A = safety.assess([track(1.45 + 1.0, 0.0)], (1.0, 0.0, 0.0), JAAL)
    assert A.scale == 0.0 and A.reason == "obstacle"


def test_static_stop_distance_shrinks_at_crawl_speed():
    obstacle = [track(1.45 + 1.0 + 0.18, 0.0)]                   # 1.0 m gap
    assert safety.assess(obstacle, (1.0, 0.0, 0.0), JAAL).scale == 0.0
    assert safety.assess(obstacle, (0.3, 0.0, 0.0), JAAL).scale > 0.0
    person = [track(1.45 + 2.0 + 0.18, 0.0, moving=True)]
    assert safety.assess(person, (0.3, 0.0, 0.0), JAAL).scale == 0.0      # people keep the full 2.5 m


def test_object_behind_or_beside_the_corridor_is_ignored():
    A = safety.assess([track(-4.0, 0.0), track(3.0, 2.5)], (1.0, 0.0, 0.0), JAAL)
    assert A.scale == 1.0


def test_person_crossing_is_predicted():
    # 6 m ahead, 3 m to the left, walking right at 1.3 m/s: reaches the path in ~2 s
    A = safety.assess([track(6.0, 3.0, moving=True, vy=-1.3)], (1.0, 0.0, 0.0), JAAL)
    assert A.scale == 0.0 and A.reason == "person_crossing"
    # the same person walking away from the path is only a slow-down at most
    A = safety.assess([track(6.0, 3.0, moving=True, vy=1.3)], (1.0, 0.0, 0.0), JAAL)
    assert A.scale > 0.0


def test_person_close_stops_even_when_stationary_or_pivoting():
    A = safety.assess([track(0.0, 1.5, moving=True)], (0.0, 0.0, 0.0), JAAL)
    assert A.scale == 0.0 and A.reason == "person_near"
    A = safety.assess([track(0.0, 2.2, moving=True)], (0.0, 0.0, 0.4), JAAL)     # inside the swept circle + 0.5
    assert A.scale == 0.0


def test_tool_zone_holds_for_people_not_saplings():
    tool = (-1.65, 0.0)
    A = safety.assess([track(-3.0, 0.5, moving=True)], (0.0, 0.0, 0.0), JAAL, tool)
    assert A.tool_hold
    A = safety.assess([track(-1.65, 0.0)], (0.0, 0.0, 0.0), JAAL, tool)           # the sapling it just planted
    assert not A.tool_hold
    A = safety.assess([track(-6.0, 0.0, moving=True)], (0.0, 0.0, 0.0), JAAL, tool)
    assert not A.tool_hold


def test_detour_sides_report_what_is_in_the_way():
    crate = track(2.6, 0.0, r=0.4, tid=1)
    A = safety.assess([crate], (1.0, 0.0, 0.0), JAAL)
    assert A.scale == 0.0 and A.free_left and A.free_right
    A = safety.assess([crate, track(3.5, 1.9, tid=2)], (1.0, 0.0, 0.0), JAAL)     # something on the left lane
    assert A.free_right and not A.free_left


def test_katar_ignores_static_roadside_but_stops_for_people_in_the_boom_strip():
    cfg = safety.SafetyConfig.for_robot("katar")
    A = safety.assess([track(3.0, 0.0)], (0.7, 0.0, 0.0), cfg)
    assert A.scale == 1.0
    A = safety.assess([track(-2.0, 4.2, moving=True)], (0.7, 0.0, 0.0), cfg)    # walking beside the boom
    assert A.scale == 0.0 and A.reason == "person_near"


def test_to_base_rotates_into_the_robot_frame():
    t = safety.Track(1, 1.0, 2.0, 0.2, 0, 0, 1.0, 2.0, vx=0.0, vy=1.0)
    (_, bx, by, bvx, bvy), = safety.to_base([t], (1.0, 0.0, math.pi / 2))
    assert abs(bx - 2.0) < 1e-9 and abs(by) < 1e-9 and abs(bvx - 1.0) < 1e-9


def test_ground_hits_on_a_slope_are_dropped():
    # rolled with a 14 deg slope the lidar plane runs parallel to the hillside, so what it hits is
    # terrain rising above the plane: here a 0.75 m terrace step 2 m uphill
    slope = math.tan(math.radians(14))
    elev = lambda x, y: np.asarray(y, float) * slope + np.where(np.asarray(y, float) > 2.0, 0.75, 0.0)
    roll = math.radians(14)
    pts = np.array([[0.0, 2.8, 0.5],     # on the terrace face
                    [6.0, 0.0, 0.5],     # something standing on the contour ahead
                    [0.0, -4.0, 0.5]])   # downhill: the plane is well above the ground
    keep = safety.above_ground(pts, roll, 0.0, 0.0, (0.0, 0.0), 0.21, elev)
    assert keep.tolist() == [False, True, True]

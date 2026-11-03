"""Checks for painted-line coverage and body intrusion on the connected map."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from safe_drive_safety.connected_lane_geometry import check_lane_lines


MARKINGS = json.loads((
    Path(__file__).resolve().parents[2] / 'safe_drive_sim/config'
    / 'connected_road_markings.json').read_text())
WORLD = (Path(__file__).resolve().parents[2] /
         'safe_drive_sim/worlds/connected_road.world')


def check(x, y, yaw):
    return check_lane_lines(x, y, yaw, 4.2, 1.8, MARKINGS)


def test_eastbound_lane_center_is_clear():
    result = check(-15.0, -1.75, 0.0)
    assert result.road == 'main'
    assert result.direction == 'east'
    assert result.checked_corners == 4
    assert result.centerline_overlap_m == 0.0
    assert result.edge_line_overlap_m == 0.0


def test_eastbound_body_on_centerline_is_detected():
    result = check(-15.0, -0.7, 0.0)
    assert result.centerline_overlap_m == pytest.approx(0.26)
    assert result.edge_line_overlap_m == 0.0


def test_northbound_centerline_and_outer_edge_are_separate():
    clear = check(-34.25, -30.0, math.pi / 2)
    center = check(-35.5, -30.0, math.pi / 2)
    edge = check(-33.0, -30.0, math.pi / 2)
    assert clear.checked_corners == 4
    assert clear.centerline_overlap_m == 0.0
    assert center.centerline_overlap_m > 0.0
    assert center.edge_line_overlap_m == 0.0
    assert edge.edge_line_overlap_m > 0.0
    assert edge.centerline_overlap_m == 0.0


def test_unpainted_junction_does_not_invent_a_centerline():
    result = check(-36.0, -56.0, 2.2)
    assert result.checked_corners == 0
    assert result.centerline_overlap_m == 0.0
    assert result.edge_line_overlap_m == 0.0


def test_marking_inventory_matches_generated_world_paint():
    world = ET.parse(WORLD).getroot()
    visuals = world.findall('.//visual')
    for road in MARKINGS['roads']:
        painted = []
        for visual in visuals:
            if not visual.get('name', '').startswith(
                    road['name'] + '_center_'):
                continue
            pose = [float(value) for value in visual.findtext('pose').split()]
            size = [float(value) for value in
                    visual.findtext('./geometry/box/size').split()]
            coordinate = pose[0] if road['axis'] == 'x' else pose[1]
            span = size[0] if road['axis'] == 'x' else size[1]
            painted.append((coordinate - span / 2, coordinate + span / 2))
        assert len(painted) == len(road['intervals'])
        for actual, expected in zip(sorted(painted),
                                    sorted(road['intervals'])):
            assert actual == pytest.approx(expected)

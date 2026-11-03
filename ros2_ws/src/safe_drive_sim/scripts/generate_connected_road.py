"""
Build a compact, connected road grid around the validated signal scene.

Run from any directory. The source intersection world is left untouched.
"""

import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'worlds/signal_intersection.world'
OUTPUT = ROOT / 'worlds/connected_road.world'
MARKINGS = ROOT / 'config/connected_road_markings.json'
ASPHALT = ('file://safe_drive_road/materials/scripts',
           'file://safe_drive_road/materials/textures', 'SafeDrive/Asphalt')
GRASS = ('file://safe_drive_road/materials/scripts',
         'file://safe_drive_road/materials/textures', 'SafeDrive/Grass')
YELLOW = '0.95 0.85 0.10 1'
WHITE = '0.95 0.95 0.95 1'


def box(parent, kind, name, x, y, length, width, z=-0.05, height=0.1,
        color=None, texture=None):
    element = ET.SubElement(parent, kind, name=name)
    ET.SubElement(element, 'pose').text = f'{x:g} {y:g} {z:g} 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'box'),
                  'size').text = f'{length:g} {width:g} {height:g}'
    if kind == 'visual':
        material = ET.SubElement(element, 'material')
        if texture:
            script = ET.SubElement(material, 'script')
            for uri in texture[:2]:
                ET.SubElement(script, 'uri').text = uri
            ET.SubElement(script, 'name').text = texture[2]
        else:
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color


def road(link, name, x, y, length, width):
    box(link, 'collision', name + '_collision', x, y, length, width)
    box(link, 'visual', name + '_asphalt', x, y, length, width,
        z=-0.045, height=0.01, texture=ASPHALT)


def paint(link, name, x, y, length, width, color=WHITE):
    box(link, 'visual', name, x, y, length, width,
        z=-0.028, height=0.012, color=color)


def horizontal_markings(link, name, y, gaps):
    # Gaps around junctions leave the centerline out of turning space.
    points = [-40.0] + [value for gap in gaps for value in gap] + [40.0]
    intervals = list(zip(points[::2], points[1::2]))
    for index, (start, end) in enumerate(intervals):
        if end <= start:
            continue
        paint(link, f'{name}_center_{index}', (start + end) / 2, y,
              end - start, 0.12, YELLOW)
        for side in (-3.75, 3.75):
            paint(link, f'{name}_edge_{index}_{side:g}',
                  (start + end) / 2, y + side, end - start, 0.1)
    return [(start, end) for start, end in intervals if end > start]


def vertical_markings(link, name, x, gaps):
    points = [-60.0] + [value for gap in gaps for value in gap] + [60.0]
    intervals = list(zip(points[::2], points[1::2]))
    for index, (start, end) in enumerate(intervals):
        if end <= start:
            continue
        paint(link, f'{name}_center_{index}', x, (start + end) / 2,
              0.12, end - start, YELLOW)
        for side in (-3.75, 3.75):
            paint(link, f'{name}_edge_{index}_{side:g}',
                  x + side, (start + end) / 2, 0.1, end - start)
    return [(start, end) for start, end in intervals if end > start]


def main():
    marked_roads = []
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', 'safe_drive_connected_road')
    road_link = world.find("./model[@name='road']/link")
    for element in list(road_link):
        name = element.get('name', '')
        if name == 'collision':
            element.find('./pose').text = '0 0 -0.05 0 0 0'
            element.find('./geometry/box/size').text = '80 8 0.1'
        elif name == 'asphalt':
            element.find('./pose').text = '0 0 -0.045 0 0 0'
            element.find('./geometry/box/size').text = '80 8 0.01'
        elif name.startswith(('center_line_', 'left_edge_line_',
                              'right_edge_line_')):
            road_link.remove(element)
    intervals = horizontal_markings(
        road_link, 'main', 0,
        [(-40, -32), (-2, 6), (25, 29), (32, 40)])
    marked_roads.append({'name': 'main', 'axis': 'x', 'center': 0,
                         'intervals': intervals})
    # An unsignalized pedestrian crossing before the east-side right turn.
    # The yield line uses the same 2.3 m approach gap as the central crossing.
    for index, y in enumerate((-3.45, -2.45, -1.45, -0.45,
                               0.55, 1.55, 2.55, 3.55), start=1):
        paint(road_link, f'outer_crosswalk_bar_{index}', 27, y, 3.6, 0.50)
    paint(road_link, 'outer_eastbound_yield_line',
          22.75, -1.75, 0.30, 3.30)

    cross_link = world.find("./model[@name='cross_road']/link")
    for element in list(cross_link):
        if element.get('name', '').endswith(('center_line', 'edge_line')) or \
                '_edge_line' in element.get('name', ''):
            cross_link.remove(element)
    road(cross_link, 'north_extension', 2, 41, 8, 38)
    road(cross_link, 'south_extension', 2, -51, 8, 18)
    intervals = vertical_markings(cross_link, 'central', 2,
                                  [(-60, -52), (-4, 4), (52, 60)])
    marked_roads.append({'name': 'central', 'axis': 'y', 'center': 2,
                         'intervals': intervals})

    # Long sidewalks in the old scene would cross the two new side streets.
    # Keep the crossing entries, central signal fixtures, furniture and houses.
    roadside = world.find("./model[@name='roadside']/link")
    for element in list(roadside):
        name = element.get('name', '')
        if (name.endswith(('_west', '_east')) and
                name.startswith(('north_grass', 'south_grass',
                                 'north_sidewalk', 'south_sidewalk',
                                 'north_curb', 'south_curb'))):
            roadside.remove(element)

    ring = ET.SubElement(world, 'model', name='connected_road_grid')
    ET.SubElement(ring, 'static').text = 'true'
    link = ET.SubElement(ring, 'link', name='road_surfaces')
    box(link, 'visual', 'grass_base', 0, 0, 100, 140,
        z=-0.105, height=0.1, texture=GRASS)
    for y, name in [(-56, 'south'), (56, 'north')]:
        road(link, name, 0, y, 80, 8)
        intervals = horizontal_markings(link, name, y,
                                        [(-40, -32), (-2, 6), (32, 40)])
        marked_roads.append({'name': name, 'axis': 'x', 'center': y,
                             'intervals': intervals})
    for x, name in [(-36, 'west'), (36, 'east')]:
        road(link, name, x, 0, 8, 120)
        intervals = vertical_markings(link, name, x,
                                      [(-60, -52), (-4, 4), (52, 60)])
        marked_roads.append({'name': name, 'axis': 'y', 'center': x,
                             'intervals': intervals})

    # Visual-only pavements stop before each junction. They cannot create a
    # hidden collision barrier across a vehicle turn.
    for y, row in [(-56, 'south'), (0, 'middle'), (56, 'north')]:
        for start, end in [(-32, -8), (10, 32)]:
            for side in (-4.9, 4.9):
                box(link, 'visual', f'{row}_sidewalk_{start}_{side:g}',
                    (start + end) / 2, y + side, end - start, 1.8,
                    z=-0.015, color='0.62 0.62 0.59 1')
    for x, column in [(-36, 'west'), (2, 'middle'), (36, 'east')]:
        for start, end in [(-52, -8), (8, 52)]:
            for side in (-4.9, 4.9):
                box(link, 'visual', f'{column}_sidewalk_{start}_{side:g}',
                    x + side, (start + end) / 2, 1.8, end - start,
                    z=-0.015, color='0.62 0.62 0.59 1')

    # Road paint at the original intersection is kept, including its measured
    # stop line and crosswalk. Other junctions are currently unsignalized.
    ET.indent(tree, space='  ')
    OUTPUT.write_text('<?xml version="1.0" ?>\n' +
                      ET.tostring(tree.getroot(), encoding='unicode') + '\n')
    MARKINGS.parent.mkdir(parents=True, exist_ok=True)
    MARKINGS.write_text(json.dumps({
        'schema_version': 1,
        'source_world': OUTPUT.name,
        'lane_edge_offset_m': 3.75,
        'centerline_width_m': 0.12,
        'edge_line_width_m': 0.10,
        'roads': marked_roads,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()

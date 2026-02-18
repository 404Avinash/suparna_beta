"""
SUPARNA — KMZ & Mission Report Export
Post-mission deliverables: KMZ (Google Earth) + Coverage Report (JSON).

Generates:
  - KMZ file with flight path polyline, loiter circles, obstacle markers
  - Coverage report with energy breakdown, phase timing, statistics
"""

import json
import math
import os
import zipfile
from typing import List, Optional
from xml.etree.ElementTree import Element, SubElement, tostring

from ..core.geometry import Point


def _kml_placemark(name: str, desc: str, coords: List[dict], style: str = 'path') -> Element:
    """Create a KML Placemark element."""
    pm = Element('Placemark')
    SubElement(pm, 'name').text = name
    SubElement(pm, 'description').text = desc
    SubElement(pm, 'styleUrl').text = f'#{style}'

    if len(coords) == 1:
        # Point
        point = SubElement(pm, 'Point')
        c = coords[0]
        SubElement(point, 'coordinates').text = f"{c.get('lon',0)},{c.get('lat',0)},{c.get('alt',0)}"
    else:
        # LineString
        ls = SubElement(pm, 'LineString')
        SubElement(ls, 'altitudeMode').text = 'relativeToGround'
        coord_str = ' '.join(
            f"{c.get('lon',0)},{c.get('lat',0)},{c.get('alt',0)}"
            for c in coords
        )
        SubElement(ls, 'coordinates').text = coord_str

    return pm


def _create_style(doc: Element, style_id: str, color: str, width: int = 3):
    """Add a line style to the KML document."""
    style = SubElement(doc, 'Style', id=style_id)
    ls = SubElement(style, 'LineStyle')
    SubElement(ls, 'color').text = color
    SubElement(ls, 'width').text = str(width)


def export_kmz(
    mission_data: dict,
    output_path: str,
    origin_lat: float = 34.3,
    origin_lon: float = 78.1,
    meters_per_deg: float = 111000.0,
) -> str:
    """
    Export mission data as a KMZ file (zipped KML).

    Since our simulation uses local XY coordinates, we convert to lat/lon
    using a simple offset from an origin point (default: Ladakh region).

    Args:
        mission_data: The mission JSON data dict
        output_path: Path to write the .kmz file
        origin_lat: Latitude of map origin
        origin_lon: Longitude of map origin
        meters_per_deg: Approximate meters per degree

    Returns:
        Path to the generated KMZ file
    """
    kml = Element('kml', xmlns='http://www.opengis.net/kml/2.2')
    doc = SubElement(kml, 'Document')
    SubElement(doc, 'name').text = 'SUPARNA Mission'
    SubElement(doc, 'description').text = 'SUPARNA PCCE Mission Export'

    # Styles
    _create_style(doc, 'flightpath', 'ff0000ff', 3)      # Red — flight path
    _create_style(doc, 'loiter', 'ff00ff00', 2)           # Green — loiter circles
    _create_style(doc, 'obstacle', 'ff0000ff', 2)         # Blue — obstacles
    _create_style(doc, 'descent', 'ff00ffff', 3)          # Cyan — descent

    def xy_to_latlon(x: float, y: float, alt: float = 0) -> dict:
        return {
            'lat': origin_lat + y / meters_per_deg,
            'lon': origin_lon + x / meters_per_deg,
            'alt': alt,
        }

    # Flight path
    waypoints = mission_data.get('waypoints', [])
    if waypoints:
        path_coords = [xy_to_latlon(w['x'], w['y'], 150) for w in waypoints]
        doc.append(_kml_placemark('Flight Path', 'SUPARNA mission path', path_coords, 'flightpath'))

    # Home base
    home = mission_data.get('home', mission_data.get('map', {}).get('start', {}))
    if home:
        hx = home.get('x', 0)
        hy = home.get('y', 0)
        doc.append(_kml_placemark(
            'Home Base (FOB)',
            'Indian Forward Operating Base',
            [xy_to_latlon(hx, hy, 0)],
            'loiter',
        ))

    # Loiter zones
    loiters = mission_data.get('loiters', [])
    for i, loiter in enumerate(loiters):
        cx, cy = loiter['x'], loiter['y']
        r = loiter.get('radius', 55)
        # Create circle approximation
        circle_coords = []
        for a in range(0, 361, 15):
            angle = math.radians(a)
            lx = cx + r * math.cos(angle)
            ly = cy + r * math.sin(angle)
            circle_coords.append(xy_to_latlon(lx, ly, 150))
        doc.append(_kml_placemark(f'Loiter {i+1}', f'Observation zone {i+1}', circle_coords, 'loiter'))

    # Obstacles
    obstacles = mission_data.get('obstacles', [])
    for obs in obstacles:
        doc.append(_kml_placemark(
            obs.get('name', 'Obstacle'),
            f"Radius: {obs.get('r', 0)}m",
            [xy_to_latlon(obs['x'], obs['y'], 0)],
            'obstacle',
        ))

    # Descent path
    descent = mission_data.get('descent', {})
    descent_wps = descent.get('waypoints', [])
    if descent_wps:
        desc_coords = [xy_to_latlon(w['x'], w['y'], w.get('alt', 0)) for w in descent_wps]
        doc.append(_kml_placemark('Loiter-to-Land', 'Spiral descent path', desc_coords, 'descent'))

    # Write KMZ
    kml_bytes = b'<?xml version="1.0" encoding="UTF-8"?>\n' + tostring(kml, encoding='unicode').encode('utf-8')

    kmz_path = output_path if output_path.endswith('.kmz') else output_path + '.kmz'
    with zipfile.ZipFile(kmz_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        zf.writestr('doc.kml', kml_bytes)

    return kmz_path


def export_report(
    mission_data: dict,
    energy_budget: Optional[dict] = None,
    output_path: str = 'mission_report.json',
) -> str:
    """
    Export a comprehensive mission report as JSON.

    Args:
        mission_data: The mission JSON data dict
        energy_budget: Energy budget breakdown dict
        output_path: Path to write the report

    Returns:
        Path to the generated report
    """
    report = {
        'title': 'SUPARNA Mission Report',
        'platform': 'SUPARNA Fixed-Wing ISR UAS',
        'engine': 'Physics-Constrained Coverage Engine (PCCE)',
        'map': {
            'width_m': mission_data.get('map', {}).get('width', 0),
            'height_m': mission_data.get('map', {}).get('height', 0),
            'type': mission_data.get('map', {}).get('type', 'random'),
        },
        'coverage': {
            'loiter_count': len(mission_data.get('loiters', [])),
            'obstacle_count': len(mission_data.get('obstacles', [])),
            'waypoint_count': len(mission_data.get('waypoints', [])),
        },
        'algorithms_used': [
            'Greedy Set Cover (Coverage Planner)',
            'Dubins Curves — all 6 types, closed-form O(1) (Transition Planner)',
            'A* with obstacle inflation O(V log V) (Pathfinder)',
            'Bug2 7-ray raycasting O(r) (Reactive Avoidance)',
            'Nearest-Neighbour TSP O(k²) (Loiter Sequencer)',
            'ISA Atmosphere Correction (High-Altitude Ops)',
            'Loiter-to-Land Spiral Descent (Landing)',
        ],
    }

    if energy_budget:
        report['energy'] = energy_budget

    descent = mission_data.get('descent', {})
    if descent:
        report['landing'] = {
            'type': 'Loiter-to-Land Spiral Descent',
            'loops': descent.get('n_loops', 0),
            'start_altitude_m': descent.get('start_altitude_m', 0),
            'energy_wh': descent.get('energy_wh', 0),
        }

    with open(output_path, 'w') as f:
        json.dump(report, f, indent=2)

    return output_path

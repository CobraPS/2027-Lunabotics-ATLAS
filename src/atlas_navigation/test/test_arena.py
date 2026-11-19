"""Keep the generated arena, its terrain collision, and installation assets coherent."""

from pathlib import Path
import runpy
import struct
import xml.etree.ElementTree as ET
import zlib


PACKAGE = Path(__file__).resolve().parents[1]


def test_arena_contains_physical_terrain_and_obstacles():
    """Craters belong to terrain collision, with no flat collision plane covering them."""
    world = ET.parse(PACKAGE / 'worlds/lunabotics_arena.sdf').find('world')
    assert world.get('name') == 'lunabotics_arena'
    assert world.findtext('gravity') == '0 0 -9.8'
    model = world.find("model[@name='regolith']")
    collision = model.find('link/collision/geometry/heightmap')
    visual = model.find('link/visual/geometry/heightmap')
    assert collision.findtext('uri') == visual.findtext('uri')
    assert [float(v) for v in collision.findtext('size').split()] == [6.8, 5.0, .30]
    # Harmonic's renderer ignores parent transforms for heightmaps; DART does not.
    assert model.find('pose') is None
    assert model.findtext('link/collision/pose') == visual.findtext('pos') + ' 0 0 0'
    assert collision.findtext('pos') == '0 0 0'
    assert not world.findall('.//collision/geometry/plane')
    assert len([m for m in world.findall('model') if m.get('name').startswith('boulder_')]) == 3
    assert world.find("model[@name='central_column']/link/collision") is not None


def png_pixels(path):
    """Compare decoded image data across zlib versions rather than compressed bytes."""
    data = path.read_bytes()
    assert data.startswith(b'\x89PNG\r\n\x1a\n')
    offset, compressed = 8, b''
    while offset < len(data):
        size = struct.unpack('>I', data[offset:offset + 4])[0]
        tag, payload = data[offset + 4:offset + 8], data[offset + 8:offset + 8 + size]
        if tag == b'IHDR':
            width, height = struct.unpack('>II', payload[:8])
        if tag == b'IDAT':
            compressed += payload
        offset += size + 12
    return width, height, zlib.decompress(compressed)


def test_crater_pixels_are_below_the_starting_surface():
    """The installed image encodes negative depressions rather than raised visual disks."""
    width, height, pixels = png_pixels(PACKAGE / 'worlds/terrain.png')

    def elevation(x, y):
        column, row = round(x / 6.8 * (width - 1)), round((5 - y) / 5 * (height - 1))
        return pixels[row * (width + 1) + column + 1] / 255 * .30 - .30

    # Starting zone (1, 1) is near the reference surface
    assert abs(elevation(1, 1)) < .004
    # Craters appear as depressions (negative elevation) along the right edge
    for y in [1.4, 2.8, 4.3]:
        assert elevation(6.35, y) < -0.05


def test_arena_assets_match_generator(tmp_path):
    """A geometry edit must update every checked-in generated asset."""
    generator = runpy.run_path(str(PACKAGE / 'tools/generate_arena.py'))
    generator['generate'](tmp_path)
    for path in tmp_path.iterdir():
        actual = PACKAGE / 'worlds' / path.name
        if path.suffix == '.png':
            assert png_pixels(path) == png_pixels(actual), path.name
        else:
            assert path.read_bytes() == actual.read_bytes(), path.name


def test_boulder_faces_supply_normals_for_dart_collision():
    """DART rejects meshes without a normal for each face vertex."""
    lines = (PACKAGE / 'worlds/boulder.obj').read_text().splitlines()
    normals = [line for line in lines if line.startswith('vn ')]
    assert normals
    for line in lines:
        if line.startswith('f '):
            assert all('//' in vertex and 0 < int(vertex.split('//')[1]) <= len(normals)
                       for vertex in line.split()[1:])

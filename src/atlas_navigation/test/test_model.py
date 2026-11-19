"""Check the provisional rover's mechanical and drivetrain contracts."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


MODEL_DIR = Path(__file__).resolve().parents[1] / 'models' / 'atlas'


@pytest.fixture
def model():
    """Read the same standalone SDF that the launch file spawns."""
    return ET.fromstring((MODEL_DIR / 'model.sdf').read_text().lstrip()).find('model')


def test_four_driven_wheels(model):
    """All four wheels must be connected and driven on the correct side."""
    drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive is not None
    radius_text = drive.findtext('wheel_radius')
    separation_text = drive.findtext('wheel_separation')
    assert radius_text is not None and separation_text is not None
    wheel_radius = float(radius_text)
    wheel_separation = float(separation_text)
    for side, y_sign in [('left', 1), ('right', -1)]:
        names = {f'{end}_{side}_wheel_joint' for end in ('front', 'rear')}
        assert {node.text for node in drive.findall(f'{side}_joint')} == names
        for name in names:
            joint = model.find(f"joint[@name='{name}']")
            assert joint is not None
            assert joint.get('type') == 'revolute'
            assert joint.findtext('parent') == 'base_link'
            wheel = model.find(f"link[@name='{joint.findtext('child')}']")
            assert wheel is not None
            pose_text = wheel.findtext('pose')
            assert pose_text is not None
            pose = [float(v) for v in pose_text.split()]
            # Wheels sit symmetrically at half the declared separation.
            assert pose[1] * y_sign > 0
            assert abs(pose[1]) == pytest.approx(wheel_separation / 2)
            cylinder = wheel.find('collision/geometry/cylinder')
            assert cylinder is not None
            radius_text = cylinder.findtext('radius')
            assert radius_text is not None
            # Collision radius matches the drivetrain radius, and the wheel
            # center sits at its own radius so it touches the ground plane.
            assert float(radius_text) == pytest.approx(wheel_radius)
            assert pose[2] == pytest.approx(wheel_radius)
            assert joint.findtext('axis/xyz') == '0 1 0'
    assert drive.findtext('frame_id') == 'odom'
    assert drive.findtext('child_frame_id') == 'base_link'


def test_mass_inertia_and_transport_envelope(model):
    """Every body is physical and the complete transport model fits SDR limits."""
    total_mass = 0
    bounds = [[], [], []]
    for link in model.findall('link'):
        mass = float(link.findtext('inertial/mass'))
        assert mass > 0
        total_mass += mass
        diagonal = [float(link.findtext(f'inertial/inertia/i{axis}{axis}'))
                    for axis in 'xyz']
        assert all(value > 0 for value in diagonal)
        assert all(2 * value <= sum(diagonal) + 1e-9 for value in diagonal)
        link_pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
        assert link_pose[3:] == [0, 0, 0]
        for shape in link.findall('collision') + link.findall('visual'):
            pose = [float(v) for v in shape.findtext('pose', '0 0 0 0 0 0').split()]
            assert pose[4:] == [0, 0]
            box = shape.find('geometry/box')
            if box is not None:
                assert pose[3] == 0
                sizes = [float(v) for v in box.findtext('size').split()]
            else:
                cylinder = shape.find('geometry/cylinder')
                radius = float(cylinder.findtext('radius'))
                # Cylinders are rotated at the geometry, with their axes along Y.
                assert abs(pose[3]) == pytest.approx(1.5707963267948966)
                sizes = [2 * radius, float(cylinder.findtext('length')), 2 * radius]
            for axis in range(3):
                center = link_pose[axis] + pose[axis]
                bounds[axis].extend([center - sizes[axis] / 2, center + sizes[axis] / 2])
    # Total mass must be positive and within SDR limit
    assert 0 < total_mass <= 80
    # Each axis extent must be within SDR limits
    limits = [1.5, 0.75, 0.75]
    for values, limit in zip(bounds, limits):
        extent = max(values) - min(values)
        assert extent <= limit
    assert min(bounds[2]) == pytest.approx(0)
    assert len([link for link in model.findall('link')
                if link.get('name').endswith('_drum')]) == 4
    assert len([link for link in model.findall('link')
                if link.get('name').endswith('_arm')]) == 2


def test_standalone_sdf_matches_parameter_source():
    """Changing the assumptions must also update the checked-in spawn model."""
    generated = xacro.process_file(str(MODEL_DIR / 'model.sdf.xacro')).toxml()
    actual = ET.parse(MODEL_DIR / 'model.sdf').getroot()
    expected = ET.fromstring(generated)
    # Ignore formatting and xacro's generated comments, not values or structure.

    def canonical(element):
        return (element.tag, element.attrib, (element.text or '').strip(),
                [canonical(child) for child in element])
    assert canonical(actual) == canonical(expected)

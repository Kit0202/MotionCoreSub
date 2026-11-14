"""Stylized simulation presets, not manufacturer-certified product dimensions."""
from dataclasses import dataclass


@dataclass(frozen=True)
class CanSpec:
    id: str
    label: str
    text: str
    radius: float
    half_height: float
    mass: float
    rgba: tuple
    grip_offset: tuple = (.13, .04, 0.)
    grip: tuple = (0., -.85, -.8, 1., .9, 1., .9)

    def public(self):
        return dict(id=self.id, label=self.label, diameter=self.radius*2,
                    height=self.half_height*2, mass=self.mass+.002)


CAN_TYPES = {
    'cola': CanSpec('cola', 'Cola can', 'COLA', .0325, .0575, .35, (.82,.015,.025,1.)),
    'tuna': CanSpec('tuna', 'Tuna can', 'TUNA', .041, .029, .18, (.92,.62,.12,1.),
                    (.13,.047,-.015)),
    '100plus': CanSpec('100plus', '100PLUS can', '100PLUS', .0315, .0625, .34, (.87,.92,.95,1.)),
    'pepsi': CanSpec('pepsi', 'Pepsi can', 'PEPSI', .0325, .060, .35, (.025,.12,.72,1.)),
}

# Each label is rendered using collision-free pixels on the curved can surface.
FONT = dict(zip('ACELNIOPSTU10', [
    '010/101/111/101/101', '111/100/100/100/111', '111/100/110/100/111',
    '100/100/100/100/111', '101/111/111/111/101', '111/010/010/010/111',
    '111/101/101/101/111', '110/101/110/100/100', '111/100/111/001/111',
    '111/010/010/010/010', '101/101/101/101/111', '010/110/010/010/111',
    '111/101/101/101/111',
]))

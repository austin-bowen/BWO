"""Convert lengths."""

from enum import Enum
from typing import Union

IntOrFloat = Union[int, float]


class Length:
    __slots__ = (
        'meters',
        'centimeters',
        'millimeters',

        'inches',
        'feet',
    )

    class Unit(Enum):
        METER = 1.0
        CENTIMETER = 0.01
        MILLIMETER = 0.001

        INCH = 0.0254
        FOOT = 0.0254 * 12

        @staticmethod
        def convert(length: IntOrFloat, from_unit: Enum, to_unit: Enum):
            if from_unit is to_unit:
                return length

            meters = length * from_unit.value
            return meters / to_unit.value

    def __init__(self, length: IntOrFloat, unit: Unit):
        self.meters = self.Unit.convert(length, unit, self.Unit.METER)
        self.centimeters = self.Unit.convert(length, unit, self.Unit.CENTIMETER)
        self.millimeters = self.Unit.convert(length, unit, self.Unit.MILLIMETER)

        self.inches = self.Unit.convert(length, unit, self.Unit.INCH)
        self.feet = self.Unit.convert(length, unit, self.Unit.FOOT)

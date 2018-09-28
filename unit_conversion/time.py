"""Convert times."""

from enum import Enum
from typing import Union

IntOrFloat = Union[int, float]


class Time:
    __slots__ = (
        'seconds',
        'milliseconds',
        'minutes',
        'hours',
        'days'
    )

    class Unit(Enum):
        SECOND = 1
        MILLISECOND = 0.001
        MINUTE = 60
        HOUR = 60 * 60
        DAY = 60 * 60 * 24

        @staticmethod
        def convert(time: IntOrFloat, from_unit: Enum, to_unit: Enum):
            if from_unit is to_unit:
                return time

            seconds = time * from_unit.value
            return seconds / to_unit.value

    def __init__(self, length: IntOrFloat, unit: Unit):
        self.seconds = self.Unit.convert(length, unit, self.Unit.SECOND)
        self.milliseconds = self.Unit.convert(length, unit, self.Unit.MILLISECOND)
        self.minutes = self.Unit.convert(length, unit, self.Unit.MINUTE)
        self.hours = self.Unit.convert(length, unit, self.Unit.HOUR)
        self.days = self.Unit.convert(length, unit, self.Unit.DAY)

"""BWO settings."""

from unit_conversion.length import Length
from numpy import pi

BODY_RADIUS = Length(4, Length.Unit.INCH)

WHEEL_DIAMETER = Length(58, Length.Unit.MILLIMETER)
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi

MOTOR_VOLTAGE = 5
LEFT_MOTOR_CHANNEL = 5
RIGHT_MOTOR_CHANNEL = 3
BACK_MOTOR_CHANNEL = 4

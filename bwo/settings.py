"""BWO settings."""

from numpy import pi

from unit_conversion.length import Length

BODY_RADIUS = Length(4, Length.Unit.INCH)

WHEEL_DIAMETER = Length(58, Length.Unit.MILLIMETER)
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi

MOTOR_VOLTAGE = 6.5
LEFT_MOTOR_CHANNEL = 3
RIGHT_MOTOR_CHANNEL = 5
BACK_MOTOR_CHANNEL = 4

def normalize_angle_degrees(angle):
    """
    :return: The angle normalized to (-180, 180] degrees.
    """

    angle = angle % 360
    return angle if angle <= 180 else (angle - 360)

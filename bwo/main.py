"""
BWO the robot.
"""

'''
class VideoCapture(cv2.VideoCapture):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


def cat_seeker(bwo: Bwo, controller: gamesir.GameSirController):
    import cv2

    cat_detector = cv2.CascadeClassifier('haarcascade_frontalcatface_extended.xml')

    blackout_start_time = None
    with VideoCapture(0) as camera:
        while True:
            # Get frame
            ret, frame = camera.read()

            # Blackout frame?
            if np.max(frame) <= 20:
                now = time()
                if blackout_start_time is None:
                    blackout_start_time = now
                    bwo.drive_motors.stop()
                    continue
                elif (now - blackout_start_time) >= 10:
                    print('Blackout timeout reached!')
                    break
            else:
                blackout_start_time = None

            # Detect cats
            cat_rects = cat_detector.detectMultiScale(
                frame,
                scaleFactor=1.1,
                minNeighbors=1,
                minSize=(10, 10)
            )
            if not len(cat_rects):
                continue

            # Pick the largest detected cat
            cat_rect = max(cat_rects, key=lambda rect: rect[2] * rect[3])
            del cat_rects

            # Get the cat's center point
            cat_center = (
                cat_rect[0] + (cat_rect[2] / 2),
                cat_rect[1] + (cat_rect[3] / 2)
            )
            del cat_rect

            # Get the cat's offset from center
            frame_height, frame_width, _ = frame.shape
            cat_offset = (
                2 * (cat_center[0] / frame_width) - 1,
                2 * (cat_center[1] / frame_height) - 1
            )
            del cat_center, frame_height, frame_width

            # Set the body velocity and head position
            bwo.drive_motors.set_body_velocity(0, 0.0, cat_offset[0] * 0.5)
            bwo.head.set_head_position(0)
'''

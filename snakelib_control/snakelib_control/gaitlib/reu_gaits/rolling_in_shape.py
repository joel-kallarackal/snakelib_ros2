import numpy as np


def rotate_shape(x, y, rot):
    return x * np.cos(rot) + y * np.sin(rot)


def rolling_in_shape(self, t=0, current_angles=None, params=None):

    rot_speed = max(params.get("wT_odd", 3), params.get("wT_even", 3))

    x, y = np.array(current_angles[::2]), np.array(current_angles[1::2])

    imbalanced = False

    if len(x) > len(y):
        y = np.concatenate((y, np.array([0])))
        imbalanced = True
    elif len(y) > len(x):
        x = np.concatenate((x, np.array([0])))
        imbalanced = True

    rot = (rot_speed * t) % (2 * np.pi)

    rotated_x = rotate_shape(x, y, rot)
    rotated_y = rotate_shape(x, y, rot + np.pi / 2)

    angles = np.stack([rotated_x, rotated_y]).T.flatten()

    if imbalanced:
        return angles[: len(angles) - 1]
    else:
        return angles

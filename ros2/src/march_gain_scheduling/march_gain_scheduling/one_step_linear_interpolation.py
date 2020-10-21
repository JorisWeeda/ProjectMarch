from typing import List

import numpy as np

from .interpolation_errors import NegativeValueError, UnequalLengthError


def interpolate(current_gains: np.ndarray, needed_gains: np.ndarray, gradient: float, delta_t: float) -> np.ndarray:
    """ Interpolate linearly between the current gains and the needed gains.

    Based on whether the needed gains are larger or smaller than the current gains, a value of gradient * delta_t is
    either added or subtracted from the current gains to get closer to the needed gains.
    The values are clamped using np.minimum or np.maximum to prevent interpolating too much.

    :param current_gains Starting point of the interpolation
    :param needed_gains Ending point of the interpolation
    :param gradient Gradient to interpolate with, must be larger than 0
    :param delta_t Difference in time (s)

    :returns Returns the interpolated gains
    """
    if len(current_gains) != len(needed_gains):
        raise UnequalLengthError('current_gains and needed_gains do not have the same length')
    if gradient <= 0:
        raise NegativeValueError('gradient')
    if delta_t < 0:
        raise NegativeValueError('delta_t')
    next_gains = np.zeros(current_gains.shape)

    # Interpolate negatively where the needed gains are smaller than the current gains
    interpolate_negative = needed_gains <= current_gains
    next_gains[interpolate_negative] = np.maximum(needed_gains[interpolate_negative],
                                                  current_gains[interpolate_negative] - gradient * delta_t)

    # Interpolate positively where the needed gains are larger than the current gains
    interpolate_positive = ~interpolate_negative
    next_gains[interpolate_positive] = np.minimum(needed_gains[interpolate_positive],
                                                  current_gains[interpolate_positive] + gradient * delta_t)
    return next_gains

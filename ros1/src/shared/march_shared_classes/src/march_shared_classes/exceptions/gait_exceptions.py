class GaitError(Exception):
    def __init__(self, msg=None):
        """Base class for exceptions in gait modules.

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'An error occurred with a gait module.'
        super(GaitError, self).__init__(msg)


class GaitNameNotFound(GaitError):
    def __init__(self, gait_name, msg=None):
        """Class to raise an error when given gait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Could not find gait name: {gait} in map.'.format(gait=gait_name)

        super(GaitNameNotFound, self).__init__(msg)


class SubgaitNameNotFound(GaitError):
    def __init__(self, subgait_name, gait_name, msg=None):
        """Class to raise an error when given subgait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Could not find subgait name {subgait} of gait {gait} in map.'.format(subgait=subgait_name,
                                                                                        gait=gait_name)

        super(SubgaitNameNotFound, self).__init__(msg)


class NonValidGaitContent(GaitError):
    def __init__(self, gait_name=None, msg=None):
        """Class to raise an error when given gait has incorrect content .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'The given gait: {gn} has incorrect information'.format(gn=gait_name)

        super(NonValidGaitContent, self).__init__(msg)


class SubgaitGraphError(GaitError):
    def __init__(self, msg):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when transition between two subgaits has an error .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = 'Subgaits can not transition'

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when it was not possible to interpolate between subgaits."""
        if msg is None:
            msg = 'An error occurred while trying to merge two subgaits.'

        super(SubgaitInterpolationError, self).__init__(msg)


class SideSpecificationError(Exception):
    def __init__(self, foot, msg=None):
        """Class to raise an error when a foot ('right' or 'left') has to be specified but this did not happen."""
        if msg is None:
            msg = "An incorrect side was supplied. Must be either 'left' or 'right', but was '{foot}'.".\
                format(foot=foot)

        super(SideSpecificationError, self).__init__(msg)


class IncorrectCoordinateError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when the coordinates of a position are incorrect."""
        if msg is None:
            msg = "The keys of a position or velocity dictionary should be ['x', 'y', 'z'], but were different."

        super(IncorrectCoordinateError, self).__init__(msg)


class WeightedAverageError(Exception):
    def __init__(self, msg=None):
        """Class to raise an error when a weighted average cannot be computed."""
        if msg is None:
            msg = 'The calculation of the weighted average cannot be executed safely.'

        super(WeightedAverageError, self).__init__(msg)

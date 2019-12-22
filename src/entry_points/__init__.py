from inspect import getmodule


class RobotPYEntryPoint:
    """
    Base class for all robotpy entry points
    """
    def __init__(self, parser):
        """
        The init method is handled by this class
        :param parser: The cmd_parser provided by robotpy
        """
        self.parser = parser


def robotpy_entry_point(*, name):
    """
    Creates a robotpy compatible entry point for the decorated function
    :return: The function to turn into an entry point
    """
    def function_wrapper(func):
        entry_point = type(name, (RobotPYEntryPoint,), {'run': staticmethod(func)})
        entry_point.__doc__ = func.__doc__
        entry_point.__qualname__ = func.__module__ + '.' + name
        setattr(getmodule(func), name, entry_point)
        return func
    return function_wrapper

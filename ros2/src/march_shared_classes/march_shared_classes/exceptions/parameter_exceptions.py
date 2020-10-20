class ParameterException(Exception):
    """General exception for when something with ROS1 parameter server goes wrong."""
    def __init__(self, parameter_name):
        self.parameter_name = parameter_name

    def __str__(self):
        return f'Something went wrong with parameter server for parameter {self.parameter_name}'


class ParameterGetException(ParameterException):
    """Exception for when something goes wrong with getting a parameter in the ROS1 server"""
    def __str__(self):
        return f'Something went wrong with getting parameter {self.parameter_name}'


class ParameterSetException(ParameterException):
    """Exception for when something goes wrong with setting a parameter in the ROS1 server"""
    def __str__(self):
        return f'Something went wrong with setting parameter {self.parameter_name}'

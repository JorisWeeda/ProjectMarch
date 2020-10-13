import rospy
from march_shared_resources.srv import RequestParamString, RequestParamList, \
     RequestParamStringResponse, RequestParamListResponse, RequestParamInt, RequestParamIntResponse, \
     RequestParamFloat, RequestParamFloatResponse, RequestParamBool, RequestParamBoolResponse, SetParamFloat


class ParameterServer:
    """
    The ParameterServer class is used to construct a Node that ROS2 nodes can use to retrieve information from the
    ROS1 parameter server.
    """

    def __init__(self):
        """
        Construct a service for each parameter type
        """
        rospy.Service('march/parameter_server/request_param_string', RequestParamString,
                      lambda req: self.request_callback(req, RequestParamStringResponse))
        rospy.Service('march/parameter_server/request_param_list', RequestParamList,
                      lambda req: self.request_callback(req, RequestParamListResponse))
        rospy.Service('march/parameter_server/request_param_bool', RequestParamBool,
                      lambda req: self.request_callback(req, RequestParamBoolResponse))
        rospy.Service('march/parameter_server/request_param_float', RequestParamFloat,
                      lambda req: self.request_callback(req, RequestParamFloatResponse))
        rospy.Service('march/parameter_server/request_param_int', RequestParamInt,
                      lambda req: self.request_callback(req, RequestParamIntResponse))
        rospy.Service('march/parameter_server/set_param_float', SetParamFloat,
                      lambda req: self.set_callback(req))

    @staticmethod
    def request_callback(req, type):
        """
        Look into the ROS1 parameter server and return the value of a parameter

        :req request of the client
        :type type of request response
        """
        rospy.loginfo('Retrieving param with name: ' + req.name)

        if req.name in rospy.get_param_names():
            return type(rospy.get_param(req.name))
        raise InvalidParamName(req.name)

    @staticmethod
    def set_callback(req):
        rospy.loginfo("Setting param with name " + req.name + " to value: " + str(req.name))
        rospy.set_param(req.name, req.value)


class InvalidParamName(Exception):
    """
    Custom exception for when a parameter is not available in the parameter server
    """
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return 'Param with name %s doesn\'t exist in the parameter server' % self.name


def main():
    rospy.init_node('march_parameter_server_node')
    ParameterServer()
    rospy.spin()

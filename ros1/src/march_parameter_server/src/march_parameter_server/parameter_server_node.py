import rospy
from march_shared_resources.srv import GetParamString, GetParamStringList, \
     GetParamStringResponse, GetParamStringListResponse, GetParamInt, GetParamIntResponse, \
     GetParamFloat, GetParamFloatResponse, GetParamBool, GetParamBoolResponse, \
     SetParamFloat, SetParamFloatResponse, SetParamString, SetParamStringResponse, \
     SetParamStringList, SetParamStringListResponse, SetParamBool, SetParamBoolResponse, \
    SetParamInt, SetParamIntResponse


class ParameterServer:
    """
    The ParameterServer class is used to construct a Node that ROS2 nodes can use to retrieve information from the
    ROS1 parameter server.
    """

    def __init__(self):
        """
        Construct a service for each parameter type
        """
        # Get services
        rospy.Service('march/parameter_server/get_param_string', GetParamString,
                      lambda req: self.get_callback(req, GetParamStringResponse))
        rospy.Service('march/parameter_server/get_param_string_list', GetParamStringList,
                      lambda req: self.get_callback(req, GetParamStringListResponse))
        rospy.Service('march/parameter_server/get_param_bool', GetParamBool,
                      lambda req: self.get_callback(req, GetParamBoolResponse))
        rospy.Service('march/parameter_server/get_param_float', GetParamFloat,
                      lambda req: self.get_callback(req, GetParamFloatResponse))
        rospy.Service('march/parameter_server/get_param_int', GetParamInt,
                      lambda req: self.get_callback(req, GetParamIntResponse))
        
        # Set services
        rospy.Service('march/parameter_server/set_param_string', SetParamString,
                      lambda req: self.set_callback(req, SetParamStringResponse))
        rospy.Service('march/parameter_server/set_param_string_list', SetParamStringList,
                      lambda req: self.set_callback(req, SetParamStringListResponse))
        rospy.Service('march/parameter_server/set_param_bool', SetParamBool,
                      lambda req: self.set_callback(req, SetParamBoolResponse))
        rospy.Service('march/parameter_server/set_param_float', SetParamFloat,
                      lambda req: self.set_callback(req, SetParamFloatResponse))
        rospy.Service('march/parameter_server/set_param_int', SetParamInt,
                      lambda req: self.set_callback(req, SetParamIntResponse))

    @staticmethod
    def get_callback(req, type):
        """
        Look into the ROS1 parameter server and return the value of a parameter

        :req request of the client
        :type type of request response
        """
        rospy.loginfo('Retrieving param with name: ' + req.name)

        if req.name in rospy.get_param_names():
            response = type(rospy.get_param(req.name))

            rospy.loginfo('Returning response')
            rospy.loginfo(str(response))

            return response
        raise InvalidParamName(req.name)

    @staticmethod
    def set_callback(req, type):
        rospy.loginfo("Setting param with name " + req.name + " to value: " + str(req.value))

        if req.name in rospy.get_param_names():
            rospy.set_param(req.name, req.value)
            return type(rospy.get_param(req.name))
        raise InvalidParamName(req.name)


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
import rospy
from march_shared_resources.msg import GainValues, GainValuesList

from march_shared_resources.srv import GetParamBool, GetParamBoolResponse, GetParamFloat, GetParamFloatResponse, \
    GetParamInt, GetParamIntResponse, GetParamString, GetParamStringList, GetParamStringListResponse, \
    GetParamStringResponse, SetParamBool, SetParamBoolResponse, SetParamFloat, SetParamFloatResponse, SetParamInt, \
    SetParamIntResponse, SetParamString, SetParamStringList, SetParamStringListResponse, SetParamStringResponse, \
    GetGainValuesListResponse, GetGainValuesList, SetGainValuesList, SetGainValuesListResponse


class ParameterServer:
    """The ParameterServer class is used to get and set parameters on the ROS1 parameter server."""

    def __init__(self):
        """Construct a get and set service for each parameter type."""
        # Get services
        rospy.Service('/march/parameter_server/get_param_string', GetParamString,
                      lambda req: self.get_callback(req, GetParamStringResponse))
        rospy.Service('/march/parameter_server/get_param_string_list', GetParamStringList,
                      lambda req: self.get_callback(req, GetParamStringListResponse))
        rospy.Service('/march/parameter_server/get_param_bool', GetParamBool,
                      lambda req: self.get_callback(req, GetParamBoolResponse))
        rospy.Service('/march/parameter_server/get_param_float', GetParamFloat,
                      lambda req: self.get_callback(req, GetParamFloatResponse))
        rospy.Service('/march/parameter_server/get_param_int', GetParamInt,
                      lambda req: self.get_callback(req, GetParamIntResponse))

        # Set services
        rospy.Service('/march/parameter_server/set_param_string', SetParamString,
                      lambda req: self.set_callback(req, SetParamStringResponse))
        rospy.Service('/march/parameter_server/set_param_string_list', SetParamStringList,
                      lambda req: self.set_callback(req, SetParamStringListResponse))
        rospy.Service('/march/parameter_server/set_param_bool', SetParamBool,
                      lambda req: self.set_callback(req, SetParamBoolResponse))
        rospy.Service('/march/parameter_server/set_param_float', SetParamFloat,
                      lambda req: self.set_callback(req, SetParamFloatResponse))
        rospy.Service('/march/parameter_server/set_param_int', SetParamInt,
                      lambda req: self.set_callback(req, SetParamIntResponse))

        # Get and setters for gain values list
        rospy.Service('/march/parameter_server/get_gain_values_list', GetGainValuesList, self.get_gain_values_callback)
        rospy.Service('/march/parameter_server/set_gain_values_list', SetGainValuesList, self.set_gain_values_callback)

    @staticmethod
    def get_callback(req, response_type):
        """
        Look into the ROS1 parameter server and return the value of a parameter.

        :param req get request of the client
        :param response_type of request response

        :return Returns a Response of 'response_type'.
                The success variable of the response is set to true if the parameter could be read from the ROS1
                parameter server, otherwise it is set to false.
        """
        rospy.loginfo('Retrieving parameter with name: ' + req.name)

        try:
            return response_type(rospy.get_param(req.name), True)
        except KeyError:
            return response_type(None, False)

    @staticmethod
    def set_callback(req, response_type):
        """
        Set a parameter in the ROS1 parameter server.

        :param req set request of the client
        :param response_type of request response

        :return Returns a Response of 'response_type' with the success variable set to true
        """
        rospy.loginfo('Setting parameter with name ' + req.name + ' to value: ' + str(req.value))

        rospy.set_param(req.name, req.value)
        return response_type(True)

    @staticmethod
    def get_gain_values_callback(req):
        rospy.loginfo('Getting gains for joints:' + str(req.joints))

        gain_values_list = GainValuesList([GainValues([
            rospy.get_param('/march/controller/trajectory/gains/{joint}/{gain}'.format(joint=joint, gain=gain))
            for gain in ['p', 'i', 'd']])
                       for joint in req.joints])

        return GetGainValuesListResponse(gain_values_list, True)

    @staticmethod
    def set_gain_values_callback(req):
        rospy.loginfo('Setting gains for joints:' + str(req.joints))

        for joint_index, joint in enumerate(req.joints):
            gains = req.gain_values.gain_values_list[joint_index].gains
            for gain_index, gain in enumerate(['p', 'i', 'd']):
                rospy.set_param('/march/controller/trajectory/gains/{joint}/{gain}'.format(joint=joint, gain=gain),
                                gains[gain_index])

        return SetGainValuesListResponse(True)


def main():
    rospy.init_node('march_parameter_server_node')
    ParameterServer()
    rospy.spin()

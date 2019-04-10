
#ifndef MARCH_GAIT_GENERATOR_POSESTAMPED_H
#define MARCH_GAIT_GENERATOR_POSESTAMPED_H


#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <utility>

class PoseStamped {

private:
    float seconds;
    float percentage;
    Pose pose;

public:

    PoseStamped(float seconds, float percentage, const std::vector<std::string> &name) : seconds(seconds), percentage(percentage), pose(name) {}
    PoseStamped(float seconds, float percentage, Pose pose) : seconds(seconds), percentage(percentage), pose(pose) {}
    explicit PoseStamped(const std::vector<std::string> &name): PoseStamped(0,0, name) {}

    /** @brief Override comparison operator */
    friend bool operator==(const PoseStamped& lhs, const PoseStamped& rhs)
    {
        return lhs.seconds == rhs.seconds && lhs.percentage == rhs.percentage;
    }

    friend bool operator!=(const PoseStamped& lhs, const PoseStamped& rhs)
    {
        return !(lhs == rhs);
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const PoseStamped& poseStamped)
    {
        return os << "seconds: " << poseStamped.seconds << " percentage: " << poseStamped.percentage;
    }


};


#endif //MARCH_GAIT_GENERATOR_POSESTAMPED_H

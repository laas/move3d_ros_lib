#ifndef MOVE3D_ROBOTSTFPUBLISHER_H
#define MOVE3D_ROBOTSTFPUBLISHER_H

#include <vector>
#include <string>
#include <tf/transform_broadcaster.h>

namespace ros{
class NodeHandle;
}
class Robot;
namespace move3d {

class SceneManager;

class RobotsTfPublisher
{
public:
    RobotsTfPublisher(SceneManager *scMgr, ros::NodeHandle *nh);
    ~RobotsTfPublisher();

    bool publishTf(const std::vector<std::string > &names, const ros::Time &date);
    bool publishTf(Robot *r, const ros::Time &date);

private:
    SceneManager *_scMgr;
    ros::NodeHandle *_nh;
    tf::TransformBroadcaster *_tf_br;
};

} // namespace move3d

#endif // MOVE3D_ROBOTSTFPUBLISHER_H

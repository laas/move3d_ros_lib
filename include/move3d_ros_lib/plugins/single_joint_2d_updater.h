#ifndef MOVE3D_SINGLEJOINT2DUPDATER_H
#define MOVE3D_SINGLEJOINT2DUPDATER_H

#include "move3d_ros_lib/plugins/base_human_updater.h"

namespace move3d {

class SingleJoint2dUpdater : public BaseHumanUpdater
{
public:
    SingleJoint2dUpdater();
    virtual ~SingleJoint2dUpdater();

    virtual bool update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings);
    virtual bool computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q);
};

} // namespace move3d

#endif // MOVE3D_SINGLEJOINT2DUPDATER_H

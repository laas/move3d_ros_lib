#ifndef MOVE3D_HEADHANDUPDATER_H
#define MOVE3D_HEADHANDUPDATER_H

#include "move3d_ros_lib/plugins/base_human_updater.h"

namespace move3d {

class HeadHandUpdater : public BaseHumanUpdater
{
public:
    HeadHandUpdater();
    virtual ~HeadHandUpdater();

    virtual bool update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings);
    virtual bool computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q);

    /**
     * @brief getBothJoints
     * @param[in] h
     * @param[in] base
     * @param[in] joints
     * @param[in] input_name
     * @param[in] m3d_name
     * @param[out] joint
     * @param[out] m3d_joint_index
     * @return false if at least one is not found
     */
    bool getBothJoints(Robot *h, const std::map<std::string, Eigen::Affine3d> &joints,
                       const std::string &input_name, const std::string &m3d_name,
                       std::map<std::string, Eigen::Affine3d>::const_iterator &joint, uint &m3d_joint_index);

    bool updateFreeFlyer(RobotState &q,uint first_dof_index,const Eigen::Affine3d &pos);
};

} // namespace move3d

#endif // MOVE3D_HEADHANDUPDATER_H

#ifndef MOVE3D_HEADHANDUPDATER_H
#define MOVE3D_HEADHANDUPDATER_H

#include "move3d_ros_lib/plugins/base_human_updater.h"

namespace tf{class TransformBroadcaster;}

namespace move3d {

/**
 * @brief The HeadHandUpdater class updates base, hand and head.
 *
 * @todo the names are hard coded.
 * move3d joints: base_torso, torso_head, right_hand_joint
 * input link positions: base, head, rightHand
 */
class HeadHandUpdater : public BaseHumanUpdater
{
public:
    HeadHandUpdater();
    virtual ~HeadHandUpdater();

    virtual bool update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings);
    virtual bool computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q);

private:
    /**
     * @brief searches for the corresponding joint position in the input and joint index in the move3d Robot object
     * @param[in] h move3d Robot object representing the human
     * @param[in] base base position
     * @param[in] joints input joint position list
     * @param[in] input_name name of joint in the input
     * @param[in] m3d_name name of the joint in move3d
     * @param[out] joint iterator to the position of the joint in the input
     * @param[out] m3d_joint_index index of the corresponding joint in move3d
     * @return false if the matching elements are not found.
     */
    bool getBothJoints(Robot *h, const std::map<std::string, Eigen::Affine3d> &joints,
                       const std::string &input_name, const std::string &m3d_name,
                       std::map<std::string, Eigen::Affine3d>::const_iterator &joint, uint &m3d_joint_index);

    bool updateFreeFlyer(RobotState &q,uint first_dof_index,const Eigen::Affine3d &pos);
    tf::TransformBroadcaster *_tf_br;
};

} // namespace move3d

#endif // MOVE3D_HEADHANDUPDATER_H

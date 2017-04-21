#include "move3d_ros_lib/plugins/single_joint_2d_updater.h"

#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/ConfigSpace/RobotState.hpp>

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>


PLUGINLIB_EXPORT_CLASS(move3d::SingleJoint2dUpdater, move3d::BaseHumanUpdater)

namespace move3d {

SingleJoint2dUpdater::SingleJoint2dUpdater():BaseHumanUpdater()
{

}

SingleJoint2dUpdater::~SingleJoint2dUpdater()
{

}

bool SingleJoint2dUpdater::update(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings)
{
    RobotState q(h);
    bool ok=computeConf(h,base,joints,settings,q);
    h->setAndUpdate(q);
    return ok;
}

bool SingleJoint2dUpdater::computeConf(Robot *h, const Eigen::Affine3d &base, const std::map<std::string, Eigen::Affine3d> &joints, const HumanSettings &settings, RobotState &q)
{
    bool ok=true;
    q = *h->getCurrentPos();
    std::string torsojoint = "base_torso";
    if(h->getHriAgent()->type==HRI_HERAKLES){
        torsojoint="Pelvis";
    }
    for(uint i=0;i<3;i++){
        q[i+6]=base.translation()[i];
    }
    Eigen::Vector3d rotations = base.rotation().eulerAngles(0,1,2);
    for(uint i=0;i<3;i++){
        q[i+3+6]=rotations[i];
    }
    Joint *torso=h->getJoint(torsojoint);
    if(torso){
        uint index=torso->getIndexOfFirstDof();
        float height;
        if(settings.getData("height",height))
            q[index+2]=height;
        else{
            ROS_WARN("No height specified for human %s, needed by SingleJoint2dUpdater human manager plugin",h->getName().c_str());
            ok=false;
        }
    }else{
        ROS_WARN("No joint named %s in human model %s",torsojoint.c_str(),h->getName().c_str());
        ok=false;
    }
    return ok;
}

} // namespace move3d


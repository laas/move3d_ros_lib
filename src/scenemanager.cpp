#include "move3d_ros_lib/scenemanager.h"
#include "move3d_ros_lib/humanmgr.h"

#include <libmove3d/planners/API/project.hpp>
#include <ros/ros.h>
#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/ConfigSpace/RobotState.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <sstream>
#include <libmove3d/planners/utils/Geometry.h>
//#include <libmove3d/p3d/proto/p3d_rw_scenario_proto.h>
//#include <libmove3d/include/device.h>
#include <libmove3d/include/Collision-pkg.h>
#include <libmove3d/include/P3d-pkg.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace move3d
{
SceneManager::SceneManager(ros::NodeHandle *nh):
    _nh(nh),_p3dPath(""),_scePath(""), _project(0), _updateAcceptBaseOnly(0)
{
    preInit();
}

SceneManager::SceneManager(ros::NodeHandle *nh, const std::string &p3d_path):
    _nh(nh),_p3dPath(p3d_path),_scePath(""), _project(0), _updateAcceptBaseOnly(0)
{
    _frame_transform = Eigen::Affine3d();
    preInit();
}

SceneManager::~SceneManager()
{

}

bool SceneManager::addModule(const std::string &name)
{
    if(_project){
        ROS_WARN("Cannot add module %s: project already initialized",name.c_str());
        return false;
    }else{
        _modules_to_activ.insert(name);
    }
    return true;
}

bool SceneManager::createScene()
{
    bool ok(1);
    if(_p3dPath.size()==0){
        ok=false;
        ROS_ERROR("no p3d file is given.");
        _project=NULL;
    }
    if(ok){
        p3d_col_set_mode(p3d_col_mode_none);
        p3d_read_desc(_p3dPath.c_str());
        p3d_col_set_mode(p3d_col_mode_pqp);
        p3d_col_start(p3d_col_mode_pqp);
        if(_scePath.size()){
            ROS_INFO("loading sce file %s",_scePath.c_str());
            p3d_read_scenario(_scePath.c_str());
        }
        global_Project = new Project(new Scene(XYZ_ENV));
        foreach(const std::string &n,_modules_to_activ){
            global_Project->addModule(n);
        }
        ok=global_Project->init();
        for(uint i=0;i<global_Project->getActiveScene()->getNumberOfRobots();i++){
            Robot *r=global_Project->getActiveScene()->getRobot(i);
            r->setAndUpdate(*r->getInitialPosition());
        }
    }
    if(ok){
        _project=global_Project;
    }else{
        ROS_ERROR("error on creating the move3d project");
        _project=NULL;
    }

    _humanMgr = new HumanMgr(_nh,this);
    return ok;
}

bool SceneManager::updateRobotPose(const std::string &name, const geometry_msgs::Pose &base_pose)
{
    if(!_project){
        ROS_ERROR("the project is not initialized, cannot update robot position");
        return false;
    }
    bool ok(1);
    Robot *r=_project->getActiveScene()->getRobotByName(name);
    if(!r){
        ROS_ERROR("no robot named %s in the scene",name.c_str());
        ok=false;
    }
    if(ok){
        m3dGeometry::setBasePosition(r,Eigen::Vector3d(base_pose.position.x,base_pose.position.y,base_pose.position.z));
        Eigen::Matrix3d mat = Eigen::Quaterniond(base_pose.orientation.w,
                           base_pose.orientation.x,
                           base_pose.orientation.y,
                           base_pose.orientation.z).matrix();
        m3dGeometry::setBaseOrientation(r,mat);
    }

    return ok;
}

bool SceneManager::updateRobot(const std::string &name, const geometry_msgs::Pose &base_pose, const std::vector<double> &dof_values)
{
    bool ok(true);
    std::vector<double> m3d_dofs;

    ok = convertConfRosToM3d(name,dof_values,m3d_dofs);
    if(!ok && updateAcceptBaseOnly()){
        ROS_WARN_ONCE("Updating %s: no or wrong configuration given, updating only base pose.",name.c_str());
        ROS_DEBUG("Updating %s: no or wrong configuration given, updating only base pose.",name.c_str());
        return updateRobotPose(name,base_pose);
    }
    if(!ok){
        return false;
    }

    m3d_dofs[0+6]=base_pose.position.x;
    m3d_dofs[1+6]=base_pose.position.y;
    m3d_dofs[2+6]=base_pose.position.z;

    //tf::Quaternion rot(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w); // w last in tf
    Eigen::Quaterniond rot(base_pose.orientation.w,base_pose.orientation.x,base_pose.orientation.y,base_pose.orientation.z); //w first in Eigen
    Eigen::Matrix3d mat= rot.matrix();
    Eigen::Vector3d euler =mat.eulerAngles(0,1,2);
    for(unsigned int i=0;i<3;++i){
        m3d_dofs[3+6+i]=euler[i];
    }
    return updateRobotM3d(name,m3d_dofs);

}

bool SceneManager::updateRobotM3d(const std::string &name, std::vector<double> &dof_values)
{
    if(!_project){
        ROS_ERROR("the project is not initialized, cannot update robot position");
        return false;
    }
    bool ok(1);
    Robot *r=_project->getActiveScene()->getRobotByName(name);
    if(!r){
        ROS_ERROR("no robot named %s in the scene",name.c_str());
        ok=false;
    }
    if(ok){
        RobotState q(r,1);
        for(uint i=6;i<dof_values.size();++i){
            q[i]=dof_values[i];
        }
        r->setAndUpdate(q);
        //if(q.getNbDof() == dof_values.size()){
        //    q.setConfiguration(&dof_values[0]);
        //    r->setAndUpdate(q);
        //}else{
        //    ok=false;
        //    ROS_ERROR("wrong number of joints for robot %s. Given %lu, move3d says it has %u",name.c_str(),dof_values.size(),q.getNbDof());
        //}
    }
    return ok;
}

bool SceneManager::updateObject(const std::string &name, const geometry_msgs::Pose &pose)
{
    return updateRobotPose(name,pose);
}

Eigen::Affine3d pose2affine(const geometry_msgs::Pose &pose){
    Eigen::Affine3d aff;
    Eigen::Vector3d tr(pose.position.x,pose.position.y,pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    aff=Eigen::Translation3d(pose.position.x,pose.position.y,pose.position.z);
    //aff.rotate(q);
    aff*=q;
    for(uint i=0;i<3;i++){
        ROS_DEBUG("%f %f",aff.translation()[i],tr[i]);
    }
    return aff;
}

bool SceneManager::updateHuman(const std::string &name, const geometry_msgs::Pose &base_pose, const std::map<std::string, geometry_msgs::Pose> &joints)
{

    Eigen::Affine3d base;
    std::map<std::string,Eigen::Affine3d> joints_tf;
    base = pose2affine(base_pose);
    typedef std::pair<std::string, geometry_msgs::Pose> Iterator;
    foreach(Iterator it,joints){
        joints_tf[it.first] = pose2affine(it.second);
    }
    return _humanMgr->setHumanPos(name,base,joints_tf);
}

int indexOfDof(Robot *r,const std::string &dof_name){
    for(unsigned int j=0;j<=r->getNumberOfJoints();j++){
        Joint *joint=r->getJoint(j);
        if(joint->getName().find(dof_name) == 0){
            if(joint->getNumberOfDof() == 1){
                return joint->getIndexOfFirstDof();
            }else{
                //several dofs, have to read the dof index in the name
                std::string subname = dof_name.substr(dof_name.find_first_of("0123456789"), joint->getName().size());
                int index;
                if ( ! (std::istringstream(subname) >> index) ){
                    return -1;
                }
                return joint->getIndexOfFirstDof() + index;
            }
        }
    }
    return -1;
}

bool SceneManager::setDofNameOrdered(const std::string &robot_name, const std::vector<std::string> &dof_names)
{
    JointCorrespName_t::const_iterator corresp_names_it = _jointCorrespNames.find(robot_name);
    bool ok(1);
    if(corresp_names_it == _jointCorrespNames.end()){
        ROS_ERROR("no joint correspondances set for robot %s.",robot_name.c_str());
        ok=false;
    }
    if(ok && !_project){
        ok=false;
        ROS_ERROR("the project is not initialized, cannot set robot dof correspondance");
    }
    if(ok && !(_project->getActiveScene() && _project->getActiveScene()->getRobotByName(robot_name))){
        ok=false;
        ROS_ERROR("no robot named %s in the scene",robot_name.c_str());
    }

    if(ok){
        Robot *r = _project->getActiveScene()->getRobotByName(robot_name);
        const NameMap_t &corresp_names = corresp_names_it->second;
        if(dof_names.size() == 0)
            ROS_WARN("setDofNameOrdered: dof_names is empty, will not be abble to set joint name correspondance");
        for(unsigned int i=0;i<dof_names.size();++i){
            const std::string &dof_name = dof_names[i];
            NameMap_t::const_iterator name_pair_it = corresp_names.find(dof_name);
            if(name_pair_it != corresp_names.end()){
                //the Dof has a correspondance, find the corresponding index in move3d
                int index = indexOfDof(r,name_pair_it->second);
                if(index>=0){
                    _jointCorrespIndex[robot_name][i]=index;
                }else{
                    ROS_ERROR("DoF named %s not found for robot %s in move3d (corresponding to dof %s in ROS)",name_pair_it->second.c_str(),robot_name.c_str(),dof_name.c_str());
                    ok=false;
                    break;
                }
            }
        }
    }
    return ok;
}

bool SceneManager::fetchDofCorrespParam(const std::string &param_name, const std::string &robot_name)
{
    NameMap_t &name_map = _jointCorrespNames[robot_name];
    //XmlRpc::XmlRpcValue value;
    ros::param::get(param_name,name_map);
    //for(XmlRpc::XmlRpcValue::iterator it = value.begin(); it!=value.end();++it){
    //    std::string key = it->first;
    //    XmlRpc::XmlRpcValue name = it->second;
    //    name_map[key]=name

    //}
    typedef std::pair<std::string,std::string> namePair_t;
    foreach(namePair_t v,name_map){
        ROS_INFO("%s %s",v.first.c_str(),v.second.c_str());
    }
}

bool SceneManager::convertConfRosToM3d(const std::string &robot_name, const std::vector<double> &ros_conf, std::vector<double> &m3d_conf)
{
    bool ok(true);
    m3d_conf.clear();
    if(_jointCorrespIndex.find(robot_name) == _jointCorrespIndex.end()){
        ROS_DEBUG("no joint indices configured for robot %s (yet)",robot_name.c_str());
        return false;
    }
    const UintMap_t &index_map = _jointCorrespIndex[robot_name];
    m3d_conf.assign(index_map.size(),0.);
    uint count(0);
    for(uint i=0;i<ros_conf.size();++i){
        UintMap_t::const_iterator it = index_map.find(i);
        if(it!=index_map.end()){
            ++count;
            if(m3d_conf.size()<=it->second){
                m3d_conf.resize(it->second+1);
            }
            m3d_conf.at(it->second) = ros_conf[i];
        }
    }
    if (count!=index_map.size()){
        ROS_ERROR("Some joints are missing in the given ros conf. I couldn't fill the m3d conf (%u set vs. %lu expected)",count,index_map.size());
        ok=false;
    }
    return ok;
}

void SceneManager::setDofCorresp(const std::string &robot_name, const std::map<std::string, std::string> &ros_to_move3d_map)
{
    _jointCorrespNames[robot_name]=ros_to_move3d_map;
}

void SceneManager::preInit()
{
    //init move3d logger
    logm3d::initializePlannerLogger();
}
std::string SceneManager::scePath() const
{
    return _scePath;
}

void SceneManager::setScePath(const std::string &scePath)
{
    _scePath = scePath;
}

ros::NodeHandle *SceneManager::nh() const
{
    return _nh;
}

void SceneManager::setNh(ros::NodeHandle *nh)
{
    _nh = nh;
}

bool SceneManager::updateAcceptBaseOnly() const
{
    return _updateAcceptBaseOnly;
}

void SceneManager::setUpdateAcceptBaseOnly(bool updateAcceptBaseOnly)
{
    _updateAcceptBaseOnly = updateAcceptBaseOnly;
}

Eigen::Affine3d SceneManager::getFrameTransform() const
{
    return _frame_transform;
}

void SceneManager::setFrameTransform(const Eigen::Affine3d &frame_change)
{
    _frame_transform = frame_change;
}

std::string SceneManager::p3dPath() const
{
    return _p3dPath;
}

void SceneManager::setP3dPath(const std::string &p3dPath)
{
    _p3dPath = p3dPath;
}

bool SceneManager::saveScenario(const std::string &path)
{
    for(unsigned int i=0;i<_project->getActiveScene()->getNumberOfRobots();++i){
        Robot *r=_project->getActiveScene()->getRobot(i);
        p3d_copy_config_into(r->getRobotStruct(),r->getCurrentPos()->getConfigStruct(),&(r->getRobotStruct()->ROBOT_POS) );
    }
    ROS_INFO("Saving scenario in %s",path.c_str());
    p3d_rw_scenario_init_name();
    bool ok = p3d_save_scenario(path.c_str());
    if(!ok)
        ROS_ERROR("failed to save scenario");
    return ok;
}
Project *SceneManager::project() const
{
    return _project;
}

void SceneManager::setProject(Project *project)
{
    _project = project;
}


}

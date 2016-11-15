#include "move3d_ros_lib/humanmgr.h"

#include "move3d_ros_lib/scenemanager.h"
#include "move3d_ros_lib/plugins/base_human_updater.h"

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/API/project.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace ros;
using namespace std;

namespace move3d
{

HumanMgr::~HumanMgr()
{
    delete _human_updater_loader;
}

bool HumanMgr::updateParams()
{
    _humanSettings.clear();
    bool ret(true);
    if(updateHumanList()){
        foreach (string h,_humans){
            map<string,float> settings;
            string policy;
            HumanSettingsPtr set_struct;
            bool ok= _nh->getParam("human_mgmt/"+h+"/policy",policy);
            if (ok){
                string move3d_name;
                _nh->param<map<string,float> >("human_mgmt/"+h+"/data",settings,map<string,float>());//defaults to empty
                _nh->param<string>("human_mgmt/"+h+"/move3d_name",move3d_name,h);//defaults to h
               set_struct=boost::shared_ptr<HumanSettings>(new HumanSettings(settings,policy,move3d_name));
            }else{
                ret=false;
                ROS_WARN("Invalid parameters for Human management of %s",h.c_str());
            }
            //find the corresponding human updater
            if(ok){
                UpdaterIterator it;
                it=_updaters.find(set_struct->policy);
                if(it!=_updaters.end()){
                    set_struct->updater=it->second;
                    ROS_DEBUG("human_setting for %s: policy=%s",h.c_str(),policy.c_str());
                }else{
                    ok=false;
                    ROS_WARN("Couldn't find a updater plugin class called %s for human %s",set_struct->policy.c_str(),h.c_str());
                }
            }
            if(ok){
                _humanSettings[h]=set_struct;
            }
        }
    }else{
        ret=false;
    }
    return ret;
}

bool HumanMgr::loadPlugins()
{
    _updaters.clear();
    _human_updater_loader = new pluginlib::ClassLoader<BaseHumanUpdater>("move3d_ros_lib","move3d::BaseHumanUpdater");
    vector<string> list = _human_updater_loader->getDeclaredClasses();
    foreach (string plug,list){
        ROS_INFO("available plugin %s",plug.c_str());
        //_updaters[plug]=_human_updater_loader->createInstance(plug);
    }
    foreach (string plug,list){
        ROS_INFO("loading plugin %s",plug.c_str());
        _updaters[plug]=_human_updater_loader->createInstance(plug);
    }
}

bool HumanMgr::setHumanPos(const std::string &name,const Eigen::Affine3d &base, const std::map<std::string,Eigen::Affine3d> &joints)
{
    HumanSettingsPtr settings;
    bool ok(true);
    //find the settings for that human
    if(ok){
        HumanSettingsIterator it;
        it = _humanSettings.find(name);
        if(it!=_humanSettings.end()){
            settings=it->second;
        }else{
            ok=false;
            ROS_WARN("No settings found to update/translate the position of human %s",name.c_str());
        }
    }
    //find the robot in the scene
    Robot *h;
    if(ok){

        h=_scMgr->project()->getActiveScene()->getRobotByName(settings->move3d_name);
        if(!h){
            ok=false;
            ROS_WARN("No human named %s in move3d scene.",settings->move3d_name.c_str());
        }
    }
    if(ok){
        if(!settings->updater){
            ok=false;
        }
    }
    if(ok){
        ok=settings->updater->update(h,base,joints,*settings);
    }
    return ok;
}

bool HumanMgr::updateHumanList()
{
    _humans.clear();
    XmlRpc::XmlRpcValue value;
    if(_nh->getParam("human_mgmt",value)){
        for(XmlRpc::XmlRpcValue::iterator it=value.begin();it!=value.end();++it){
            _humans.push_back(it->first);
        }
        return true;
    }else{
        ROS_WARN("No human_mgmt parameter found");
    }
    return false;
}
std::vector<std::string> HumanMgr::humanList() const
{
    return _humans;
}

HumanSettingsPtr HumanMgr::getHumanSettings(const string &name)
{
    HumanSettingsIterator it=_humanSettings.find(name);
    if(it!=_humanSettings.end()){
        return it->second;
    }else{
        return HumanSettingsPtr();
    }
}

HumanMgr::HumanMgr(NodeHandle *nh, SceneManager *sc):
    _nh(nh),_scMgr(sc)
{
    loadPlugins();
    updateParams();
}


}

#include "move3d_ros_lib/savescenariosrv.h"
#include "move3d_ros_lib/scenemanager.h"
#include <ros/service_server.h>

SaveScenarioSrv::SaveScenarioSrv(SceneManager *sceneMgr, ros::NodeHandle *node):
    _sceneMgr(sceneMgr),_node(node),_srv(0)
{

}

SaveScenarioSrv::~SaveScenarioSrv()
{
    if(_srv)
        delete _srv;

}

bool SaveScenarioSrv::advertise(const std::string &name)
{
    _srv = new ros::ServiceServer(_node->advertiseService(name,&SaveScenarioSrv::save,this));
}

bool SaveScenarioSrv::save(move3d_ros_lib::SaveScenarioRequest &req, move3d_ros_lib::SaveScenarioResponse &res)
{
    bool ok = _sceneMgr->saveScenario(req.file_path);
    res.success=ok;

    return true;
}


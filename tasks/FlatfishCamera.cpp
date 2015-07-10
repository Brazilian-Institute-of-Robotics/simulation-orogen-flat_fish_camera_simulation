/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FlatfishCamera.hpp"

using namespace flat_fish_camera_simulation;

FlatfishCamera::FlatfishCamera(std::string const& name)
    : FlatfishCameraBase(name)
{
}

FlatfishCamera::FlatfishCamera(std::string const& name, RTT::ExecutionEngine* engine)
    : FlatfishCameraBase(name, engine)
{
}

FlatfishCamera::~FlatfishCamera()
{
}

bool FlatfishCamera::configureHook()
{
    if (! FlatfishCameraBase::configureHook())
        return false;

    setupLaserLines();

    return true;
}

bool FlatfishCamera::startHook()
{
    if (! FlatfishCameraBase::startHook())
        return false;

    addLaserLinePlugins();

    return true;
}

void FlatfishCamera::updateHook()
{
    FlatfishCameraBase::updateHook();

    for(std::map<std::string,RTT::base::PortInterface*>::iterator it = laserLineDataPorts.begin();
            it != laserLineDataPorts.end(); ++it)
    {
        std::string linkName = it->first;
        std::string posePortName = linkNameToPosePortName[linkName];

        RTT::InputPort<base::samples::RigidBodyState>* linkPoseCmd = dynamic_cast<RTT::InputPort<base::samples::RigidBodyState>*>(laserLinePosePorts[posePortName]);
        base::samples::RigidBodyState linkPose;
        while (linkPoseCmd->readNewest(linkPose) == RTT::NewData) {

            std::cout << "setTransformation" << std::endl;
            std::cout << "source_frame: " << linkPose.sourceFrame << std::endl;
            std::cout << "target_frame: " << linkPose.targetFrame << std::endl;
            vizkit3dWorld->setTransformation(linkPose);
        }

        RTT::InputPort<base::samples::LaserScan>* laserCmd = dynamic_cast<RTT::InputPort<base::samples::LaserScan>*>(it->second);

        base::samples::LaserScan laserData;
        while (laserCmd->readNewest(laserData) == RTT::NewData) {
            laserLinePlugins[linkName]->updateData(laserData);
        }


    }

}

void FlatfishCamera::errorHook()
{
    FlatfishCameraBase::errorHook();
}

void FlatfishCamera::stopHook()
{
    removeLaserLinePlugins();
    FlatfishCameraBase::stopHook();
}

void FlatfishCamera::cleanupHook()
{
    cleanupLaserLines();
    FlatfishCameraBase::cleanupHook();
}

void FlatfishCamera::setupLaserLines() {
    std::vector<LaserLineParams> params = _laser_line_params.get();
    for(std::vector<LaserLineParams>::iterator it = params.begin();
            it != params.end(); ++it )
    {
        std::string link_name = it->link_name;
        std::string pose_port_name = it->pose_port_name;

        RTT::InputPort<base::samples::LaserScan> *dataPortIn = new RTT::InputPort<base::samples::LaserScan>(link_name);
        RTT::InputPort<base::samples::RigidBodyState> *posePortIn = new RTT::InputPort<base::samples::RigidBodyState>(pose_port_name);

        laserLineDataPorts.insert(std::make_pair(link_name, dataPortIn));
        laserLinePosePorts.insert(std::make_pair(pose_port_name, posePortIn));

        linkNameToPosePortName.insert(std::make_pair(link_name, pose_port_name));

        ports()->addEventPort(*dataPortIn);
        ports()->addEventPort(*posePortIn);

        vizkit3d::LaserLine* laserLinePlugin = new vizkit3d::LaserLine();
        laserLinePlugins.insert(std::make_pair(link_name, laserLinePlugin));
    }
}

void FlatfishCamera::cleanupLaserLines()
{
    for(std::map<std::string,RTT::base::PortInterface*>::iterator it = laserLineDataPorts.begin();
            it != laserLineDataPorts.end(); ++it)
    {
        std::string linkName = it->first;
        std::string posePortName = linkNameToPosePortName[linkName];

        ports()->removePort(posePortName);
        ports()->removePort(linkName);

        delete laserLinePosePorts[posePortName];
        delete laserLinePlugins[linkName];
        delete it->second;
    }

    linkNameToPosePortName.clear();
    laserLinePlugins.clear();
    laserLineDataPorts.clear();
}

void FlatfishCamera::addLaserLinePlugins() {
    for(std::map<std::string, vizkit3d::LaserLine*>::iterator it = laserLinePlugins.begin();
            it != laserLinePlugins.end(); ++it ) {
        std::string link_name = it->first;
        vizkit3d::LaserLine* plugin = it->second;
        vizkit3dWorld->addPlugin(plugin);
//        plugin->setPluginName(QString::fromStdString(link_name));
//        plugin->setVisualizationFrame(QString::fromStdString(link_name));
    }
}

void FlatfishCamera::removeLaserLinePlugins() {

    for (std::map<std::string, vizkit3d::LaserLine*>::iterator it = laserLinePlugins.begin();
            it != laserLinePlugins.end(); ++it) {
        vizkit3dWorld->getWidget()->removePlugin(it->second);
    }

}


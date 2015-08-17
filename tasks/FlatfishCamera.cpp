/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FlatfishCamera.hpp"

using namespace flat_fish_camera_simulation;

FlatfishCamera::FlatfishCamera(std::string const& name)
    : FlatfishCameraBase(name),
      laserLineFrontPlugin(NULL),
      laserLineBottomPlugin(NULL)
{
}

FlatfishCamera::FlatfishCamera(std::string const& name, RTT::ExecutionEngine* engine)
    : FlatfishCameraBase(name, engine),
      laserLineFrontPlugin(NULL),
      laserLineBottomPlugin(NULL)
{
}

FlatfishCamera::~FlatfishCamera()
{
}

bool FlatfishCamera::configureHook()
{
    if (! FlatfishCameraBase::configureHook())
        return false;

    return true;
}

bool FlatfishCamera::startHook()
{
    if (! FlatfishCameraBase::startHook())
        return false;

    return true;
}

void FlatfishCamera::updateHook()
{
    FlatfishCameraBase::updateHook();

    //update front laser line plugin
    updateLaserLinePlugin(laserLineFrontPlugin, _laser_line_front_data_cmd, _laser_line_front_pose_cmd);

    //update bottom laser line plugin
    updateLaserLinePlugin(laserLineBottomPlugin, _laser_line_bottom_data_cmd, _laser_line_bottom_pose_cmd);
}

void FlatfishCamera::updateLaserLinePlugin(vizkit3d::LaserLine *plugin, RTT::InputPort<base::samples::LaserScan>& dataCmd, RTT::InputPort<base::samples::RigidBodyState>& poseCmd)
{
    /**
     * Read laser line link pose
     */
    base::samples::RigidBodyState pose;
    while (poseCmd.readNewest(pose) == RTT::NewData) {
        vizkit3dWorld->setTransformation(pose);
    }

    /**
     * read laser scan data sent
     */
    base::samples::LaserScan data;
    while (dataCmd.readNewest(data) == RTT::NewData) {
        plugin->updateData(data);
    }
}

void FlatfishCamera::errorHook()
{
    FlatfishCameraBase::errorHook();
}

void FlatfishCamera::stopHook()
{
    FlatfishCameraBase::stopHook();
}

void FlatfishCamera::cleanupHook()
{
    FlatfishCameraBase::cleanupHook();
}

void FlatfishCamera::addLaserLinePlugins() {
    laserLineFrontPlugin = new vizkit3d::LaserLine();
    vizkit3dWorld->getWidget()->addPlugin(laserLineFrontPlugin);
    setupLaserLinePlugin(laserLineFrontPlugin, _laser_line_front_params.get());

    laserLineBottomPlugin = new vizkit3d::LaserLine();
    vizkit3dWorld->getWidget()->addPlugin(laserLineBottomPlugin);
    setupLaserLinePlugin(laserLineBottomPlugin, _laser_line_bottom_params.get());
}

void FlatfishCamera::setupLaserLinePlugin(vizkit3d::LaserLine *plugin, LaserLineParams params)
{
    QColor qcolor;
    qcolor.setRgbF(params.line_color[0], params.line_color[1], params.line_color[2]);

    /**
     * the plugin name is the link name
     */
    plugin->setPluginName(QString::fromStdString(params.link_name));

    /**
     * the frame name must be the same name of the link which represents the laser line in model
     */
    plugin->setVisualizationFrame(QString::fromStdString(params.link_name));
    plugin->setColor(qcolor);
    plugin->setLineWidth(params.line_width);
}

void FlatfishCamera::removeLaserLinePlugins() {

    if (laserLineFrontPlugin) {
        vizkit3dWorld->getWidget()->removePlugin(laserLineFrontPlugin);
        delete laserLineFrontPlugin;
        laserLineFrontPlugin = NULL;
    }

    if (laserLineBottomPlugin) {
        vizkit3dWorld->getWidget()->removePlugin(laserLineFrontPlugin);
        delete laserLineBottomPlugin;
        laserLineBottomPlugin  = NULL;
    }
}

void FlatfishCamera::onCreateWorld() {
    FlatfishCameraBase::onCreateWorld();
    addLaserLinePlugins();
}

void FlatfishCamera::onDestroyWorld() {
    removeLaserLinePlugins();
    FlatfishCameraBase::onDestroyWorld();
}


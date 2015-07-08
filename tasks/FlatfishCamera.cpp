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

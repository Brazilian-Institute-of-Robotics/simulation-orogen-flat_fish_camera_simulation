/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef FLAT_FISH_CAMERA_SIMULATION_FLATFISHCAMERA_TASK_HPP
#define FLAT_FISH_CAMERA_SIMULATION_FLATFISHCAMERA_TASK_HPP

#include "flat_fish_camera_simulation/FlatfishCameraBase.hpp"
#include <vizkit3d/LaserLine.hpp>

namespace flat_fish_camera_simulation{
    class FlatfishCamera : public FlatfishCameraBase
    {
	friend class FlatfishCameraBase;
    protected:
        vizkit3d::LaserLine *laserLinePlugin;

    public:
        FlatfishCamera(std::string const& name = "flat_fish_camera_simulation::FlatfishCamera");
        FlatfishCamera(std::string const& name, RTT::ExecutionEngine* engine);
    	~FlatfishCamera();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif


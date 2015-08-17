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

        /**
         * Represents the flatfish's front laser line
         */
        vizkit3d::LaserLine *laserLineFrontPlugin;

        /**
         * Represents the flatfish's bottom laser line
         */
        vizkit3d::LaserLine *laserLineBottomPlugin;

        /**
         * Configure the laser line plugin using the parameters
         *
         * @param[in, out]  plugin Pointer to laser line plugin
         * @param[in]       params The laser line parameters
         */
        void setupLaserLinePlugin(vizkit3d::LaserLine *plugin, LaserLineParams params);

        /**
         * Read input ports and update laser line plugin
         * @param[in,out]   plugin Pointer to laser line plugin
         * @param[in]       dataCmd Input port with laser scan data
         * @param[in]       poseCmd Input port with laser link pose
         */
        inline void updateLaserLinePlugin(vizkit3d::LaserLine *plugin, RTT::InputPort<base::samples::LaserScan>& dataCmd, RTT::InputPort<base::samples::RigidBodyState>& poseCmd);

        /**
         * Add laser laser line plugins to vizkit3d widget
         */
        void addLaserLinePlugins();

        /**
         * Remove laser line plugins to vizkit3d widget
         */
        void removeLaserLinePlugins();

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

        virtual void onCreateWorld();

        virtual void onDestroyWorld();
    };
}

#endif

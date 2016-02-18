#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <rsb/Factory.h>
#include <rsb/Listener.h>
#include <rsb/Informer.h>

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/geometry/Translation.pb.h>

#include <rsc/logging/ConsoleLogger.h>
#include <rsc/logging/LoggerFactory.h>

#include <stdio.h>
#include <time.h>

namespace gazebo_rsb_model_state_plugin {
class RSBModelStatePlugin: public gazebo::ModelPlugin {
public:
	void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	void OnUpdate(const gazebo::common::UpdateInfo & _info);

private:
	void readConfig(sdf::ElementPtr _sdf);

	// Pointer to the model
	gazebo::physics::ModelPtr model;
	// Pointer to the update event connection
	gazebo::event::ConnectionPtr updateConnection;

	// Publish current Pose of the model to the outside
//	rsb::ListenerPtr rsbcmdJointPosition_Listener;
	rsb::Informer<rst::geometry::Pose>::Ptr inf_CurrentPose;

	rsc::logging::LoggerPtr logger = rsc::logging::Logger::getLogger(
			"gazebo.rsb.RSBModelStatePlugin");

	// Scope used to publish current Pose of the model
	std::string scope_current_pose;

	double time_counter = 0;
	clock_t this_time;
	clock_t last_time;

};
}

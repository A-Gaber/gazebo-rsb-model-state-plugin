/*
 * author Dennis Leroy Wigand
 */
#include "gazebo-rsb-model-state-plugin.hpp"

#include <boost/lexical_cast.hpp>
//boost::lexical_cast<std::string>(...)

#include <rsb/MetaData.h>
#include <rsb/EventCollections.h>
#include <rsb/converter/EventsByScopeMapConverter.h>
#include <rsb/converter/ByteArrayConverter.h>

#include <rsb/filter/ScopeFilter.h>
#include <rsb/converter/Converter.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rsc/misc/langutils.h>

using namespace std;
using namespace rsb;
using namespace rsb::patterns;
using namespace gazebo;
using namespace gazebo_rsb_model_state_plugin;

const double NUM_SECONDS = 0.1;

void RSBModelStatePlugin::Load(physics::ModelPtr _parent,
		sdf::ElementPtr _sdf) {

	this_time = clock();
	last_time = this_time;

	// Store the pointer to the model
	this->model = _parent;

	Factory& factory = getFactory();

	// Set Defaults
	scope_current_pose = "/gazebo/model/" + this->model->GetName() + "/status";
	inf_CurrentPose = factory.createInformer<rst::geometry::Pose>(
			scope_current_pose);

	// Read Config
	this->readConfig(_sdf);

	cout << "Setup current model pose informer on scope: " << scope_current_pose
			<< endl;

	RSCINFO(logger,
			"Setup current model pose informer on scope: " << scope_current_pose);

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&RSBModelStatePlugin::OnUpdate, this, _1));
	RSCINFO(logger, "Connect model plugin to world update loop.");
}

// Called by the world update start event
void RSBModelStatePlugin::OnUpdate(const common::UpdateInfo & _info) {
	this_time = clock();
	time_counter += (double) (this_time - last_time);
	last_time = this_time;

	if (time_counter > (double) (NUM_SECONDS * CLOCKS_PER_SEC)) {
		time_counter -= (double) (NUM_SECONDS * CLOCKS_PER_SEC);
		// Apply a small linear velocity to the model.
		//this->model->SetLinearVel(math::Vector3(.03, 0, 0));
		math::Pose model_pose = this->model->GetWorldPose();

		// only publish in specific rhythm
		boost::shared_ptr<rst::geometry::Pose> curr_pose2pub(
				new rst::geometry::Pose);

		curr_pose2pub->mutable_translation()->set_x(model_pose.pos.x);
		curr_pose2pub->mutable_translation()->set_y(model_pose.pos.y);
		curr_pose2pub->mutable_translation()->set_z(model_pose.pos.z);

		curr_pose2pub->mutable_rotation()->set_qw(model_pose.rot.w);
		curr_pose2pub->mutable_rotation()->set_qx(model_pose.rot.x);
		curr_pose2pub->mutable_rotation()->set_qy(model_pose.rot.y);
		curr_pose2pub->mutable_rotation()->set_qz(model_pose.rot.z);

		inf_CurrentPose->publish(curr_pose2pub);
		RSCINFO(logger,
				"Public current model pose: x = " << model_pose.pos.x << ", y = " << model_pose.pos.y << ", z = " << model_pose.pos.z << "    w = " << model_pose.rot.w << ", x = " << model_pose.rot.x << ", y = " << model_pose.rot.y << ", z = " << model_pose.rot.z);

	}
}

void RSBModelStatePlugin::readConfig(sdf::ElementPtr _sdf) {
	sdf::ElementPtr startElem = _sdf->GetFirstElement();
	while (startElem) {
		if (startElem->GetName() == "scope") {
			if (startElem->HasAttribute("name")) {
				sdf::ParamPtr scopeNameParam = startElem->GetAttribute("name");
				string name = scopeNameParam->GetAsString();
				sdf::ElementPtr elem = _sdf->GetFirstElement();
				if (elem) {
					string val = elem->GetValue()->GetAsString();
					// set scope:
					if (name == "scope_current_pose") {
						scope_current_pose = val;
					}
				}
			}
		}

		startElem = _sdf->GetNextElement();
	}
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RSBModelStatePlugin)

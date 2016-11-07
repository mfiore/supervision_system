#ifndef BasicPick_H
#define BasicPick_H

#include <ros/ros.h>

#include <action_nodes/basic_actions/basic_action.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <situation_assessment_msgs/PutObjectInHand.h>

class BasicPick: public BasicAction {
	public:
	BasicPick(ros::NodeHandle node_handle);

protected:
	bool checkPreconditions(StringMap parameters);
	void setPostconditions(StringMap parameters);

private:
	ros::ServiceClient put_object_in_hand_client_;
};

#endif
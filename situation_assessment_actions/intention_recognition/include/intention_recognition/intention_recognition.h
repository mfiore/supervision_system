#ifndef INTENTION_RECOGNITION_H
#define INTENTION_RECOGNITION

#include <ros/ros.h>
#include "intention_recognition/observations_collector.h"
#include "situation_assessment_actions_msgs/StartMonitorIntentions.h"
#include "situation_assessment_actions_msgs/ExecutableActions.h"
#include "situation_assessment_actions_msgs/ExecutableAgentActions.h"
#include  "situation_assessment_actions_msgs/IntentionGraphResult.h"
#include  "situation_assessment_actions_msgs/IntentionProb.h"
#include "IntentionGraph.h"
#include "Mdp.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/lock_guard.hpp> 
#include <set>
#include <vector>
#include <string>
#include <map>
#include <std_msgs/Float32.h>


typedef std::vector<std::string> StringVector;
typedef std::pair<std::string,std::string> StringPair;

class IntentionRecognition {
public:
	IntentionRecognition(ros::NodeHandle node_handle);
	void intentionLoop();

private:

	//creates an IG for an agent
	bool startMonitoring(situation_assessment_actions_msgs::StartMonitorIntentions::Request &req,
		situation_assessment_actions_msgs::StartMonitorIntentions::Response &res);

	//loads all the known MDPs, by reading their model files
	void loadMdp();

	void actionCallback(const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg);


	void createIntentionGraph(std::string agent,std::vector<std::string> actions);

	std::string createActionString(action_management_msgs::Action a);

	std::map<std::string,IntentionGraph*> getIntentionGraphs();

	ros::NodeHandle node_handle_;
	ObservationsCollector observations_collector_;

	StringVector contexts_;

	std::map<std::string, std::vector<std::string> > linked_contexts_;
	std::map<StringPair,double> intention_conditional_probabilities_;
	std::map<std::string,Mdp*> mdp_map_;

	StringVector intention_list_;
	std::map<std::string,IntentionGraph*> agents_intentions_;

	ros::Subscriber action_sub_;

	ros::Publisher pub_intentions_;
	std::map<std::string, ros::Publisher*> pub_intentions_plot_;
	ros::Publisher tea_pub_;


	boost::mutex mutex_igs_;

	std::map<std::string,std::vector<action_management_msgs::Action> > executable_actions_;
	std::vector<std::string> locations_;
	std::vector<std::string> humans_;
	std::map<std::string,std::vector<std::string> > move_actions_;  
	std::map<std::string,std::string> human_locations_;

	int n_loops_;
};
#endif
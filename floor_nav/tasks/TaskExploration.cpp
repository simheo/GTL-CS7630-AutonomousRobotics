#include "TaskExploration.h"
#include "floor_nav/TaskExplorationConfig.h"
#include "tf/transform_datatypes.h"

#include "std_msgs/Bool.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_Exploration
#ifdef DEBUG_Exploration
#warning Debugging task Exploration
#endif


TaskIndicator TaskExploration::initialise() 
{
    explorePub = env->getNodeHandle().advertise<std_msgs::Bool>("/floor_nav/explore",1);

    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskExploration::iterate()
{
	std_msgs::Bool msg;
	msg.data=true;
	explorePub.publish(msg);
	//ROS_INFO("Trigger Sent");
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryExploration);

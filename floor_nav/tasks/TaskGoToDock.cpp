#include "TaskGoToDock.h"
#include "floor_nav/TaskGoToDockConfig.h"
#include "kobuki_msgs/AutoDockingGoal.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GoToDock
#ifdef DEBUG_GoToDock
#warning Debugging task GoToDock
#endif


TaskIndicator TaskGoToDock::initialise() 
{
    ROS_INFO("Initialization of docking");
    client = new actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction>("dock_drive_action",true);
    client->waitForServer();
    
    kobuki_msgs::AutoDockingGoal goal;
    client->sendGoal(goal);
    

    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToBase::iterate()
{
	state = client->getState();
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("I'm on the dock");
		return TaskStatus::TASK_COMPLETED;
    }
	
	
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToBase::terminate()
{
    
	return TaskStatus::TASK_TERMINATED;
}


DYNAMIC_TASK(TaskFactoryGoToDock);

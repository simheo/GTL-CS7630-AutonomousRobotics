#include "TaskCheckBatt.h"
#include "floor_nav/TaskExplorationConfig.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float32.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_CheckBatt
#ifdef DEBUG_CheckBatt
#warning Debugging task CheckBatt
#endif

//void TaskCheckBatt::BatteryLevelCallback(const kobuki_msgs::SensorStateConstPtr msg){
	//uint8_t battLevel;
	//battLevel=msg.battery;
void TaskCheckBatt::BatteryLevelCallback(std_msgs::Float32 msg){
	battLevel = msg.data;
}


TaskIndicator TaskCheckBatt::initialise() 
{
    //ROS_INFO("Init battery task")
    //checkBattSub = env->getNodeHandle().subscribe("/mobile_base/sensors/core",1,&TaskCheckBatt::checkBatteryCallback,this);
	BattSub = env->getNodeHandle().subscribe("/acpi_monitor/battery",1,&TaskCheckBatt::BatteryLevelCallback,this);
	
	
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskCheckBatt::iterate()
{
	if (battLevel<=10.0){
		ROS_INFO("Low Battery");
		return TaskStatus::TASK_COMPLETED;
	}
	ROS_INFO("battery level %f",battLevel);
	return TaskStatus::TASK_RUNNING;
}


DYNAMIC_TASK(TaskFactoryCheckBatt);



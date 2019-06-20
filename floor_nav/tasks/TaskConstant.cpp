#include <math.h>
#include "TaskConstant.h"
#include "floor_nav/TaskConstantConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskConstant::initialise() 
{
    t0 = ros::Time::now();
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskConstant::iterate()
{
    if ((ros::Time::now()-t0).toSec() > cfg.duration) {
        return TaskStatus::TASK_COMPLETED;
    }
    env->publishVelocity(cfg.linear, cfg.angular);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskConstant::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryConstant);

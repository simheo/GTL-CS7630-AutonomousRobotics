#ifndef TASK_CheckBatt_H
#define TASK_CheckBatt_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskCheckBattConfig.h"
#include "std_msgs/Float32.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskCheckBatt : public TaskInstance<TaskCheckBattConfig,SimTasksEnv>
    {
        protected:
             ros::Subscriber BattSub;
             float battLevel;
             
            
        public:
            TaskCheckBatt(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskCheckBatt() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
            
            void BatteryLevelCallback(std_msgs::Float32 msg );
    };
    class TaskFactoryCheckBatt : public TaskDefinition<TaskCheckBattConfig, SimTasksEnv, TaskCheckBatt>
    {

        public:
            TaskFactoryCheckBatt(TaskEnvironmentPtr env) : 
                Parent("TaskCheckBatt","Check the battery level",true,env) {}
            virtual ~TaskFactoryCheckBatt() {};
    };
};

#endif // TASK_CheckBatt_H

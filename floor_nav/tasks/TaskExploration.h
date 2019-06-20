#ifndef TASK_Exploration_H
#define TASK_Exploration_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskExplorationConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskExploration : public TaskInstance<TaskExplorationConfig,SimTasksEnv>
    {
        protected:
            ros::Publisher explorePub;
            
        public:
            TaskExploration(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskExploration() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
    };
    class TaskFactoryExploration : public TaskDefinition<TaskExplorationConfig, SimTasksEnv, TaskExploration>
    {

        public:
            TaskFactoryExploration(TaskEnvironmentPtr env) : 
                Parent("TaskExploration","Triggers the exploration node",true,env) {}
            virtual ~TaskFactoryExploration() {};
    };
};

#endif // TASK_Exploration_H

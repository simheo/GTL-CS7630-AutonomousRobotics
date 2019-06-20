#ifndef TASK_Constant_H
#define TASK_Constant_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskConstantConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskConstant : public TaskInstance<TaskConstantConfig,SimTasksEnv>
    {
        protected:
            ros::Time t0;
        public:
            TaskConstant(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskConstant() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryConstant : public TaskDefinition<TaskConstantConfig, SimTasksEnv, TaskConstant>
    {

        public:
            TaskFactoryConstant(TaskEnvironmentPtr env) : 
                Parent("Constant","Apply a constant command for a given duration",true,env) {}
            virtual ~TaskFactoryConstant() {};
    };
};

#endif // TASK_Constant_H

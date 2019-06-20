#ifndef TASK_MOVE_BASE_H
#define TASK_MOVE_BASE_H

#include "task_manager_lib/TaskDefinition.h"
#include "task_manager_action/TaskActionGeneric.h"
#include "kobuki_msgs/AutoDockingAction.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskAutoDockConfig.h"

using namespace task_manager_lib;
using namespace task_manager_action;

namespace floor_nav {
    class TaskAutoDock : public TaskActionGeneric<kobuki_msgs::AutoDockingAction,TaskAutoDockConfig, SimTasksEnv>
    {
        protected:
            //
            typedef TaskActionGeneric<kobuki_msgs::AutoDockingAction,
                    TaskAutoDockConfig, SimTasksEnv> Parent;

            const std::string & getActionName() const {
                return Parent::cfg.action_name;
            }

            void buildActionGoal(typename Parent::Goal & goal) const {
                // This goal is empty
            }
        public:
            TaskAutoDock(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskAutoDock() {};
    };
    class TaskFactoryAutoDock : public TaskDefinition<TaskAutoDockConfig, SimTasksEnv, TaskAutoDock>
    {

        public:
            TaskFactoryAutoDock(TaskEnvironmentPtr env) : 
                Parent("AutoDock","Call the AutoDocking action",true,env) {}
            virtual ~TaskFactoryAutoDock() {};
    };
};

#endif // TASK_MOVE_BASE_H

#ifndef TASK_GoToDock_H
#define TASK_GoToDock_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToDockConfig.h"
#include <actionlib/client/simple_action_client.h>
#include "kobuki_msgs/AutoDockingAction.h"

typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> Client;

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGoToDock : public TaskInstance<TaskGoToDockConfig,SimTasksEnv>
    {
        protected:
			actionlib::SimpleActionClient,kobuki_msgs::AutoDockingAction>* client;
			//actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;

        public:
            TaskGoToBack(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoToDock() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
            
            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoToDock : public TaskDefinition<TaskGoToDockConfig, SimTasksEnv, TaskGoToDock>
    {

        public:
            TaskFactoryGoToDock(TaskEnvironmentPtr env) : 
                Parent("GoToDock","Goes find its docking charger",true,env) {}
            virtual ~TaskFactoryGoToDock() {};
    };
};

#endif // TASK_GoToDock_H

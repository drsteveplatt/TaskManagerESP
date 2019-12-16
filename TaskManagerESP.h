#ifndef TASKMANAGER_H_INCLUDED
#define TASKMANAGER_H_INCLUDED
#define TASKMANAGERESP_H_INCLUDED

#include <TaskManagerMacros.h>
#include <TaskManagerESPCore.h>
TaskManagerESP TaskMgr;
void loop() {
    TaskMgr.loop();
}
#endif // TASKMANAGER_H_INCLUDED



/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <gtest/gtest.h>

#include <rw/core/Ptr.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadSafeVariable.hpp>
#include <rw/common/ThreadTask.hpp>
#include <rw/math/Random.hpp>
#include <rw/core/macros.hpp>


#include <boost/function.hpp>

using namespace rw::common;
using namespace rw::core;
using rw::math::Random;

static const unsigned int THREADS = 3;
static const unsigned int JOBS = 50;
static const unsigned int STOPJOB = 10; // ThreadPool should be able to shut down before reaching job JOBS
static ThreadSafeVariable<bool> requestStop(false);
static boost::mutex jobMutex;
static bool jobDone[JOBS];
static boost::mutex testMutex;

void workFunction(ThreadPool* pool, unsigned int id) {
	Timer::sleepMs(50);
	{
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_GT(pool->getQueueSize() , 0);
		EXPECT_EQ(pool->isStopping() , false);
	}
	try {
		if (pool->isStopping())
			return;
		boost::mutex::scoped_lock lock(jobMutex);
		jobDone[id] = true;
	} catch(boost::thread_interrupted &) {
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(pool->isStopping() , true);
	}
}

void workFunctionStop(ThreadPool* pool, unsigned int id) {
	try {
		Timer::sleepMs(50);
		{
			boost::mutex::scoped_lock lock(testMutex);
			EXPECT_GT(pool->getQueueSize() , 0);
		}
		if (id >= STOPJOB) {
			requestStop = true;
			Timer::sleepMs(500); // slow it a bit down to give main thread time to call stop
		}
		if (pool->isStopping())
			return;
		boost::mutex::scoped_lock lock(jobMutex);
		jobDone[id] = true;
	} catch(boost::thread_interrupted &) {
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(pool->isStopping() , true);
	}
}

int runPool(bool stop) {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));
	{
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(pool->getNumberOfThreads() , THREADS);
		EXPECT_EQ(pool->getQueueSize() , 0);
	}

	for (unsigned int i = 0; i < JOBS; i++) {
		jobDone[i] = false;
		ThreadPool::WorkFunction work;
		if (!stop)
			work = boost::bind(&workFunction,boost::arg<1>(),i);
		else
			work = boost::bind(&workFunctionStop,boost::arg<1>(),i);
		pool->addWork(work);
	}

	if (stop) {
		bool reqStop = requestStop();
		while (!reqStop) reqStop = requestStop.waitForUpdate(reqStop);
		pool->stop();
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(pool->isStopping() , true);
		EXPECT_EQ(pool->getQueueSize() , 0);
	}

	pool->waitForEmptyQueue();

	boost::mutex::scoped_lock lock(testMutex);
	EXPECT_EQ(pool->getQueueSize() , 0);

    if (stop) {
    	// We can only test the first and last jobs (and hope that we are lucky)
    	EXPECT_TRUE(jobDone[0]);
    	EXPECT_FALSE(jobDone[JOBS-1]);
    } else {
    	for (unsigned int i = 0; i < JOBS; i++) {
    		EXPECT_TRUE(jobDone[i]);
    	}
    }
    pool = NULL;

	return 0;
}

int runTaskEmpty() {
	boost::mutex::scoped_lock lock(testMutex);
	ThreadPool::Ptr dummyPool = ownedPtr(new ThreadPool(THREADS));
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));

	ThreadTask::Ptr task = ownedPtr(new ThreadTask(pool));

	EXPECT_EQ(task->getState() , ThreadTask::INITIALIZATION);
	EXPECT_EQ(task->getThreadPool() , pool);
	EXPECT_TRUE(task->setThreadPool(dummyPool));
	EXPECT_EQ(task->getThreadPool() , dummyPool);
	EXPECT_TRUE(task->setThreadPool(pool));
	EXPECT_EQ(task->getThreadPool(),pool);
	EXPECT_FALSE(task->keepAlive());
	task->setKeepAlive(true);
	EXPECT_TRUE(task->keepAlive());
	task->setKeepAlive(false);
	EXPECT_FALSE(task->keepAlive());
	EXPECT_EQ(task->getSubTasks().size(),0);

	ThreadTask::Ptr subtask1 = ownedPtr(new ThreadTask(task));
	EXPECT_TRUE(task->addSubTask(subtask1));
	EXPECT_EQ(task->getSubTasks().size(),1);

	task->setKeepAlive(true);
	EXPECT_TRUE(task->execute());
	EXPECT_FALSE(task->execute());
	EXPECT_NE(task->getState() , ThreadTask::INITIALIZATION);

	EXPECT_FALSE(task->setThreadPool(dummyPool));
	dummyPool = NULL;
	EXPECT_EQ(task->getThreadPool(),pool);

	ThreadTask::Ptr subtask2 = ownedPtr(new ThreadTask(task));
	EXPECT_TRUE(task->addSubTask(subtask2));
	EXPECT_EQ(task->getSubTasks().size(),2);
	task->setKeepAlive(true);

	ThreadTask::TaskState state;
	state = task->getState();
	while (state != ThreadTask::IDLE) state = task->wait(state);
	EXPECT_EQ(task->getState(),ThreadTask::IDLE);

	task->setKeepAlive(false);

	task->waitUntilDone();
	EXPECT_EQ(subtask1->getState(),ThreadTask::DONE);
	EXPECT_EQ(subtask2->getState(),ThreadTask::DONE);
	EXPECT_EQ(task->getState(),ThreadTask::DONE);

	return 0;
}

struct Event {
	typedef enum TaskType {
		MainTask,
		Task,
		SubTask
	} TaskType;
	typedef enum TaskFunction {
		Run,
		SubTaskDone,
		Idle,
		Done
	} TaskFunction;
	Event(TaskType type, TaskFunction function, unsigned int taskID, unsigned int subTaskID): type(type), function(function), taskID(taskID), subTaskID(subTaskID) {};
	TaskType type;
	TaskFunction function;
	unsigned int taskID;
	unsigned int subTaskID;
};
static boost::mutex eventMutex;
static std::vector<Event> events;

class SubTask: public ThreadTask {
public:
	SubTask(ThreadPool::Ptr pool, unsigned int taskID, unsigned int id): ThreadTask(pool), _taskID(taskID), _id(id) {};
	unsigned int getID() const { return _id; }
	void run() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Run,_taskID,_id));
		Timer::sleepMs(Random::ranI(0,25));
	};
	void subTaskDone(ThreadTask* subtask) { // should not be called!
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::SubTaskDone,_taskID,_id));
	};
	void idle() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Idle,_taskID,_id));
	};
	void done() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Done,_taskID,_id));
	};
private:
	const unsigned int _taskID;
	const unsigned int _id;
};

class Task: public ThreadTask {
public:
	Task(ThreadPool::Ptr pool, unsigned int id): ThreadTask(pool), _id(id), _idleTaskAdded(false) {}
	unsigned int getID() const { return _id; }
	void run() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Run,_id,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,0))),true);
		EXPECT_EQ(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,1))),true);
	};
	void subTaskDone(ThreadTask* subtask) {
		SubTask* stask = static_cast<SubTask*>(subtask);
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::SubTaskDone,_id,stask->getID()));
		}
		if (stask->getID() == 1) {
			boost::mutex::scoped_lock lock(testMutex);
			EXPECT_EQ(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,2))),true);
		}
	};
	void idle() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Idle,_id,0));
		}
		if (!_idleTaskAdded) {
			_idleTaskAdded = true;
			boost::mutex::scoped_lock lock(testMutex);
			EXPECT_EQ(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,3))),true);
		}
	};
	void done() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Done,_id,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,4))),false);
	};
private:
	const unsigned int _id;
	bool _idleTaskAdded;
};

class MainTask: public ThreadTask {
public:
	MainTask(ThreadPool::Ptr pool): ThreadTask(pool) {};
	void run() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::MainTask,Event::Run,0,0));
		}
		for(unsigned int i = 0; i < JOBS; i++) {
			ThreadTask::Ptr task = ownedPtr(new Task(getThreadPool(),i));
			boost::mutex::scoped_lock lock(testMutex);
			EXPECT_EQ(addSubTask(task),true);
		}
	};
	void subTaskDone(ThreadTask* subtask) {
		Task* task = static_cast<Task*>(subtask);
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::MainTask,Event::SubTaskDone,task->getID(),0));
	};
	void idle() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::MainTask,Event::Idle,0,0));
	};
	void done() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::MainTask,Event::Done,0,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_FALSE(addSubTask(ownedPtr(new Task(getThreadPool(),100))));
	};
};

int runTask() {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));
	ThreadTask::Ptr task = ownedPtr(new MainTask(pool));
	{
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_TRUE(task->execute());
		EXPECT_FALSE(task->execute());
		EXPECT_NE(task->getState() , ThreadTask::INITIALIZATION);
	}
	task->waitUntilDone();
	{
		boost::mutex::scoped_lock lock(testMutex);
		EXPECT_EQ(task->getSubTasks().size(),JOBS);
		EXPECT_EQ(task->getState(),ThreadTask::DONE);
	}

	std::vector<Event> eventList;
	{
		boost::mutex::scoped_lock lock(eventMutex);
		eventList = events;
	}

	// Verify the event order
	bool mainRun = false;
	bool mainSubTask[JOBS];
	bool mainIdle = false;
	bool mainDone = false;
	bool taskRun[JOBS];
	bool taskSubTask[JOBS][4];
	bool taskSubTaskNewAdded[JOBS];
	bool taskIdleFirst[JOBS];
	bool taskIdleSecond[JOBS];
	bool taskDone[JOBS];
	bool subTaskRun[JOBS][4];
	bool subTaskIdle[JOBS][4];
	bool subTaskDone[JOBS][4];
	for (unsigned int i = 0; i < JOBS; i++) {
		mainSubTask[i] = false;
		taskRun[i] = false;
		taskSubTaskNewAdded[i] = false;
		taskIdleFirst[i] = false;
		taskIdleSecond[i] = false;
		taskDone[i] = false;
		for (unsigned int j = 0; j < 4; j++) {
			taskSubTask[i][j] = false;
			subTaskRun[i][j] = false;
			subTaskIdle[i][j] = false;
			subTaskDone[i][j] = false;
		}
	}
	boost::mutex::scoped_lock lock(testMutex);
	for(const Event &event: eventList) {
		if (!mainRun) {
			if (event.type == Event::MainTask && event.function == Event::Run)
				mainRun = true;
			EXPECT_TRUE(mainRun);
		} else {
			EXPECT_LT(event.taskID , JOBS);
			EXPECT_LT(event.subTaskID , 4);
			if (event.function == Event::Run) {
				EXPECT_NE(event.type,Event::MainTask);
				if (event.type == Event::Task) {
					EXPECT_FALSE(taskRun[event.taskID]);
					taskRun[event.taskID] = true;
				} else if (event.type == Event::SubTask) {
					EXPECT_TRUE(taskRun[event.taskID]);
					EXPECT_FALSE(subTaskRun[event.taskID][event.subTaskID]);
					subTaskRun[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::SubTaskDone) {
				EXPECT_NE(event.type,Event::SubTask);
				if (event.type == Event::MainTask) {
					EXPECT_FALSE(mainSubTask[event.taskID]);
					EXPECT_TRUE(taskDone[event.taskID]);
					mainSubTask[event.taskID] = true;
				} else if (event.type == Event::Task) {
					EXPECT_TRUE(taskRun[event.taskID]);
					EXPECT_FALSE(taskSubTask[event.taskID][event.subTaskID]);
					EXPECT_TRUE(subTaskDone[event.taskID][event.subTaskID]);
					if (!taskSubTaskNewAdded[event.taskID]) {
						EXPECT_LT(event.subTaskID , 2);
						if (event.subTaskID == 1)
							taskSubTaskNewAdded[event.taskID] = true;
					} else {
						if (!taskIdleFirst[event.taskID]) {
							EXPECT_LT(event.subTaskID , 3);
						}
					}
					taskSubTask[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::Idle) {
				if (event.type == Event::MainTask) {
					for (unsigned int i = 0; i < JOBS; i++) {
						EXPECT_EQ(mainSubTask[i],true);
					}
					mainIdle = true;
				} else if (event.type == Event::Task) {
					if (!taskIdleFirst[event.taskID]) {
						EXPECT_EQ(taskSubTask[event.taskID][0],true);
						EXPECT_EQ(taskSubTask[event.taskID][1],true);
						EXPECT_EQ(taskSubTask[event.taskID][2],true);
						taskIdleFirst[event.taskID] = true;
					} else {
						EXPECT_EQ(taskIdleSecond[event.taskID],false);
						EXPECT_EQ(taskSubTask[event.taskID][3],true);
						taskIdleSecond[event.taskID] = true;
					}
				} else if (event.type == Event::SubTask) {
					EXPECT_EQ(subTaskIdle[event.taskID][event.subTaskID],false);
					EXPECT_EQ(subTaskRun[event.taskID][event.subTaskID],true);
					subTaskIdle[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::Done) {
				if (event.type == Event::MainTask) {
					EXPECT_EQ(mainDone,false);
					EXPECT_EQ(mainIdle,true);
					mainDone = true;
				} else if (event.type == Event::Task) {
					EXPECT_EQ(taskDone[event.taskID],false);
					EXPECT_EQ(taskIdleSecond[event.taskID],true);
					taskDone[event.taskID] = true;
				} else if (event.type == Event::SubTask) {
					EXPECT_EQ(subTaskDone[event.taskID][event.subTaskID],false);
					EXPECT_EQ(subTaskIdle[event.taskID][event.subTaskID],true);
					subTaskDone[event.taskID][event.subTaskID] = true;
				}
			}
		}
	}
	EXPECT_EQ(mainRun,true);
	EXPECT_EQ(mainIdle,true);
	EXPECT_EQ(mainDone,true);
	for (unsigned int i = 0; i < JOBS; i++) {
		EXPECT_EQ(mainSubTask[i],true);
		EXPECT_EQ(taskRun[i],true);
		EXPECT_EQ(taskSubTaskNewAdded[i],true);
		EXPECT_EQ(taskIdleFirst[i],true);
		EXPECT_EQ(taskIdleSecond[i],true);
		EXPECT_EQ(taskDone[i],true);
		for (unsigned int j = 0; j < 4; j++) {
			EXPECT_TRUE(taskSubTask[i][j]);
			EXPECT_TRUE(subTaskRun[i][j]);
			EXPECT_TRUE(subTaskIdle[i][j]);
			EXPECT_TRUE(subTaskDone[i][j]);
		}
	}

	return 0;
}

// Do a test where thread pool is allowed to finish all work (timeout = 5 seconds)
TEST(ThreadPoolTest,timed_test_5sec) {
	long start = rw::common::Timer::currentTimeMs();

	runPool(false);
	long end = rw::common::Timer::currentTimeMs();
	if(5000 < end-start) {
		EXPECT_NO_THROW(RW_THROW("TimeOut after 5sec, time was: " << (end-start)));
	}
}

// Do a test where thread pool is stopped before work is finished (timeout = 20 seconds)
TEST(ThreadPoolTest_Abort, timed_test_20sec) {
	long start = rw::common::Timer::currentTimeMs();

	runPool(true);
	long end = rw::common::Timer::currentTimeMs();
	if(20000 < end-start) {
		EXPECT_NO_THROW(RW_THROW("TimeOut after 20sec, time was: " << (end-start)));
	}
}

// Do a simple test with empty tasks
TEST(ThreadTaskTest, timed_test_5sec) {

	long start = rw::common::Timer::currentTimeMs();

	runTaskEmpty();
	long end = rw::common::Timer::currentTimeMs();
	if(5000 < end-start) {
		EXPECT_NO_THROW(RW_THROW("TimeOut after 5sec, time was: " << (end-start)));
	}

}

// Do a complex test where order of function calls is tested
TEST(ThreadTaskTest_InvocationOrder, times_test_20sec) { 
   
	long start = rw::common::Timer::currentTimeMs();

	runTask();
	long end = rw::common::Timer::currentTimeMs();
	if(20000 < end-start) {
		EXPECT_NO_THROW(RW_THROW("TimeOut after 20sec, time was: " << (end-start)));
	}

}

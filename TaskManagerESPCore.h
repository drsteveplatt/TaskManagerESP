#ifndef TASKMANAGERESPCORE_H_INCLUDED
#define TASKMANAGERESPCORE_H_INCLUDED

#include <TaskManagerCore.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//#include "Streaming.h"
//#include "ring.h"
//#include <setjmp.h>

/*! \file TaskManagerCore.h
    Header for Arduino TaskManager ESP communications library
*/


/*! \def TASKMGR_MESSAGE_SIZE
    The maximum size of a message passed between tasks

    This defines the maximum size for an inter-task message.  It is constrained, in part,
    by plans for RFI communication between tasks running on different devices.

    Note that the ESP-NOW's max payload size is 25.  Message overhead is 4 bytes.
*/

// TASKMGR_MESSAGE_SIZE calc:  esp-now packet size - 4 (cmd, from task/node) - 1 (first byte is to task)
// Note this is the user message size...
//#define ESP_NOW_MESSAGE_SIZE 250
//#define TASKMGR_MESSAGE_SIZE (ESP_NOW_MESSAGE_SIZE-4-1)
#define TASKMGR_MESSAGE_QUEUE_SIZE 10


/*!	\struct	_TaskManagerRadioPacket
	A packet of information being sent by radio between two TaskManager nodes
*/
// Note we reorder things becuase ESP32 uses 16 bit boundaries when packing things.
struct _TaskManagerRadioPacket {
	byte	m_cmd;							//!< Command information
	byte	m_fromTaskId;						// source task
	tm_nodeId_t	m_fromNodeId;						// source node
	byte	m_data[TASKMGR_MESSAGE_SIZE+1];	//! The data being transmitted.
		// reminder:  signal -> [0] is the signum
		//			  msg -> [0] is the taskID of the To, user data starts at [1]
		//			  suspend/resume -> [0] is the taskID to suspend/resume
};

// This is where we build MAC data for setting our MAC and pairing setup
// It has enough constant data that it is easier to just keep one around.
static byte _TaskManagerMAC[] = { 0xA6, 'T', 'M',  0, 0x00, 0x00 };

/**********************************************************************************************************/

/*! \class TaskManagerESP
    \brief Adds ESP-32 ESP-Now abilities to TaskManager

    Manages a set of cooperative tasks.  This includes round-robin scheduling, yielding, and inter-task
    messaging and signaling.  It also replaces the loop() function in standard Arduino programs.  Nominally,
    there is a single instance of TaskManager called TaskMgr.  TaskMgr is used for all actual task control.
	Each node has a nodeID.  nodeID=0 is 'the current node'.
    Each task has a taskID.  By convention, user tasks' taskID values are in the range [0 127].
*/

class TaskManagerESP: public TaskManager {

private:
	tm_nodeId_t		m_myNodeId;			// radio node number. 0 if radio not enabled.

public:
	// Constructor and destructor
	// Not included in doxygen documentation because the
	// user never constructs or destructs a TaskManager object
	/*! \brief Constructor,  Creates an empty task
	*/
    TaskManagerESP();
    /*!  \brief Destructor.  Destroys the TaskManager.

	    After calling this, any operations based on the object will fail.  For normal purpoases, destroying the
	    TaskMgr instance will have serious consequences for the standard loop() routine.
	*/
    ~TaskManagerESP();

public:

	/*!	@name Sending Signals and Messages

		These methods send signals or messages to other tasks running on this or other nodes
		@note If the nodeID is 0 on any routine that sends signals/messages to other nodes,
		the signal/message will be sent to this node.
	*/
	/*! @{ */

	/*!	\brief  Sends a signal to a task

		Sends a signal to a task running on a different node.  The signal will go to only one task.
		If there are several tasks on the node waiting on the signal, it will go to
		the first task found that is waiting for this particular signal.  Note that once a task is signalled, it will not be waiting for
		other instances of the same siggnal number.

		\param nodeId -- The node that is to receive the signal
		\param sigNum -- The value of the signal to be sent
		\sa yieldForSignal(), sendSignalAll(), addWaitSignal, addAutoWaitSignal()
	*/
	bool sendSignal(tm_nodeId_t nodeId, byte sigNum);

	/*! \brief Send a signal to all tasks that are waiting for this particular signal.

		Signals all tasks that are waiting for signal <i>sigNum</i>.
		\param sigNum -- The signal number to be sent
		\sa sendSignal(), yieldForSiganl(), addWaitSignal(), addAutoWaitSignal()
	*/

	/*!	\brief  Sends a signal to a task

		Sends a signal to a task running on this node.  The signal will go to only one task.
		If there are several tasks on the node waiting on the signal, it will go to
		the first task found that is waiting for this particular signal.  Note that once a task is signalled, it will not be waiting for
		other instances of the same siggnal number.

		\param sigNum -- The value of the signal to be sent
		\sa yieldForSignal(), sendSignalAll(), addWaitSignal, addAutoWaitSignal()
	*/
	bool sendSignal(byte sigNum) { sendSignal(0, sigNum); }

	/*! \brief Send a signal to all tasks on a different node that are waiting for this particular signal.

		Signals all tasks that are waiting for signal <i>sigNum</i>.
		\param sigNum -- The signal number to be sent
		\sa sendSignal(), yieldForSiganl(), addWaitSignal(), addAutoWaitSignal()
	*/

	bool sendSignalAll(tm_nodeId_t nodeId, byte sigNum);

	/*! \brief Send a signal to all tasks on this node that are waiting for this particular signal.

		Signals all tasks that are waiting for signal <i>sigNum</i>.
		\sa sendSignal(), yieldForSiganl(), addWaitSignal(), addAutoWaitSignal()
	*/

	bool sendSignalAll(byte sigNum) { sendSignalAll(0, sigNum); }

	/*! \brief  Sends a string message to a task

		Sends a message to a task running on a different node.  The message will go to only one task.

		Note that once a task has been sent a message, it will not be waiting for
		other instances of the same siggnal number.
		Note that additional messages sent prior to the task executing will overwrite any prior messages.
		Messages that are too large are ignored.  Remember to account for the trailing '\n'
		when considering the string message size.

		\param nodeId -- the node the message is sent to
		\param taskId -- the ID number of the task
		\param message -- the character string message.  It is restricted in length to
		TASKMGR_MESSAGE_LENGTH-1 characters.
		\sa yieldForMessage()
	*/
	bool sendMessage(tm_nodeId_t nodeId, byte taskId, char* message);

	/*! \brief  Sends a string message to a task

		Sends a message to a task running on this node.  The message will go to only one task.

		Note that once a task has been sent a message, it will not be waiting for
		other instances of the same siggnal number.
		Note that additional messages sent prior to the task executing will overwrite any prior messages.
		Messages that are too large are ignored.  Remember to account for the trailing '\n'
		when considering the string message size.

		\param taskId -- the ID number of the task
		\param message -- the character string message.  It is restricted in length to
		TASKMGR_MESSAGE_LENGTH-1 characters.
		\sa yieldForMessage()
	*/

	bool sendMessage(byte taskId, char* message) { sendMessage(0, taskId, message); }

	/*! \brief Send a binary message to a task

		Sends a message to a task on a different node.  The message will go to only one task.
		Note that once a task has been sent a message, it will not be waiting for
		other instances of the same signal number.  Messages that are too large are
		ignored.

		Note that additional messages sent prior to the task executing will overwrite any prior messages.
		\param taskId -- the ID number of the task
		\param buf -- A pointer to the structure that is to be passed to the task
		\param len -- The length of the buffer.  Buffers can be at most TASKMGR_MESSAGE_LENGTH
		bytes long.
		\sa yieldForMessage()
	*/

	bool sendMessage(tm_nodeId_t nodeId, byte taskId, void* buf, int len);

	/*! \brief Send a binary message to a task

		Sends a message to a task on this node.  The message will go to only one task.
		Note that once a task has been sent a message, it will not be waiting for
		other instances of the same signal number.  Messages that are too large are
		ignored.

		Note that additional messages sent prior to the task executing will overwrite any prior messages.
		\param taskId -- the ID number of the task
		\param buf -- A pointer to the structure that is to be passed to the task
		\param len -- The length of the buffer.  Buffers can be at most TASKMGR_MESSAGE_LENGTH
		bytes long.
		\sa yieldForMessage()
	*/

	bool sendMessage(byte taskId, void* buf, int len) { sendMessage(0, taskId, buf, len); }

	/*!	\brief Get source node/task of last message/signal

		Returns the nodeId and taskId of the node/task that last sent a signal or message
		to the current task.  If the current task has never received a signal, returns [0 0].
		If the last message/signal was from "this" node, returns fromNodeId=0.

		\param[out] fromNodeId -- the nodeId that sent the last message or signal
		\param[out] fromTaskId -- the taskId that sent the last message or signal
	*/
	void getSource(tm_nodeId_t& fromNodeId, byte& fromTaskId);

	/*! @} */

    /*!	@name Task Management */
    /*! @{ */

	/*!	\brief Suspend the given task on the given node

		Suspends a task on a different node.  If nodeID==0, it suspends a task on this node. If the node or task
		do not exist, nothing happens.  If the task was already suspended, it remains suspended.
		\param nodeId The node containing the task
		\param taskId The task to be suspended

		\note Not implemented.
		\sa resume()
	*/
	bool suspend(tm_nodeId_t nodeId, byte taskId);			// node, task

	/*!	\brief Suspend the given task on the given node

		Suspends a task on this node.  If nodeID==0, it suspends a task on this node. If the node or task
		do not exist, nothing happens.  If the task was already suspended, it remains suspended.
		\param nodeId The node containing the task
		\param taskId The task to be suspended

		\note Not implemented.
		\sa resume()
	*/
	bool suspend(byte taskId) { suspend(0, taskId); }

	/*!	\brief Resume the given task on the given node

		Resumes a task on another node.  If nodeID==0, it resumes a task on this node.  If the node or task
		do not exist, nothing happens.  If the task had not been suspended, nothing happens.
		\param nodeId The node containnig the task
		\param taskId The task to be resumed
		\note Not implemented.
		\sa suspend()
	*/
	bool resume(tm_nodeId_t nodeId, byte taskId);			// node, task

	/*!	\brief Resume the given task on the given node

		Resumes a task on this node.  If nodeID==0, it resumes a task on this node.  If the node or task
		do not exist, nothing happens.  If the task had not been suspended, nothing happens.
		\param nodeId The node containnig the task
		\param taskId The task to be resumed
		\note Not implemented.
		\sa suspend()
	*/
	bool resume(byte taskId) { resume(0, taskId); }


	/*! @} */


private:
	bool radioSender(tm_nodeId_t);	// generic packet sender

    // status requests/
    //void yieldPingNode(byte);					// node -> status (responding/not responding)
    //void yieldPingTask(byte, byte);				// node, task -> status
    //bool radioFree();						// Is the radio available for use?

    // radio
private:
	// notes on parameters to the commands
	//	signal, signalAll: m_data[0] = sigNum
	//  message: m_data[0] = taskID, m_data[1+] = message
	//  suspend, resume: m_data[0] = taskID
	enum RadioCmd {
		tmrNoop,			//!<	Do nothing
		tmrStatus,			//!<	Request status of this node.
		tmrAck,				//!<	Node status returned from a tmrStatus request
		tmrTaskStatus,		//!<	Request status of a task on this node
		tmrTaskAck,			//!<	Task status returned from a tmrTaskStatus request
		tmrSignal,			//!<	Send a signal
		tmrSignalAll,		//!<	Send a signal to all on this node
		tmrMessage,			//!<	Send a message
		tmrSuspend,			//!<	Suspend a task
		tmrResume			//!<	Resume a task
	};
	_TaskManagerRadioPacket	radioBuf;
	bool	m_radioReceiverRunning;
	esp_err_t m_lastESPError;

public:
	/*! \brief Radio receiver task for inter-node communication

		This receives radio messages and processes them.  It will transmit messages and signals as needed.

		This routine is for internal use only.
	*/
	void tmRadioReceiverTask();

	/*! \brief Create the radio and start it receiving

		Set up the ESP-32 ESP-Now configuration set our radio node ID.

		There are three variants:  radioBegin(nodeId), radioBegin(nodeId, ssId), radioBegin(nodeId, ssid, pw).

		The first is used when all of the nodes of a project are only using ESP-NOW.
		It closes the WiFi connection after ESP-NOW has been configured.

		The second is used if this node is ESP-NOW-only, but another node in the project (which this node is communicating with)
		is using WiFi.  It configures ESP-NOW to use the same channel as the WiFi node, but closes WiFi for this node.

		The third is used if this node is using both ESP-NOW and WiFi.  It configures both ESP-NOW and WiFi, acquires an IP address,
		and leaves WiFi open for later use.

		Note that additional messages sent prior to the task executing will overwrite any prior messages.

		\param nodeId -- the node the message is sent to
		\param ssid -- the ssid of the local WiFi node (if using WiFi in a project).
		\param pw -- the password of the local WiFi node (if using WiFi from this node).
	*/
	bool radioBegin(tm_nodeId_t nodeId, char* ssid=NULL, char* pw=NULL);

	/* \brief Add a peer for ESP-Now communications

		\param nodeID -- A peer node for future communications.
	*/
	bool registerPeer(tm_nodeId_t nodeId);

	/*	\brief Remove a peer from ESP-Now communications
		\param nodeID -- A peer node that will no longer be usable as a peer
	*/
	bool unRegisterPeer(tm_nodeId_t nodeId);

	/* \brief Get the last ESP error indicator

		Returns an esp_err_t value of the return status of the last completed ESP operation.
	*/
	esp_err_t lastESPError() { return m_lastESPError; }

	/* 	\brief Return last ESP error status

		\return The esp_err_t value of the last ESP call.  For normal operations, this will be ESP_OK.
		If an error had occurred, the error code will be returned.
	*/

	/*! @name Miscellaneous and Informational Routines */
	/*! @{ */

	/*!	\brief Return the node ID of this system.
		\return The byte value that is the current node's radio ID.  If the radio has not been
		enabled, returns 0.
	*/
    tm_nodeId_t myNodeId();
    /*! @} */
};

//
// Defining our global TaskMgr
//


//
// Inline stuff
//

inline tm_nodeId_t TaskManagerESP::myNodeId() {
	return m_myNodeId;
}


#endif // TASKMANAGERESPCORE_H_INCLUDED



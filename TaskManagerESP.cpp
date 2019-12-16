#define TASKMANAGER_MAIN

#include <arduino.h>
#include <TaskManagerCore.h>
#include <TaskManagerESPCore.h>


extern TaskManagerESP TaskMgr;

#include <Streaming.h>

/*! \file TaskManagerRF.cpp
    Implementation file for Arduino Task Manager
*/

// Configuration
//	Which WiFi channel to use
#define WIFI_CHANNEL 1

static void radioReceiverTask() {
	TaskMgr.tmRadioReceiverTask();
}

//
// Incoming message queue
//
// Design note:  TaskManager will be using _TaskManagerRadioPacket objects to transport data between
// nodes.  The message queue will send full packets and just absorb whatever returns.
// Also, the HAL will read/write arbitrary buffers.
// So:  The HAL will expect a uint8_t* buffer and a size.  It will send or receive it.  The high level
// routine will pass in the uint8_t* buffer and a size of sizeof(_TaskManagerRadioPacket).  When polling
// the message queue, it will provide a buffer and a short* in return, even though the short* will always
// receive sizeof(_TaskManagerRadioPacket)
// Note that the radio packet will contain a short nodeID and a byte taskID.
//

// Semaphore to coordinate queue calls (so we don't call add (from the receive callback) while we
// are in the middle of a remove).
static SemaphoreHandle_t _TaskManagerMessageQueueSemaphore;

class MessageQueue {
  private:
	_TaskManagerRadioPacket m_packets[TASKMGR_MESSAGE_QUEUE_SIZE];
	short m_lengths[TASKMGR_MESSAGE_QUEUE_SIZE];
	bool m_isEmpty;
	short m_head;	// oldest entry
	short m_tail;	// newest entry
  public:
  	MessageQueue(): m_isEmpty(true), m_head(0), m_tail(0) {};
	bool isEmpty() { return m_isEmpty; }
	bool add(const uint8_t* dat, const byte len);
	bool remove(uint8_t* dat, byte* len);
};
bool MessageQueue::add(const uint8_t* dat, const byte len) {
	// if we can't grab the semaphore in a ms, just ignore the message.
	if(xSemaphoreTake(_TaskManagerMessageQueueSemaphore,1000)==pdFALSE) return false;
    if(m_isEmpty) {
        m_isEmpty = false;
        memcpy(&m_packets[m_tail], dat, sizeof(len));
        m_lengths[m_tail] = len;
    } else if (m_head==((m_tail+1)%TASKMGR_MESSAGE_QUEUE_SIZE)) {
        Serial.print("<ignored>");
    } else {
        m_tail = (m_tail+1)%TASKMGR_MESSAGE_QUEUE_SIZE;
        memcpy(&m_packets[m_tail], dat, sizeof(len));
        m_lengths[m_tail] = len;
    }
    xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
};
bool MessageQueue::remove(uint8_t* dat, byte* len) {
	if(xSemaphoreTake(_TaskManagerMessageQueueSemaphore,1000)==pdFALSE) return false;
    if(m_isEmpty) {
		xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
        printf("<empty>");
        return false;
    } else if(m_head==m_tail) {
        memcpy(dat, &m_packets[m_head], sizeof(_TaskManagerRadioPacket));
        *len = m_lengths[m_head];
        m_isEmpty = true;
    } else {
        memcpy(dat, &m_packets[m_head], sizeof(_TaskManagerRadioPacket));
        *len = m_lengths[m_head];
        m_head = (m_head+1) % TASKMGR_MESSAGE_QUEUE_SIZE;
    }
    xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
    return true;
};

static MessageQueue _TaskManagerIncomingMessages;

// shared buf for MAC address; last two bytes are set to nodeID
static byte nodeMac[6] = { 0xA6, 'T', 'M', 0, 0, 0};

//
// Callbacks
//
static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus) {
	// We sent a message to the designated mac.  The message was sent with
	// sendStatus status.

	// for now, do nothing.
}

static void msg_recv_cb(const uint8_t *mac, const uint8_t* data, int len) {
	// We have received a message from the given MAC with the accompanying data.
	// Save the data in the "incoming message" queue
	// We don't use taskENTER_CRITICAL here because 'add' does it as needed.
	_TaskManagerIncomingMessages.add(data, len&0x0ff);
}
//
// Implementation of TaskManagerRF
//

// Constructor and Destructor


TaskManagerESP::TaskManagerESP() {
	m_myNodeId = 0;
	m_radioReceiverRunning = false;
	_TaskManagerMessageQueueSemaphore = xSemaphoreCreateBinary();
}

TaskManagerESP::~TaskManagerESP() {
	vSemaphoreDelete(_TaskManagerMessageQueueSemaphore);
}

// ***************************
//  All the world's radio code
// ***************************


// General purpose receiver.  Checks the message queue for delivered messages and processes the first one
void TaskManagerESP::tmRadioReceiverTask() {
	static byte len;
	// polled receiver -- if there is a packet waiting, grab and process it
	// receive packet from NRF24.  Poll and process messages
	// We need to find the destination task and save the fromNode and fromTask.
	// They are saved on the task instead of the TaskManager object in case several
	// messages/signals have been received.
	while(true) {
		if(_TaskManagerIncomingMessages.isEmpty()) {
			break;
		}
		// read a packet
		//m_rf24->read((void*)(&radioBuf), sizeof(radioBuf));
		_TaskManagerIncomingMessages.remove((uint8_t*)&radioBuf, &len);
		// process it
		switch(radioBuf.m_cmd) {
			case tmrNoop:
				break;
			case tmrStatus:	// NYI
				break;
			case tmrAck:	// NYI
				break;
			case tmrTaskStatus:	// NYI
				break;
			case tmrTaskAck:	// NYI
				break;
			case tmrSignal:
				internalSendSignal(radioBuf.m_fromNodeId, radioBuf.m_fromTaskId, radioBuf.m_data[0]);
				break;
			case tmrSignalAll:
				TaskManager::sendSignalAll(radioBuf.m_data[0]);
				break;
			case tmrMessage:
				internalSendMessage(radioBuf.m_fromNodeId, radioBuf.m_fromTaskId,
					radioBuf.m_data[0], &radioBuf.m_data[1], TASKMGR_MESSAGE_SIZE);
				break;
			case tmrSuspend:
				TaskManager::suspend(radioBuf.m_data[0]);
				break;
			case tmrResume:
				TaskManager::resume(radioBuf.m_data[0]);
				break;
		}
	}
}

// General purpose sender.  Sends a message somewhere (varying with the kind of radio)
bool TaskManagerESP::radioSender(short destNodeID) {

	nodeMac[4] = (destNodeID>>8)&0x0ff;
	nodeMac[5] = destNodeID&0x0ff;
	m_lastESPError = esp_now_send(nodeMac, (byte*)&radioBuf, sizeof(radioBuf)) == ESP_OK;
	return m_lastESPError == ESP_OK;
}

// If we have different radio receivers, they will have different instantiation routines.

bool TaskManagerESP::radioBegin(short nodeID) {
	// Initialize WiFi system
	WiFi.mode(WIFI_STA);
	nodeMac[4] = (nodeID>>8)&0x0ff;
	nodeMac[5] = nodeID & 0x0ff;
	m_lastESPError = esp_wifi_set_mac(ESP_IF_WIFI_STA, nodeMac);
	if(m_lastESPError!=ESP_OK) return false;
	WiFi.disconnect();

	m_lastESPError = esp_now_init();
	if(m_lastESPError!=ESP_OK) return false;

	delay(10);

	// register callbacks
	m_lastESPError = esp_now_register_recv_cb(msg_recv_cb);
	if(m_lastESPError!=ESP_OK) return false;
	m_lastESPError = esp_now_register_send_cb(msg_send_cb);
	if(m_lastESPError!=ESP_OK) return false;

	// final cleanup
	m_myNodeId = nodeID;
	m_radioReceiverRunning = true;
	return true;
}

bool TaskManagerESP::registerPeer(short nodeID) {
	// register the partner nodeID as a peer
	esp_now_peer_info_t peer;
	nodeMac[4] = (nodeID>>8)&0x0ff;
	nodeMac[5] = nodeID & 0x0ff;
	memcpy(peer.peer_addr, &nodeMac, 6);
	peer.channel = WIFI_CHANNEL;
	peer.ifidx = ESP_IF_WIFI_STA;
	peer.encrypt=false;
	m_lastESPError = esp_now_add_peer(&peer);
	return m_lastESPError==ESP_OK;
}

bool TaskManagerESP::sendSignal(short nodeId, byte sigNum) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendSignal(sigNum); return true; }
	radioBuf.m_cmd = tmrSignal;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = sigNum;
	return radioSender(nodeId);
}

bool TaskManagerESP::sendSignalAll(short nodeId, byte sigNum) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendSignalAll(sigNum); return true; }
	radioBuf.m_cmd = tmrSignalAll;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = sigNum;
	return radioSender(nodeId);
}

bool TaskManagerESP::sendMessage(short nodeId, byte taskId, char* message) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendMessage(taskId, message); return true; }
	radioBuf.m_cmd = tmrMessage;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;	// who we are sending it to
	if(strlen(message)>TASKMGR_MESSAGE_SIZE-2) {
		memcpy(&radioBuf.m_data[1], message, TASKMGR_MESSAGE_SIZE-2);
		radioBuf.m_data[TASKMGR_MESSAGE_SIZE-2]='\0';
	} else {
		strcpy((char*)&radioBuf.m_data[1], message);
	}
	return radioSender(nodeId);
}

bool TaskManagerESP::sendMessage(short nodeId, byte taskId, void* buf, int len) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendMessage(taskId, buf, len); return true; }
	if(len>TASKMGR_MESSAGE_SIZE) return false;	// reject too-long messages
	radioBuf.m_cmd = tmrMessage;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;	// who we are sending it to
	memcpy(&radioBuf.m_data[1], buf, len);
	return radioSender(nodeId);
}

bool TaskManagerESP::suspend(short nodeId, byte taskId) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::suspend(taskId); return true; }
	radioBuf.m_cmd = tmrSuspend;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;
	return radioSender(nodeId);
}

bool TaskManagerESP::resume(short nodeId, byte taskId) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::resume(taskId); return true; }
	radioBuf.m_cmd = tmrResume;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;
	return radioSender(nodeId);
}

void TaskManagerESP::getSource(short& fromNodeId, byte& fromTaskId) {
	fromNodeId = m_theTasks.front().m_fromNodeId;
	fromTaskId = m_theTasks.front().m_fromTaskId;
}



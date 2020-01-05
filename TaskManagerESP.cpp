#define TASKMANAGER_MAIN

#include <arduino.h>
#include <TaskManagerCore.h>
#include <TaskManagerESPCore.h>


// DEBUGGING
#define DEBUG false

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

static void dumpBuf(const uint8_t* buf, short len) {
	for(int i=0; i<16/*len*/; i++) {
		Serial << _HEX(buf[i]) << ' ';
		if(i%16 == 15) Serial << endl;
	}
}
static void dumpMac(const uint8_t* buf) {
	Serial << _HEX(buf[0]);
	for(int i=1; i<6; i++) Serial << ":" << _HEX(buf[i]);
}

static char* espErrText(esp_err_t err) {
	static char buf[20];
	if(err==ESP_OK) return "ESP_OK";
	else if(err==ESP_ERR_ESPNOW_ARG) return "ESP_ERR_ESPNOW_ARG";
	else if(err==ESP_ERR_ESPNOW_INTERNAL) return "ESP_ERR_ESPNOW_INTERNAL";
	else if(err==ESP_ERR_ESPNOW_IF) return "ESP_ERR_ESPNOW_IF";
	else {
		sprintf(buf,"ESP_ERR_UNKN_%d",(int)err);
		return buf;
	}
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
	short size() {
		if(isEmpty()) return 0;
		else if(m_tail<=m_head) return m_head-m_tail+1;
		else return (m_head+TASKMGR_MESSAGE_QUEUE_SIZE)-m_tail;
	}
};
bool MessageQueue::add(const uint8_t* dat, const byte len) {
	// if we can't grab the semaphore in a ms, just ignore the message.
	//if(DEBUG) Serial << "-->MessageQueue::add\n";
	if(xSemaphoreTake(_TaskManagerMessageQueueSemaphore,1000)==pdFALSE) {
	//	if(DEBUG) Serial << "Semaphore Take failed\n<--MessageQueue:add\n";
		return false;
	}
    if(m_isEmpty) {
        m_isEmpty = false;
        memcpy(&m_packets[m_tail], dat, len);
        m_lengths[m_tail] = len;
    } else if (m_head==((m_tail+1)%TASKMGR_MESSAGE_QUEUE_SIZE)) {
        Serial.print("<receive buffer full, incoming message ignored>\n");
    } else {
        m_tail = (m_tail+1)%TASKMGR_MESSAGE_QUEUE_SIZE;
        memcpy(&m_packets[m_tail], dat, len);
        m_lengths[m_tail] = len;
    }
    //if(DEBUG) Serial << "New packet added at " << m_tail << "len is " << len << endl;
    //if(DEBUG) dumpBuf((uint8_t*)&m_packets[m_tail], len);
    //if(DEBUG) Serial << endl;
    xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
    //if(DEBUG) Serial << "<--MessageQueue::add\n";
};
bool MessageQueue::remove(uint8_t* dat, byte* len) {
	if(DEBUG) Serial << "-->MessageQueue::remove\n";
	int t_at, t_len;
	if(xSemaphoreTake(_TaskManagerMessageQueueSemaphore,1000)==pdFALSE) return false;
    if(m_isEmpty) {
		xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
        printf("<empty>");
        return false;
    } else if(m_head==m_tail) {
        memcpy(dat, &m_packets[m_head], sizeof(_TaskManagerRadioPacket));
        *len = m_lengths[m_head];
        t_at = m_head; t_len = *len;
        m_isEmpty = true;
    } else {
        memcpy(dat, &m_packets[m_head], sizeof(_TaskManagerRadioPacket));
        *len = m_lengths[m_head];
        t_at = m_head; t_len = *len;
        m_head = (m_head+1) % TASKMGR_MESSAGE_QUEUE_SIZE;
    }
    xSemaphoreGive(_TaskManagerMessageQueueSemaphore);
    if(DEBUG) Serial << "Removed packet at " << t_at << " len is " << t_len
    	<< " new queue size " << size() << " isEmpty: " << (isEmpty() ? "True" : "False") << endl;
    if(DEBUG) Serial << "<--MessageQueue::remove\n";
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
	if(DEBUG) Serial << "-->msg_recv_cb\nreceived message\n";
	if(DEBUG) dumpBuf(data, len);
	if(DEBUG) Serial << endl;
	_TaskManagerIncomingMessages.add(data, len&0x0ff);
	if(DEBUG) Serial << "Queue is now " << (_TaskManagerIncomingMessages.isEmpty() ? " " : "not ") << "empty\n";
	if(DEBUG) Serial << "Queue size is now " << _TaskManagerIncomingMessages.size() << endl;
	if(DEBUG) Serial << "<--msg_recv_cb\n";
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
		if(DEBUG) Serial << "-->TaskManagerESP:tmRadioReceiverTask has a message\n";
		if(DEBUG) Serial << "Pulling message from queue\n";
		// read a packet
		//m_rf24->read((void*)(&radioBuf), sizeof(radioBuf));
		_TaskManagerIncomingMessages.remove((uint8_t*)&radioBuf, &len);
		if(DEBUG) Serial << "Received message, len is " << len << endl;
		//if(DEBUG) dumpBuf((uint8_t*)&radioBuf, len);
		//if(DEBUG) Serial << endl;
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
				if(DEBUG) Serial << "internalSendMessage from node/task" << radioBuf.m_fromNodeId
					<< "/" << radioBuf.m_fromTaskId
					<< " to task " << radioBuf.m_data[0] << endl;
				internalSendMessage(radioBuf.m_fromNodeId, radioBuf.m_fromTaskId,
					radioBuf.m_data[0], &radioBuf.m_data[1], TASKMGR_MESSAGE_SIZE);
				break;
			case tmrSuspend:
				TaskManager::suspend(radioBuf.m_data[0]);
				break;
			case tmrResume:
				TaskManager::resume(radioBuf.m_data[0]);
				break;
		} // end switch
		if(DEBUG) Serial << "<--TaskManager:tmRadioReceiverTask finished a message\n";
	}  // end while true
}

// General purpose sender.  Sends a message somewhere (varying with the kind of radio)
bool TaskManagerESP::radioSender(tm_nodeId_t destNodeID) {
	if(DEBUG) Serial << "-->radioSender, radioBuf len is " << sizeof(radioBuf) << endl;
	if(DEBUG) dumpMac(nodeMac);
	if(DEBUG) Serial << endl;
	//if(DEBUG) dumpBuf((byte*)&radioBuf, sizeof(radioBuf));
	nodeMac[4] = (destNodeID>>8)&0x0ff;
	nodeMac[5] = destNodeID&0x0ff;
	m_lastESPError = esp_now_send(nodeMac, (byte*)&radioBuf, sizeof(radioBuf));
	if(DEBUG) Serial << "Message sent, err is " << m_lastESPError << endl;
	if(DEBUG) Serial << "<--radioSender\n";
	if(m_lastESPError!=ESP_OK) Serial << "***ERR " << espErrText(m_lastESPError) << "***\n";
	return m_lastESPError == ESP_OK;
}

// If we have different radio receivers, they will have different instantiation routines.

bool TaskManagerESP::radioBegin(tm_nodeId_t nodeID) {
	// Initialize WiFi system
	if(DEBUG) Serial << "-->radioBegin\n";
	WiFi.mode(WIFI_STA);
	nodeMac[4] = (nodeID>>8)&0x0ff;
	nodeMac[5] = nodeID & 0x0ff;
	m_lastESPError = esp_wifi_set_mac(ESP_IF_WIFI_STA, nodeMac);
	if(m_lastESPError!=ESP_OK) return false;
	WiFi.disconnect();
	if(DEBUG) { Serial << "Registered my MAC as "; dumpMac(nodeMac); Serial << endl; }
	m_lastESPError = esp_now_init();
	if(m_lastESPError!=ESP_OK) return false;

	delay(10);

	// register callbacks
	m_lastESPError = esp_now_register_recv_cb(msg_recv_cb);
	if(m_lastESPError!=ESP_OK) return false;
	m_lastESPError = esp_now_register_send_cb(msg_send_cb);
	if(m_lastESPError!=ESP_OK) return false;

	// create our semaphore
	_TaskManagerMessageQueueSemaphore = xSemaphoreCreateMutex();

	// start our handler
	TaskMgr.add(TASKMGR_RF_MONITOR_TASK, radioReceiverTask);

	// final cleanup
	m_myNodeId = nodeID;
	m_radioReceiverRunning = true;
	if(DEBUG) Serial << "<--radioBegin (good exit)\n";
	return true;
}

bool TaskManagerESP::registerPeer(tm_nodeId_t nodeID) {
	// register the partner nodeID as a peer
	if(DEBUG) Serial << "-->registerPeer\n";
	esp_now_peer_info_t peer;
	nodeMac[4] = (nodeID>>8)&0x0ff;
	nodeMac[5] = nodeID & 0x0ff;
	memcpy(peer.peer_addr, &nodeMac, 6);
	peer.channel = WIFI_CHANNEL;
	peer.ifidx = ESP_IF_WIFI_STA;
	peer.encrypt=false;
	if(DEBUG) { Serial << "Registering "; dumpMac(peer.peer_addr); Serial << endl;}
	m_lastESPError = esp_now_add_peer(&peer);
	if(DEBUG) Serial << "Registration returned " << m_lastESPError << endl;
	if(DEBUG) Serial << "<--registerPeer\n";
	return m_lastESPError==ESP_OK;
}

bool TaskManagerESP::sendSignal(tm_nodeId_t nodeId, byte sigNum) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendSignal(sigNum); return true; }
	radioBuf.m_cmd = tmrSignal;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = sigNum;
	return radioSender(nodeId);
}

bool TaskManagerESP::sendSignalAll(tm_nodeId_t nodeId, byte sigNum) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::sendSignalAll(sigNum); return true; }
	radioBuf.m_cmd = tmrSignalAll;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = sigNum;
	return radioSender(nodeId);
}

bool TaskManagerESP::sendMessage(tm_nodeId_t nodeId, byte taskId, char* message) {
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

bool TaskManagerESP::sendMessage(tm_nodeId_t nodeId, byte taskId, void* buf, int len) {
	if(DEBUG) Serial << "-->TaskManager::sendMessage(1)\n";
	if(nodeId==0 || nodeId==myNodeId()) {
		TaskManager::sendMessage(taskId, buf, len);
		if(DEBUG) Serial << "sent to local node\n<--TaskManager::sendMessage(1)\n";
		return true;
	}
	if(DEBUG) Serial << "sending to remote node\n";
	if(len>TASKMGR_MESSAGE_SIZE) {
		if(DEBUG) Serial << "packet size (" << len << ") too large, >" << TASKMGR_MESSAGE_SIZE
				<< "\n<--TaskManager::sendMessage(1)\n";
		return false;	// reject too-long messages
	}
	radioBuf.m_cmd = tmrMessage;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;	// who we are sending it to
	memcpy(&radioBuf.m_data[1], buf, len);
	if(DEBUG) Serial << "sending message\n";
	bool ret = radioSender(nodeId);
	if(DEBUG) {
		Serial << "message send " << (ret ? "succeeded\n" : "failed\n");
		Serial << "<--TaskManager::sendMessage(1)\n";
	}
	return ret;
}

bool TaskManagerESP::suspend(tm_nodeId_t nodeId, byte taskId) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::suspend(taskId); return true; }
	radioBuf.m_cmd = tmrSuspend;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;
	return radioSender(nodeId);
}

bool TaskManagerESP::resume(tm_nodeId_t nodeId, byte taskId) {
	if(nodeId==0 || nodeId==myNodeId()) { TaskManager::resume(taskId); return true; }
	radioBuf.m_cmd = tmrResume;
	radioBuf.m_fromNodeId = myNodeId();
	radioBuf.m_fromTaskId = myId();
	radioBuf.m_data[0] = taskId;
	return radioSender(nodeId);
}

void TaskManagerESP::getSource(tm_nodeId_t& fromNodeId, byte& fromTaskId) {
	fromNodeId = m_theTasks.front().m_fromNodeId;
	fromTaskId = m_theTasks.front().m_fromTaskId;
}



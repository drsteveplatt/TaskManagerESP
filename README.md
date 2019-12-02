# TaskManagerESP
RF communications module for TaskManager

This module extends TaskManager to allow for communications between nodes.  It is based on 
the ESP-32 platform, and uses ESP-Now as its communications protocol.  A single node may
be paired with up to 20 other nodes.  Data packets up to 246 (est.) bytes may be transmitted
between nodes.

It adds routines to
- Initialize the network links.
- Send messages and signals to tasks on any node in the network

This library is documented within the TaskManager Wiki.

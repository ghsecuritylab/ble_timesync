**Version:** 0.1  
**Date:** 28.05.2019  
**Authors:** Stanley Nwabuona  
**Email:** nwabuonastanley@yahoo.com

## Preamble 

This document describes the services implemented in order to realize the demonstrator and outlines the planned interaction between the entities of the demonstrator. 

![System overview](/pictures/BLE_TIMESYNC_API_SEQ.png "Proposed Sequence Diagram of Demonstrator")


### Demonstrator entities

The demonstrator consists of three different entitites:
1. One Raspberry Pi Backend (RB): The RB backend is responsible for initiating and managing sensing campaigns , performing postprocessing calculations and storing the received and processed values in a database. The RB is connected to the MN via UART, which is used to send commands towards the MN and receive responses from the MN.  

2. One BLE Master Node (MN): The MN acts as a proxy between the SS and the RB. In contrast to the SS the MN is always powered on. It retains the available sensor nodes and manages 
the subscriptions of the RB regarding the available SS measurements. When RB is subscribed to a measurement, MN proactively obtains measurements from the nodes and pushes them towards the RB. Communication between RB and MN is realized via UART, the communication between MN and SS is realized via BLE. 

3. N (in the case of the demonstrator N=3) Sensor Slaves (SS): The sensor slaves are used to obtain current readings from different phases of a feeder. In contrast to the MN they are not constantley awake. During their sleep cycle they power an internal capacitor via the feeder. Once fully powered a SS will wake up and perform a measurement (triggered by the MN) of the current for a given sample size. The result is communicated back to the MN and the SS will go back to sleep until it's next wake period.


### Demonstrator sequence

The following section describes the sequence of actions underlying a demonstration as illustrated in the above diagram. This sequence will be used to derive the required services on the side of the MN and the SS to realize the demonstration. The services will be defined in more detail, after the sequence was outlined:  
1\. Upon starting the RB application, it will try to establish a UART connection with the MN.   
2\. Send an initialisation (init) command to MN.  
3\. The MN resets itself on receipt of the init command and returns a bool value (True/False).  
4\. The RB sends list of valid addresses, allowed to connect to the MN.  
5\. The MN implements a whitelist with the list in (4) and starts scanning.  
5\. The MN computes the list of connected devices that were awakened as a result of sensed current.  
5\. RB listens endlessly for updates on connected sensors.  
6\. Upon receipts of updates, RB synchronizes with MN through GPIO 27 (of the MN).  
7\. RB sends subscribe command to MN with mac address, required measurement and sampling size as arguments.  
7\. Upon receipt of subscribe command, MN configures corresponding services with sampling size and required measurements.  
8\. RB sends a start measurement command to MN while decrementing the timeout value.  
9\. MN sends a start measurement command to SS.  
9\. Upon receipt of start_measurement_command, SS starts advertising service.  
10\. RB listens for sensor values.  
10\. MN listens for updates from the configured service.  
11\.MN reads sensor values when update is made.  
12\.MN pushes sensor values to RB.  


### Required Services

#### MN
**Command_Service_BLE:** This BLE service will house all the commands between MN and SS.

#### SS
Current_Measurment_Service (TODO when we get information flow from siemens)

Voltage_Reading_Service

#### RB
**Command_Service_UART:** This API contains all the available commands (from RB to MN) as well as their corresponding responses.  
To ensure elegance, the commands are modelled after the **AT** command structure as shown in the table below.


| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**COMMAND** 	| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**EXPECTED_RESPONSE** 	| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**COMMAND EXPLANATION** 	|
|:-------------------------:	|:------------------------:	|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| <pre>**AT**</pre> 	| <pre>**OK**</pre> 	| Check if MN is connected via UART. 	|
| <pre>**AT+ADDR=<MAC_ADDRESS>**</pre> 	| <pre>**OK+1/Err**</pre> 	| The command gives the MN the list of valid MAC addresses. The addresses are sent one at a time in the standard BLE address format. **eg. XX:XX:XX:XX:XX:XX.**  The MN returns OK+1 to indicate that another address is expected and returns Err if another address is not seen after 1 second. 	|
| <pre>**AT+CONN=1**</pre> 	| <pre>**OK/Err**</pre> 	| Instructs MN to initiate connection with SS. 	|
| <pre>**AT+CONN=?**</pre> 	| <pre>**<LIST_OF_ADDR+(1/0)>/Err**</pre>   | Queries the MN for list of connected peripherals. Appending 0 to an address indicates that more addresses are expected while appending 1 indicates that the complete address is sent. 	|
| <pre>**AT+SUBS=<MAC_ADDRESS>+V/I**</pre> 	| <pre>**OK/Err**</pre> 	| Instructs the MN to subscribe to either voltage or current. 	|
| <pre>**ATMEAS**</pre> 	| <pre>**OK/Err**</pre> 	| Instructs the MN to start measuring. 	|
| <pre>**ATZ0**</pre> 	| <pre>**OK/Err**</pre> 	| Instructs the MN to reset. 	|
| <pre>**ATCONFIG=<I/V>,<X_Value>,<Y>,<Z>,<SAMPLE_SIZE>**</pre> 	| <pre>**OK/Err**</pre> 	| Subscription command from MN to SS. X, Y and Z values are hardcoded on the sensor nodes 	|

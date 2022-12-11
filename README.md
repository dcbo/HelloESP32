# HelloESP
Hello World Example for ESP32 Projects.

Start new Projects based on this Skeleton

# Functionality
* Wifi Connection  
* MQTT Connection 
* Self Monitoring connectivity and reconnect on connection loss
* Command Parser accepts commends over MQTT
* MQTT Status Topic, retained, with LastWill
* CRON System which sends different MQTT Topics every 10s, 30s and 60s
* Automatic Versioning System
  * Version Number is incremented after Upload to Production Target


# Available MQTT-Commands 
* Commands must be published to topic `[PREFIX]/cmd`
* Responses are published to `[PREFIX]/result`

## Hello-World Example MQTT-Commands
### `hello`
 Example:
 * command: `hello` 
 * result: `world`
 
### `helloadd NUM1 NUM2`
Add numbers NUM1 NUM2
 * NUM1: Unsigned Long Variable
 * NUM2: Unsigned Long Variable
 
Example:
 * command: `helloadd 40 2` 
 * result: `The Answer is: 42` 

### `helloecho STRING`
Echo String 
 * STRING: maximum of Buffersize Characters 

 Example:
 * command: `helloecho ECHOTEST` 
 * result: `ECHOTEST`


## System related Commands
### `reset`
Reboot ESP32

Example:
 * command: `reset` 
 * result: `T.B.D.`

/* 
 *  BC20_MQTT_Test.ino
 *  Copyright (c) 2021 Guan-Hsiung Liaw. All rights reserved.
 *  Written by Guan-Hsiung Liaw 
 *  
 *    This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  Description: 
 *    I define class "BC20" to be a wrapper of commonly used commands Quectel BC20 NB-IoT module.
 *    It is specifically designed for the experiments in MOE NB-IoT course. Only the functions related to 
 *    MQTT operations are considered. The provided example is to periodically get temperature data from 
 *    DHT22 sensor and publish it to the esignated MQTT broker. In addition, this example also subscribes 
 *    the messages published by itself.
 */

#include <SoftwareSerial.h>
#include <Arduino.h>
#include "IPAddress.h"
#include "DHT.h"

#define BC20_DEBUG
#define BC20_READ_TIMEOUT_OK     1000 //ms
#define BC20_READ_TIMEOUT_SECOND 8000  //ms
   
#define BC20_STATE_NOT_READY          -1
#define BC20_STATE_READY              0
#define BC20_STATE_MQTT_CLOSED        1
#define BC20_STATE_MQTT_OPENED        2
#define BC20_STATE_MQTT_CONNECTED     3
#define BC20_STATE_MQTT_DISCONNECTED  4

#define BC20_SUCCESS  0
#define BC20_FAIL     -1
#define BC20_ERROR_CHAR -2

#if defined(ESP8266) || defined(ESP32)
#include <functional>
#define MQTT_CALLBACK_SIGNATURE std::function<void(char*, uint8_t*, unsigned int)> mqtt_callback
#else
#define MQTT_CALLBACK_SIGNATURE void (*mqtt_callback)(char*, uint8_t*, unsigned int)
#endif

typedef enum {
  ECHO_ON = 0,
  ECHO_OFF,
  DISABLE_SLEEP_MODE,
  SET_BAND,
  ENABLE_UE_FUNCTION,
  ATTACH_NETWORK,
  DEATTACH_NETWORK,
  SET_PDP,
  MQTT_OPEN_CONNECTION,
  MQTT_CONNECT,
  MQTT_SUBSCRIBE,
  MQTT_UNSUBSCRIBE,
  MQTT_PUBLISH,
  MQTT_DISCONNECT,
  MQTT_CLOSE_CONNECTION
} eCommandType;

typedef struct {
  eCommandType  type;
  char  command_str[50];
  bool  more;
  char  more_str_header[15];
} CommandStruct;

class BC20 {

private:

  int   _state;
  bool  echo_on;
  IPAddress _ip;

  // Callback for incoming MQTT messages
  MQTT_CALLBACK_SIGNATURE;
  void setCallback(MQTT_CALLBACK_SIGNATURE);
  
  // SoftwareSerial
  SoftwareSerial *_serial;

  // BC20 Command parameter buffer
  uint8_t paramCount;
  char paramBuffer[512];
  char* putParam(char* input, uint8_t index);
  char* getParam(uint8_t index);

  // BC20 Response message buffer
  char respBuffer[512];
  int respLength;
  int getResponse(int timeout, int _case);
  int getEcho(char* input);

  // BC20 event message buffer
  char eventBuffer[256];
  int eventCount;
  char* eventWritePosition;
  char* eventReadPosition;
  int putEvent(char *);
  char *getNextEvent();
  void event_process(char *);

  // BC20 Commands
  CommandStruct commandList[15] = {
    {ECHO_ON, "ATE1", false, ""},
    {ECHO_OFF, "ATE0", false, ""},
    {DISABLE_SLEEP_MODE, "AT+QSCLK=0", false, ""},
    {SET_BAND, "AT+QBAND=1,8", false, ""},
    {ENABLE_UE_FUNCTION, "AT+CFUN=1", false, ""},
    {ATTACH_NETWORK, "AT+CGATT=1", false, ""},    // at first call, "+IP: ..." will be returned
    {DEATTACH_NETWORK, "AT+CGATT=0", false, ""},
    {SET_PDP, "AT+CGDCONT=1,\"IPV4V6\",,,0,0,,,,,0,0", false, ""},
    {MQTT_OPEN_CONNECTION, "AT+QMTOPEN=0,", true, "+QMTOPEN: "}, 
    {MQTT_CONNECT, "AT+QMTCONN=0,", true, "+QMTCONN: "},
    {MQTT_SUBSCRIBE, "AT+QMTSUB=0,1,", true, "+QMTSUB: "},
    {MQTT_UNSUBSCRIBE, "AT+QMTUNS=0,1,", true, "+QMTUNS: "},
    {MQTT_PUBLISH, "AT+QMTPUB=0,", true, "+QMTPUB: "}, 
    {MQTT_DISCONNECT, "AT+QMTDISC=0", true, "+QMTDISC= "},
    {MQTT_CLOSE_CONNECTION, "AT+QMTCLOSE=0", true, "+QMTCLOSE: "}
  };
  int sendCommand(eCommandType);

public:
  BC20(SoftwareSerial& serial);
  void setSoftwareSerial(SoftwareSerial& serial);
  int begin(int baud_rate);
  int disable_sleep_mode();
  int set_band();
  int enable_ue_function();
  int attach_network();
  int deattach_network();
  int set_pdp();
  void mqtt_set_callback(MQTT_CALLBACK_SIGNATURE);
  int mqtt_open(IPAddress, uint16_t);
  int mqtt_open(uint8_t*, uint16_t);
  int mqtt_open(const char*, uint16_t);
  int mqtt_connect(const char* client_id);
  int mqtt_connect(const char* client_id, const char* user, const char* pass);
  int mqtt_subscribe(const char* topic);
  int mqtt_subscribe(const char* topic, uint8_t qos);
  int mqtt_unsubscribe(const char* topic);
  int mqtt_publish(const char* topic, const char *payload);
  int mqtt_publish(const char* topic, const char *payload, uint8_t qos, boolean retain);
  int mqtt_disconnect();
  int mqtt_close();
  void loop();
  bool is_mqtt_connected();
  bool is_mqtt_opened();
  bool is_ready();
};

BC20::BC20(SoftwareSerial& serial) {
  this->_serial = NULL;
  this->_state = BC20_STATE_NOT_READY;
  this->echo_on = true;
  paramCount = 0;
  respLength = 0;
  eventCount = 0;
  eventReadPosition = eventBuffer;
  eventWritePosition = eventBuffer;
  this->mqtt_callback = NULL;
  setSoftwareSerial(serial);
}

void BC20::setCallback(MQTT_CALLBACK_SIGNATURE)
{
  this->mqtt_callback = mqtt_callback;
}

void BC20::setSoftwareSerial(SoftwareSerial& serial) {
  this->_serial = &serial;
}

char* BC20::putParam(char* input, uint8_t index)
{
    uint8_t i;
    if(index==0) paramCount = 0;
    char*   p = paramBuffer;
    for(i=0;i<index;i++) 
        p = p + strlen(p) + 1;
  sprintf(p, "%s", input);
  paramCount++;
  return p;
}

char* BC20::getParam(uint8_t index)
{
    uint8_t i;
    char*   p = paramBuffer;
    for(i=0;i<index;i++) 
        p = p + strlen(p) + 1;
    return p;
}

typedef enum {
   WAIT_CR1, WAIT_LF1, WAIT_CR2, WAIT_LF2, WAIT_CR3, WAIT_LF3, FINISH, FAIL
} eRespState;

int BC20::getEcho(char* input) 
{
  char c;
  unsigned long previous_time = millis();  
  int i, len;
  len = strlen(input);
  for(i=0;i<len;i++) {
    if(_serial->available()) {
      c = _serial->read();
      if(c!=input[i]) return BC20_FAIL;
    }
    else {
      if(millis()-previous_time > BC20_READ_TIMEOUT_OK) return BC20_FAIL;
    }
  }
  return BC20_SUCCESS;
}

int BC20::getResponse(int timeout, int _case)
{
  eRespState state = WAIT_CR1;
  bool is_timeout = false;
  unsigned long previous_time;
  char c;

  if(timeout>0) previous_time = millis();
  respLength = 0;
  respBuffer[0] = '\0';
  while(1) {
    if(_serial->available()) {
      c = _serial->read();
#ifdef BC20_DEBUG
      Serial.print(c);
#endif 
      if((state==WAIT_CR1 || state==WAIT_LF1 || state==WAIT_CR1 || state==WAIT_LF2 || state==WAIT_LF3) && (byte)c>=0x80) {
        return BC20_ERROR_CHAR;    
      }
      switch (state) {
        case WAIT_CR1:
          if(c=='\r') state = WAIT_LF1;
          else state = FAIL;
        break;
        case WAIT_LF1:
          if(c=='\n') state = WAIT_CR2;
          else state = FAIL;
        break;
        case WAIT_CR2:
          if(c=='\r') state = WAIT_LF2;
        break;
        case WAIT_LF2:
          if(c=='\n') {
            if(_case==0) state = FINISH; // for narmal commands
            else if(_case==1) state = WAIT_CR3; // for AT+QBAND
          }
          else state = FAIL;
        break;
        case WAIT_CR3:  // for AT+QBAND
          if(c=='\r') state = WAIT_LF3;
        break;
        case WAIT_LF3:  // for AT+QBAND
          if(c=='\n') state = FINISH; 
          else state = FAIL;
        default: {} 
      }
      if(c!='\r' && c!='\n') respBuffer[respLength++] = c;
      respBuffer[respLength] = '\0';
      if (state==FAIL) return BC20_FAIL;
      if (state==FINISH)  break;     
      
      if(timeout>0) previous_time = millis();
      is_timeout = false; 
    }
    else {
      if(timeout>0){
        if(millis()-previous_time > timeout) return BC20_FAIL;
      }
    }
  }
  return BC20_SUCCESS;
}

int BC20::sendCommand(eCommandType type) {
  char send_str[544], *p, *q;
  int i, ret;
  
#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("(): Start, ");
  Serial.println((int)type);
#endif
  // copy command string from commandList
  strcpy(send_str, commandList[type].command_str);
  
  // Appand parameters to command string
  if(paramCount>0) {
    for(i=0;i<paramCount-1;i++) {
      strcat(send_str, getParam(i));
      strcat(send_str, ",");      
    }
    strcat(send_str, getParam(i));
  }
  i = strlen(send_str);
  send_str[i] = '\r';
  send_str[i+1] = '\0';

  // send command via serial port
  _serial->print(send_str);
  
#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("().1: ");
  Serial.println(send_str);
#endif

  // if echo is on, get the echoed command string
  if(echo_on) getEcho(send_str);
  
  // get first response from BC20 (OK or Error)
  if(type==SET_BAND) ret = getResponse(BC20_READ_TIMEOUT_OK, 1);
  else if(type==ATTACH_NETWORK || type==DEATTACH_NETWORK) ret = getResponse(BC20_READ_TIMEOUT_SECOND, 0);
  else ret = getResponse(BC20_READ_TIMEOUT_OK, 0);

#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("().2: ");
  Serial.println(respBuffer);
#endif
  if(ret==BC20_ERROR_CHAR) {
    delay(100);
    _serial->flush();
  }
  if(ret!=BC20_SUCCESS) return BC20_FAIL;
  if(strstr(respBuffer, "OK")==NULL) return BC20_FAIL;
  if(!commandList[type].more) return BC20_SUCCESS;

  // get second reponse and trim the header
  bool correct=false;
  while(!correct) {
    if(getResponse(BC20_READ_TIMEOUT_SECOND, 0)!=BC20_SUCCESS) return BC20_FAIL;
    if(strncmp(respBuffer, commandList[type].more_str_header, strlen(commandList[type].more_str_header))==0) 
      correct = true;   
    else
      putEvent(respBuffer);
  }

#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("().3: ");
  Serial.println(respBuffer);
#endif

  p = respBuffer;
  q = respBuffer + strlen(commandList[type].more_str_header);
  i = strlen(respBuffer)-strlen(commandList[type].more_str_header);
  memmove(p, q, i);
  respBuffer[i] = '\0';
 
  // finalize
  paramCount = 0; 
  return BC20_SUCCESS;  
}

int BC20::begin(int baud_rate) {
  int ret, retry=3, i;
  char c;
  int timeout = 500, buf_len;
  unsigned long previous_time;
  char buf[15];
  bool is_ok;
  // Set echo off
  echo_on = -1;
  while(echo_on) {
    _serial->print("ATE0\r");
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().1: ");
    Serial.println("ATE0\r");
#endif
    buf_len = 0;
    previous_time = millis();
    while(1) {
      if(_serial->available()) {
        c = _serial->read();
#ifdef BC20_DEBUG
        Serial.print(c);
#endif        
        buf[buf_len++] = c;
        if(buf_len==14) break;
        previous_time = millis();
      }
      else {
        if(millis()-previous_time > timeout) break;
      }
    }
    buf[buf_len] = '\0';
    if(strstr(buf, "OK")!=NULL) echo_on = false;
    else delay(500);
  }
#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("().2: ");
  Serial.println(buf);
#endif

  // set baud rate
  sprintf(buf, "AT+IPR=%d\r", baud_rate);
  _serial->print(buf);
#ifdef BC20_DEBUG
  Serial.print(__FUNCTION__);
  Serial.print("().3: ");
  Serial.println(buf);
#endif
  buf_len = 0;
  is_ok = false;
  previous_time = millis();
  while(!is_ok) {
    while(1) {
      if(_serial->available()) {
        c = _serial->read();
#ifdef  BC20_DEBUG
        Serial.print(c);
#endif        
        buf[buf_len++] = c;
        if(buf_len==14) break;
        previous_time = millis();
      }
      else {
        if(millis()-previous_time > timeout) break;
      }
    }
    buf[buf_len] = '\0';
    if(strstr(buf, "OK")!=NULL) is_ok = true;
    else delay(500);
  }
   
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().4: ");
    Serial.println(buf);
#endif
  
  
  // Perform other initial actions
  for(i=0;i<retry;i++)
    if(disable_sleep_mode()==BC20_SUCCESS) break;
  if(i==retry) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().5: ");
    Serial.println("disable_sleep_mode(): pass");
#endif
  if(set_band()==BC20_FAIL) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().6: ");
    Serial.println("set_band(): pass");
#endif
  if(enable_ue_function()==BC20_FAIL) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().7: ");
    Serial.println("enable_ue_function(): pass");
#endif
  if(deattach_network()==BC20_FAIL) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().8: ");
    Serial.println("deattach_network(): pass");
#endif
  if(set_pdp()==BC20_FAIL) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().9: ");
    Serial.println("set_pdp(): pass");
#endif
  if(attach_network()==BC20_FAIL) return BC20_FAIL;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().10: ");
    Serial.println("attach_network(): pass");
#endif
  _state = BC20_STATE_READY;
  return BC20_SUCCESS;
}

int BC20::disable_sleep_mode()
{
  if(sendCommand(DISABLE_SLEEP_MODE)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

int BC20::set_band()
{
  if(sendCommand(SET_BAND)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

int BC20::enable_ue_function()
{
  if(sendCommand(ENABLE_UE_FUNCTION)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

int BC20::attach_network()
{
  if(sendCommand(ATTACH_NETWORK)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

int BC20::deattach_network()
{
  if(sendCommand(DEATTACH_NETWORK)==BC20_FAIL) return BC20_FAIL;
  _state = BC20_STATE_NOT_READY;
  return BC20_SUCCESS;
}

int BC20::set_pdp()
{
  if(sendCommand(SET_PDP)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

void BC20::mqtt_set_callback(MQTT_CALLBACK_SIGNATURE)
{
  this->mqtt_callback = mqtt_callback;
}

int BC20::mqtt_open(IPAddress ip, uint16_t port)
{
  char ipstr[16], portstr[6], *p;

  if(!is_ready()) return BC20_FAIL;

  // Send command
  sprintf(ipstr,"\"%d,%d,%d,%d\"", ip[0],ip[1],ip[2],ip[3]);
  sprintf(portstr,"%d", port);
  putParam(ipstr, 0);
  putParam(portstr, 1);
  if(sendCommand(MQTT_OPEN_CONNECTION)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>
  // <result> Result of the command execution
  //    -1 Failed to open network
  //    0 Opened network successfully
  //    1 Wrong parameter
  //    2 MQTT identifier is occupied
  //    3 Failed to activate PDP
  //    4 Failed to parse domain name
  //    5 Network disconnection error 
  p = strchr(respBuffer, ',');
  if(*(p+1)!='0') return BC20_FAIL;
  _state = BC20_STATE_MQTT_OPENED;
  return BC20_SUCCESS;
}

int BC20::mqtt_open(uint8_t* ip, uint16_t port)
{
  char ipstr[16], portstr[6], *p;

  if(!is_ready()) return BC20_FAIL;
  sprintf(ipstr,"\"%d,%d,%d,%d\"", ip[0],ip[1],ip[2],ip[3]);
  sprintf(portstr,"%d", port);
  putParam(ipstr, 0);
  putParam(portstr, 1);
  if(sendCommand(MQTT_OPEN_CONNECTION)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>
  // <result> Result of the command execution
  //    -1 Failed to open network
  //    0 Opened network successfully
  //    1 Wrong parameter
  //    2 MQTT identifier is occupied
  //    3 Failed to activate PDP
  //    4 Failed to parse domain name
  //    5 Network disconnection error 
  p = strchr(respBuffer, ',');
  if(*(p+1)!='0') return BC20_FAIL;

  _state = BC20_STATE_MQTT_OPENED;
  return BC20_SUCCESS;
  
}

int BC20::mqtt_open(const char* ip, uint16_t port)
{
  char ipstr[256], portstr[6], *p;

  if(!is_ready()) return BC20_FAIL;
  sprintf(ipstr,"\"%s\"", ip);
  sprintf(portstr,"%d", port);
  putParam(ipstr, 0);
  putParam(portstr, 1);
  if(sendCommand(MQTT_OPEN_CONNECTION)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>
  // <result> Result of the command execution
  //    -1 Failed to open network
  //    0 Opened network successfully
  //    1 Wrong parameter
  //    2 MQTT identifier is occupied
  //    3 Failed to activate PDP
  //    4 Failed to parse domain name
  //    5 Network disconnection error 
  p = strchr(respBuffer, ',');
  p++;
  if(*p !='0') return BC20_FAIL;

  _state = BC20_STATE_MQTT_OPENED;
  return BC20_SUCCESS;
}

int BC20::mqtt_connect(const char* client_id)
{
  char idstr[256], *p;

  if(!is_mqtt_opened()) return BC20_FAIL;
  sprintf(idstr,"\"%s\"", client_id);
  putParam(idstr, 0);
  if(sendCommand(MQTT_CONNECT)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>[,<ret_code>]
  // <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet
  //  <ret_code> Connect return code
  //    0 Connection Accepted
  //    1 Connection Refused: Unacceptable Protocol Version
  //    2 Connection Refused: Identifier Rejected
  //    3 Connection Refused: Server Unavailable
  //    4 Connection Refused: Bad User Name or Password
  //    5 Connection Refused: Not Authorized
  p = strchr(respBuffer, ',');
  p++;
  if(*p !='0') return BC20_FAIL;  // check <result>
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <ret_code>
  
  _state = BC20_STATE_MQTT_CONNECTED;
  return BC20_SUCCESS;
}

int BC20::mqtt_connect(const char* client_id, const char* user, const char* pass)
{
  char idstr[256], userstr[32], passstr[32], *p;

  if(!is_mqtt_opened()) return BC20_FAIL;
  sprintf(idstr,"\"%s\"", client_id);
  sprintf(userstr,"\"%s\"", user);
  sprintf(passstr,"\"%s\"", pass);
  putParam(idstr, 0);
  putParam(userstr, 1);
  putParam(passstr, 2);
  if(sendCommand(MQTT_CONNECT)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>[,<ret_code>]
  // <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet
  //  <ret_code> Connect return code
  //    0 Connection Accepted
  //    1 Connection Refused: Unacceptable Protocol Version
  //    2 Connection Refused: Identifier Rejected
  //    3 Connection Refused: Server Unavailable
  //    4 Connection Refused: Bad User Name or Password
  //    5 Connection Refused: Not Authorized
  p = strchr(respBuffer, ',');
  p++;
  if(*p !='0') return BC20_FAIL;  // check <result>
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <ret_code> 

  _state = BC20_STATE_MQTT_CONNECTED;
  return BC20_SUCCESS;
}

int BC20::mqtt_subscribe(const char* topic)
{
  char topicstr[256], *p;

  if(!is_mqtt_connected()) return BC20_FAIL;
  sprintf(topicstr,"\"%s\"", topic);
  putParam(topicstr, 0);
  putParam("0", 1); // default qos is 0
  if(sendCommand(MQTT_SUBSCRIBE)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<msgID>,<result>[,<value>]
  //  <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet  
  //  <value> 
  //    If <result> is 0: it is a vector of granted QoS levels. 
  //      At the same time, the value is 128, indicating that the subscription was rejected by the server.
  //    If <result> is 1: it means the times of packet retransmission.
  //    If <result> is 2: it will not be presented.
  p = strchr(respBuffer, ',');
  p++;
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 
  p = strchr(p, ',');
  p++;
  if(atoi(p)==128) return BC20_FAIL;  // check <value>
  
  return BC20_SUCCESS;
}

int BC20::mqtt_subscribe(const char* topic, uint8_t qos)
{
  char topicstr[256], qosstr[2], *p;

  if(!is_mqtt_connected()) return BC20_FAIL;
  sprintf(topicstr,"\"%s\"", topic);
  qosstr[0] = qos + '0';
  qosstr[1] = '\0';
  putParam(topicstr, 0);
  putParam(qosstr, 1); 
  if(sendCommand(MQTT_SUBSCRIBE)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<msgID>,<result>[,<value>]
  //  <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet  
  //  <value> 
  //    If <result> is 0: it is a vector of granted QoS levels. 
  //      At the same time, the value is 128, indicating that the subscription was rejected by the server.
  //    If <result> is 1: it means the times of packet retransmission.
  //    If <result> is 2: it will not be presented.
  p = strchr(respBuffer, ',');
  p++;
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 
  p = strchr(p, ',');
  p++;
  if(atoi(p)==128) return BC20_FAIL;  // check <value>
  
  return BC20_SUCCESS;
}

int BC20::mqtt_unsubscribe(const char* topic) 
{
  char topicstr[256], *p;

  if(!is_mqtt_connected()) return BC20_FAIL;
  sprintf(topicstr,"\"%s\"", topic);
  putParam(topicstr, 0);

  // Response: <tcpconnectID>,<msgID>,<result>
  //  <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet
  p = strchr(respBuffer, ',');
  p++;
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 
  
  if(sendCommand(MQTT_UNSUBSCRIBE)==BC20_FAIL) return BC20_FAIL;
  return BC20_SUCCESS;
}

int BC20::mqtt_publish(const char* topic, const char *payload)
{
  char topicstr[256], qosstr[2], payloadstr[256], *p;

  if(!is_mqtt_connected()) return BC20_FAIL;
  qosstr[0] = '0';
  qosstr[1] = '\0';
  sprintf(topicstr,"\"%s\"", topic);
  sprintf(payloadstr,"\"%s\"", payload);
  putParam(qosstr, 0); // msgid, must be 0 if qos is 0
  putParam(qosstr, 1); // qos
  putParam(qosstr, 2); // retain
  putParam(topicstr, 3);  // topic
  putParam(payloadstr, 4);  // payload
  if(sendCommand(MQTT_PUBLISH)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<msgID>,<result>[,<value>]
  //  <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet  
  p = strchr(respBuffer, ',');
  p++;
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 

  return BC20_SUCCESS;
}

int BC20::mqtt_publish(const char* topic, const char *payload, uint8_t qos, boolean retain)
{
  char topicstr[256], qosstr[2], retainstr[2], payloadstr[256], *p;

  if(!is_mqtt_connected()) return BC20_FAIL;
  sprintf(qosstr,"%s", qos);
  sprintf(retainstr,"%s", retain);
  sprintf(topicstr,"\"%s\"", topic);
  sprintf(payloadstr,"\"%s\"", payload);
  putParam(qosstr, 0); // msgid is set to be same as qos to prevent error
  putParam(qosstr, 1); // qos
  putParam(retainstr, 2); // retain
  putParam(topicstr, 3);
  putParam(payloadstr, 4);
  if(sendCommand(MQTT_PUBLISH)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<msgID>,<result>[,<value>]
  //  <result> Result of the command execution
  //    0 Sent packet successfully and received ACK from server
  //    1 Packet retransmission
  //    2 Failed to send packet  
  p = strchr(respBuffer, ',');
  p++;
  p = strchr(p, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 

  return BC20_SUCCESS;
  
}

int BC20::mqtt_disconnect()
{
  char *p;
  if(!is_mqtt_opened()) return BC20_FAIL;
  if(sendCommand(MQTT_DISCONNECT)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>
  //  <result> Result of the command execution
  //    -1 Failed to close connection
  //    0 Connection closed successfully
  p = strchr(respBuffer, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 

  _state = BC20_STATE_MQTT_CLOSED;
  return BC20_SUCCESS;
}

int BC20::mqtt_close()
{
  char *p;
  if(sendCommand(MQTT_CLOSE_CONNECTION)==BC20_FAIL) return BC20_FAIL;

  // Response: <tcpconnectID>,<result>
  //  <result> Result of the command execution
  //    -1 Failed to close connection
  //    0 Connection closed successfully
  p = strchr(respBuffer, ',');
  p++;
  if(*p != '0') return BC20_FAIL; // check <result> 

  _state = BC20_STATE_MQTT_CLOSED;
  return BC20_SUCCESS;
}

int BC20::putEvent(char *message)
{
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("(): Start, ");
    Serial.print(strlen(message));
    Serial.print(", ");
    Serial.println(message);
#endif
  
  // check if remained buffer size is enough to store message
  if(sizeof(eventBuffer)-(eventWritePosition-eventBuffer) < strlen(message)+1) {
#ifdef BC20_DEBUG
      Serial.print(__FUNCTION__);
      Serial.print("(): Not enough buffer, remained ");
      Serial.println(sizeof(eventBuffer)-(eventWritePosition-eventBuffer));
#endif
    return BC20_FAIL;
  }

  // store message to event buffer at eventWritePosistion
  strcpy(eventWritePosition, message);

  // update eventWritePosition
  eventWritePosition += (strlen(message) + 1);

  // increment eventCount
  eventCount++;
  return BC20_SUCCESS;
}

char* BC20::getNextEvent()
{
  char *message;
  if(eventCount==0) return NULL;
  message = eventReadPosition;
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("(): ");
    Serial.print(strlen(message));
    Serial.print(", ");
    Serial.println(message);
#endif
  eventReadPosition += (strlen(message)+1);
  eventCount--;
  if(eventCount==0) {
    eventReadPosition = eventBuffer;
    eventWritePosition = eventBuffer;
  }
  return message;
}

void BC20::event_process(char *message)
{
  int ret, i;
  char *p, *q, *topic;
  uint8_t *payload;
  unsigned int payload_length;

  if(strstr(message, "+IP: ")!=NULL) {
    p = message + 5;
    for(i=0;i<3;i++) {
      q = strchr(p, '.');
      *q = '\0';
      _ip[i] = atoi(p);
      p = q + 1;
    }
    _ip[3] = atoi(p);
#ifdef BC20_DEBUG
    Serial.print("IP Address: ");
    Serial.println(_ip);
#endif 
  }
  else if(strstr(message, "+QMTSTAT: ")!=NULL) {
    _state = BC20_STATE_MQTT_CLOSED;
  }
  else if(strstr(message, "+QMTRECV: ")!=NULL) {
    if(mqtt_callback) {
      p = message + 10;
      p = strchr(p, ',');
      p++;
      p = strchr(p, ',');
      p++;
      topic = p+1; // why increment 1? beacuse that there is a pair of " around the topic string
      p = strchr(p, ',');
      *(p-1) = '\0';
      p++;
      payload = (uint8_t*)(p+1);
      p = strchr(p, '\"');
      *p = '\0';
      payload_length = strlen((char *)payload);
      mqtt_callback(topic, payload, payload_length); 
    }     
  }  
}

void BC20::loop()
{ 
  int ret;
  char *message;
  
  // get incoming message from sofware serial
  while(1) {
    if(!_serial->available()) break;
    ret = getResponse(BC20_READ_TIMEOUT_OK, 0);

#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().from_serial: ");
    Serial.print(ret);
    Serial.print(", ");
    Serial.println(respBuffer);
#endif
  
    if(ret!=BC20_SUCCESS) continue;
  
    // parse incoming message
    event_process(respBuffer);
  }

  // processing messages in event buffer
  while((message=getNextEvent()) != NULL) {
#ifdef BC20_DEBUG
    Serial.print(__FUNCTION__);
    Serial.print("().from_evbuf: ");
    Serial.println(message);
#endif
    event_process(message);
  }
}

bool BC20::is_mqtt_connected()
{
  if(_state == BC20_STATE_MQTT_CONNECTED) return true;
  else return false;
}

bool BC20::is_mqtt_opened()
{
  if(_state == BC20_STATE_MQTT_OPENED) return true;
  else return false;
  
}

bool BC20::is_ready()
{
  if(_state == BC20_STATE_READY) return true;
  else return false;
}

// =======================================================================
/*
 * Arduino main program
 */

#define MQTT_BROKER_NAME  "140.127.196.119"
#define MQTT_BROKER_PORT  18883
#define MQTT_USER "netlab"
#define MQTT_PASSWORD "isuCSIE2021#"

#define RETRY_COUNT 3

#define DHTPIN A0
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial ss(3,11);
BC20 bc20(ss);
unsigned long last_loop_time;
unsigned long last_job_time;

void setup() {
  int i;
  
  dht.begin();
  
  // Initial Debug Serial port
  Serial.begin(115200);
  while(!Serial) {}

  // Initial Software Serial port and BC20 object
  ss.begin(38400);
  Serial.println("Initializing BC20 ....");
  bc20.begin(38400);
  while(!bc20.is_ready()) {
    Serial.print("Fail, wait 5 seconds ");
    for(i=0;i<5;i++) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println();
    bc20.begin(38400);
  }
  Serial.println("BC20 is successfully initialized!");
  last_loop_time = millis();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void loop() {
  int i, ret;
  float t;
  float h;
  char buf[10];
  char* topic1="test/temp";
  
  // bc20.loop() must be periodically run 
  if(millis()-last_loop_time>=100)
    bc20.loop();

  // set calback for MQTT messages
  bc20.mqtt_set_callback(callback);

  // Establish the connection to MQTT broker
  if(!bc20.is_mqtt_connected()) {
    char client_id[10];
    randomSeed(millis());
    sprintf(client_id,"test-%d", random(1000,10000));
    if(bc20.mqtt_open(MQTT_BROKER_NAME, MQTT_BROKER_PORT)==BC20_FAIL) { // Step 1. MQTT open
      Serial.println("MQTT open failed!");
      Serial.print("Wait 5 seconds to restart ");
      for(i=0;i<5;i++) {
        delay(1000);
        Serial.print(".");
      }
      Serial.println();
      return;
    }
    else
      Serial.println("MQTT open successful!");
    if(bc20.mqtt_connect(client_id, MQTT_USER, MQTT_PASSWORD)==BC20_FAIL) { // Step 2. MQTT connect
      Serial.println("MQTT connect failed!");
      Serial.print("Wait 5 seconds to restart ");
      for(i=0;i<5;i++) {
        delay(1000);
        Serial.print(".");
      }
      Serial.println();
      return;
    }
    else
      Serial.println("MQTT connect successful!");
    if(bc20.mqtt_subscribe("test/temp")==BC20_FAIL) { // Step 3. Subscribe topics
      Serial.println("MQTT subscribe failed!");
      Serial.print("Wait 5 seconds to restart ");
      for(i=0;i<5;i++) {
        delay(1000);
        Serial.print(".");
      }
      Serial.println();
      return;
    }
    else {
      Serial.println("MQTT subscribe successful!");
      last_job_time = millis(); 
    }
  }
  else {  // loop job
    if(millis()-last_job_time >= 30000) {
      last_job_time = millis();
      t = dht.readTemperature();
      h = dht.readHumidity();
      sprintf(buf,"%.2f,%.2f",t,h);
      Serial.print("MQTT Publish: ");
      Serial.print("topic=");
      Serial.print(topic1);
      Serial.print(", payload=");
      Serial.println(buf);
      ret = bc20.mqtt_publish(topic1, buf);
      
      /* Why retry after fail happened?
       *  This is because that some strange, unexpected characters will be received from the software serial connected to BC20.
       *  When such situation occurs, flush() function of the serial port will be called to clear the pending remained charaters 
       *  in the receiving buffer. Then ,the caller will get fail return. The caller can subsequently retry the command.
       *  Usually, the first retry can correctly be run. 
       */
      if(ret==BC20_FAIL) {
        for(i=0;i<RETRY_COUNT;i++) {  
          delay(1000);
          Serial.print("MQTT Publish failed, retry ");
          Serial.println(i+1);
          ret = bc20.mqtt_publish(topic1, buf);
          if(ret==BC20_SUCCESS) break;
        }
      }
    }
  }
  // delay(100);
}

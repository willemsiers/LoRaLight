/*

LoRa CSMA
Using Microchip RN2483 LoRa module.

 */
#include <SoftwareSerial.h>
#include "TimerOne.h"

#define BUFFER_SIZE 32
#define PIN_PIR A0
#define PIN_LED 13
#define PIN_MANUAL_RESET 7

#define DEBUG_DISCOVERY_ONLY false
#define LOG_VERBOSE true

//Status codes for receive_radio()
#define REC_SILENT 10
#define REC_RECEIVED 11
#define REC_ERROR 12

//number of LoRa modulation symbols before timeout
#define DEFAULT_RECEIVE_TIME 100

#define LIGHT_DURATION_INITIAL 3000
#define LIGHT_DURATION_INCREMENT 2000
#define HOPSLEFT_INITIAL 3

SoftwareSerial loraSerial(10, 11); // RX, TX

//base16 encoding/decoding
String str;
int length = 32;
char buffer[32];

//PIR and traffic detection
bool pir_state = LOW;
int pir_voltage = -1;
int traffic_ctr = 0;
int traffic_handled_ctr = 0;
int traffic_just_passed = false;

//traffic forwarding
int forward_light_duration = 0;
int forward_hopsleft = 0;

//LIGHT VARIABLES
long timestamp_turn_off_light = 0; //When the led needs to be turned off (again)

//DISCOVERY
int m_app_id = -1; //Id of this node in the network
int highest_id_in_network = -1; //Highest app_id known by this node

//
long timestamp_backoff = 0;

void pollPir()
{ //Interupt callback
  //Make sure this call is faster than the interupt time
  //TODO: maybe disable before sending receiving?
  pir_voltage = analogRead(PIN_PIR);
  bool pir_state_new = pir_voltage > 500;
  if (pir_state != pir_state_new) {
    if (pir_state_new == HIGH) {
      traffic_just_passed = true;
      traffic_ctr ++;
    } else {
      //just update state
    }
    pir_state = pir_state_new;
  }
}

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.setTimeout(1000);

  loraSerial.begin(57600);
  loraSerial.setTimeout(2000);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_MANUAL_RESET, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_MANUAL_RESET, LOW);
  delay(4);
  digitalWrite(PIN_MANUAL_RESET, HIGH);

 
  logA(F("Initing LoRa\r\n"));

  // char* radio_set_cmds[] = {"bt 0.5", "mod lora", "freq 869100000", "pwr -3", "sf sf12", "afcbw 41.7", "rxbw 25", "bitrate 50000", "fdev 25000", "prlen 8", "crc on", "iqi off", "cr 4/5", "wdt 0", "sync 12", "bw 125", NULL};

  send_cmd_blocking(F("sys get ver"));
  send_cmd_blocking(F("mac pause"));
  send_cmd_assert_ok(F("radio set bt 0.5"));
  send_cmd_assert_ok(F("radio set mod lora"));
  send_cmd_assert_ok(F("radio set freq 869100000"));
  send_cmd_assert_ok(F("radio set pwr -3"));
  send_cmd_assert_ok(F("radio set sf sf12"));
  send_cmd_assert_ok(F("radio set afcbw 41.7"));
  send_cmd_assert_ok(F("radio set rxbw 25"));
  send_cmd_assert_ok(F("radio set bitrate 50000"));
  send_cmd_assert_ok(F("radio set fdev 25000"));
  send_cmd_assert_ok(F("radio set prlen 8"));
  send_cmd_assert_ok(F("radio set crc on"));
  send_cmd_assert_ok(F("radio set iqi off"));
  send_cmd_assert_ok(F("radio set cr 4/5"));
  send_cmd_assert_ok(F("radio set wdt 0")); //watchdog time, disable for continuous reception
  send_cmd_assert_ok(F("radio set bw 125")); //bandwidth kHz
  logA(F("Initing LoRa done!\r\n"));

  loglnA(F("starting discovery..."));

  int highest_id_received = -1;
  for (int attempt = 0; highest_id_in_network == -1 && attempt < 1; attempt++) {
    logA("ATTEMPT : ");
    loglnA(String(attempt));
    int receive_status = receive_radio_for(NULL, DEFAULT_RECEIVE_TIME); //timeout means channel is silent
    if (receive_status == REC_SILENT)
    {
      // if (send_join_status)
      //{
      String discovery_buffer = String();
      bool wait_for_higher_id = true;

      for (int nothing_received_ctr = 0; nothing_received_ctr < 2; nothing_received_ctr++)
      {   
        logln(F("Sending disc high"));
        bool send_known_high_status = send_radio_blocking("disc high " + String(highest_id_received));

        int discovery_receive_status = receive_radio_for(&discovery_buffer, 100);
        if (discovery_receive_status == REC_RECEIVED)
        {
          loglnA(discovery_buffer);
          if (discovery_buffer.indexOf("disc high ") == 0 ) {
            highest_id_received = atoi(discovery_buffer.substring(10).c_str());
            highest_id_in_network = highest_id_received;
            nothing_received_ctr = 0;
          }
        } else {
          loglnA("No response to disc high, ctr=" + String(nothing_received_ctr));
        }
      }
    } else {
      logln(F("Channel wasn't silent"));
      //increment 'attempt'
    }
  }

  if (highest_id_in_network == -1)
  {
    loglnA(F("starting new network."));
    m_app_id = 0;
    highest_id_in_network = m_app_id;
  } else {
    int app_id_to_claim = highest_id_in_network + 1;

    String claim_app_id_msg = "disc high "; //todo: maybe use something like: "disc claim " ?
    claim_app_id_msg += String(app_id_to_claim);

    bool app_id_claim_confirmed = false;
    for (int broadcast_high_ctr = 0; broadcast_high_ctr < 2; broadcast_high_ctr++)
    {
      logA(F("Broadcasting my app_id: "));
      loglnA(String(app_id_to_claim));
      bool send_claim_status = send_radio_blocking(claim_app_id_msg);

      if (send_claim_status) {
        m_app_id = app_id_to_claim;
        highest_id_in_network = m_app_id;
      }
      delay(random(10, 100));
    }
  }
  /**
      String buffer_claim_response = String();
      int discovery_receive_status = receive_radio_for(&buffer_claim_response, 300);
      if (discovery_receive_status == REC_RECEIVED && buffer_claim_response.indexOf("disc claim_confirm") == 0)
      {
        app_id_claim_confirmed = true;
        m_app_id = app_id_to_claim; //new app_id isn't in response, maybe do that? atoi(buffer_claim_response.substring(19).c_str());
        highest_id_in_network = m_app_id;
      }
    } else {
      delay(random(30, 350));
      loglnA(F("radio error during claim!"));
    }
  }
  **/

  logA("STARTING WITH APP_ID=");
  loglnA(String(m_app_id));

#if !(DEBUG_DISCOVERY_ONLY)
  logA(F("Initing PIR and Interupt... "));
  Timer1.initialize(250000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(pollPir);  // attaches callback() as a timer overflow interrupt
  loglnA(F("Done initing interupt"));
#endif
}

int counter = 0;
void loop()
{ 
  if (traffic_just_passed) {
    //logln(("traffic ctr/handled == ") + String(traffic_ctr) + ("/") + String(traffic_handled_ctr));
    traffic_just_passed = false;

    traffic_handled_ctr = traffic_ctr;
    forward_hopsleft = max(forward_hopsleft, HOPSLEFT_INITIAL);
    forward_light_duration = max(forward_light_duration, LIGHT_DURATION_INITIAL);
    turn_on_light_for(LIGHT_DURATION_INITIAL);
  }

  //handle light
  if (timestamp_turn_off_light <= millis())
  {
    light_set(LOW);
  } else {
    light_set(HIGH);
  }

  //log("pir volt: ");
  //log(String(pir_voltage));
  //log("traffic_ctr: ");
  //logln(String(traffic_ctr));

  //Listening
  String receive_buffer = String();
  int status = receive_radio_for(&receive_buffer, DEFAULT_RECEIVE_TIME);

  if (status == REC_RECEIVED)
  {
    log("Received :\t");
    logln(receive_buffer);

    if (receive_buffer.indexOf("disc high ") == 0)
    {
      //HANDLE DISCOVERY MESSAGES
      //Be quiet, unless receiving a "disc high " message with a LOWER id than my m_app_id
      //Also respond initially (to the "join") if I have the highest id
      loglnA("received DISC HIGH!" );
      loglnA("highest_id_in_network=" + String(highest_id_in_network) + "  m_app_id=" + String(m_app_id));

      int received_highest_id = atoi(receive_buffer.substring(10).c_str());
      if ( received_highest_id > highest_id_in_network)
      {
        highest_id_in_network = received_highest_id;
      } else if (received_highest_id < highest_id_in_network)
      {
        if ( m_app_id == highest_id_in_network) {
          /**my id is higher than what someone else thinks is the highest. Correct this ONLY if I'm the highest id in the network, so that not everyone with a higher id will respond at the same time.
          Uncaught case is when someone that thinks it is highest does not receive this message correctly. In that case it breaks.
          **/
          String msg_highest_correction = "disc high ";
          msg_highest_correction += String(highest_id_in_network);
          bool highest_correction_status = send_radio_blocking(msg_highest_correction);
          if (!highest_correction_status)
          {
            loglnA(F("Error in discovery correction response!"));
          } else {
            loglnA("Responded : " + msg_highest_correction);
          }
        } else {
          //let someone else correct it (highest id only)
          return; //go into listening mode again
        }
      }

    } else if (receive_buffer.indexOf("rts") == 0)
    {
      //HANDLE NORMAL MESSAGES
      int rts_target_id = atoi( receive_buffer.substring(4).c_str() );
      if (rts_target_id == m_app_id)
      {
        //TODO check here if rts is intended for me or someone else (identifier), i.e. am i the destination
        //Maybe also check source of the rts
        logln(F("received rts, sending cts response..."));
        bool cts_status = send_radio_blocking("cts");
        if (cts_status) {
          log("Sending cts done");
          String receive_buffer_data = String();
          int data_status = receive_radio_for(&receive_buffer_data, DEFAULT_RECEIVE_TIME);
          if (data_status == REC_RECEIVED) {
            logA(F("received some data: "));
            loglnA(receive_buffer_data);

            bool rts_status = send_radio_blocking("ack");

            if (receive_buffer_data.indexOf("traffic ") == 0)
            {

              logA("Received traffic message with time/hopcnt: ");
              //handle traffic detected on previous node
              // first change string with new data (hopcount and time duration), and store this in a to_send queue.
              //Also turn on light here.
              //messages_to_forward

              int received_duration_millis = atoi(receive_buffer_data.substring(8, receive_buffer_data.lastIndexOf(" ")).c_str());
              int received_hopsleft = atoi(receive_buffer_data.substring(receive_buffer_data.lastIndexOf(" ") ).c_str());

              turn_on_light_for(received_duration_millis);

              forward_light_duration = max(forward_light_duration, received_duration_millis);
              forward_hopsleft = max(forward_hopsleft, received_hopsleft);

              loglnA(String(received_duration_millis) + " \t" + String(received_hopsleft));
              turn_on_light_for(received_duration_millis);

            }


          } else {
            logln(F("receiving data failed"));
          }
        } else {
          logln(F("Failed sending cts over radio"));
        }
      } else {
        //rts not for me
        delay(400);//random(200,400));
      }
    } else {
      log("Expected rts, instead received: ");
      logln(receive_buffer);
    }
  } else if (status == REC_SILENT && timestamp_backoff <= millis()) {
    if (forward_hopsleft > 0)
    {
      if (highest_id_in_network > m_app_id)
      {
        bool forward_succeeded = try_send_traffic_message(forward_light_duration + LIGHT_DURATION_INCREMENT, forward_hopsleft - 1);
        if (forward_succeeded) {
          forward_hopsleft = 0;
        } else {
          loglnA(F("Failed forwarding detected traffic."));
          timestamp_backoff = millis() + random(10, 100);
        }
      } else {
        forward_hopsleft = 0;
      }
    }
  } else {
    //status is REC_ERROR
  }

  //status == false, so try sending own traffic if neccesary
  if (traffic_handled_ctr != traffic_ctr) {


  }
}

//Note: never decreases timestamp_turn_off_light, only increases
void turn_on_light_for(long millis_duration)
{
  long millis_now = millis();
  long millis_left = timestamp_turn_off_light - millis_now;
  if ( millis_duration > millis_left )
  {
    timestamp_turn_off_light = millis_duration + millis_now;
  }
  light_set(HIGH);
}

void light_set(bool mode) //mode=HIGH/LOW
{
  digitalWrite(PIN_LED, mode);
}

bool try_send_traffic_message(long duration_millis, int hopsleft)
{
  String msg_traffic = "traffic ";
  msg_traffic += String(duration_millis);
  msg_traffic += " ";
  msg_traffic += String(hopsleft);
  bool forward_succeeded = send_msg_radio(m_app_id + 1, msg_traffic); //dst=myId+1
  return forward_succeeded;
}

//send message over radio with sensing
bool send_msg_radio(int destination, String msg)
{
  logA(F("sending msg to: "));
  logA(String(destination));
  logA(F("\t: "));
  loglnA(msg);

  bool result = false;

  String rts_message = "rts ";
  rts_message += String(destination);
  bool rts_status = send_radio_blocking(rts_message);
  log("rts sending worked? ");
  logln(String(rts_status));

  if (!rts_status) {
    result = false;
  } else {
    String buffer_rts = String();
    int cts_status = receive_radio_for(&buffer_rts, DEFAULT_RECEIVE_TIME);
    logln(String(cts_status));
    if (cts_status == REC_RECEIVED) {
      log("cts received.");
      //RTS should be responded to with CTS at this point
      //If buffer_rts is something else, then probably collision
      if (buffer_rts.indexOf("cts") != -1)
      {
        //Should be able to transmit data now
        //Still chance for future collisions in case the RTS/CTS were not properly received/processed by EVERYONE
        bool data_send_status = send_radio_blocking(msg);
        if (data_send_status) {
          logln(F("Transmission complete, waiting for ACK now"));
          String buffer_ack = String();
          int ack_status = receive_radio_for(&buffer_ack, DEFAULT_RECEIVE_TIME);
          int rts_status = send_radio_blocking("ack");
          if (ack_status == REC_RECEIVED) {
            if (buffer_ack.indexOf("ack") != -1)
            {
              loglnA(F("ACK RECEIVED!"));
              result = true;
            } else {
              logA("Expected ACK, got: ");
              loglnA(buffer_ack);
            }
          } else {
            logln(F("Receiving ack failed"));
          }
        } else {
          logln(F("Sending data failed"));
        }
      } else {
        log("Expected cts, got: ");
        logln(buffer_rts);
        result = false;
      }
    } else {
      result = false;
    }
  }
  return result;
}

//put radio into receive mode for a certain time, and receive a line into receive_buffer
int receive_radio_for(String * receive_buffer, int receive_time_millis) {
  if (receive_buffer == NULL) {
    receive_buffer = &str;
  }
  int result = -1;
  int receive_mode_time = receive_time_millis;
  String cmd = "radio rx ";
  cmd += String(receive_mode_time);
  String status = send_cmd_blocking(cmd);
  if (status.indexOf("ok") == 0)
  {
    //delay(receive_read_delay); Not sure if "radio tx 1000" blocks or not
    *receive_buffer = loraSerial.readStringUntil('\n');  //ok, invalid_param, busy
    logln("receive_radio buffer: " + *receive_buffer);
    if (*receive_buffer == "") {
      //not sure when this happens, maybe watchdog timeout?
      logln(F(">>>>>>>>>>>>>>>>>>>>receive buffer == \"\", probably TIMED OUT? <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"));
      result = REC_RECEIVED;
    } else if ((*receive_buffer).indexOf("radio_err") == 0)
    {
      //logln(F("receive buffer == \"radio_err\", (Error/Timeout)"));
      result = REC_SILENT; //TODO: This case happens both when there is a timeout, AND when it simply failed to receive... How to differentiate?
    } else if ((*receive_buffer).indexOf("radio_rx") == 0)
    {
      //remove heading "readio_rx" and only keep data, then decode it
      *receive_buffer = base16decode((*receive_buffer).substring(9));
      log("data only: ");
      logln(*receive_buffer);
      result = REC_RECEIVED;
    } else {
      logln(F("ERROR, no such response?"));
      result = REC_ERROR;
    }
  } else {
    logA(F("Receive status not OK, but: "));
    loglnA(status);
    result = REC_ERROR;
    delay(200);
  }
  return result;
}

//put radio into receive mode for a certain time, and receive a line into receive_buffer
int receive_radio(String * receive_buffer) {
  return receive_radio_for(receive_buffer, 100);
}

//send text over the radio and wait for completion
//to_send: plaintext message
bool send_radio_blocking(String to_send) {
  bool result = 0;
  String command = "radio tx ";
  command += base16encode(to_send);
  //command = "radio tx 48656c6C6F"; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //log("send_radio_blocking full command: ");
  //logln(command);
  loraSerial.println(command);
  String status = loraSerial.readStringUntil('\n');  //ok, invalid_param, busy
  //log(F("send_radio status: "));
  //logln(status);
  if (status.indexOf("ok") == 0)
  {
    status = loraSerial.readStringUntil('\n');  //radio_tx_ok, radio_tx_err
    //logln( "send_radio_blocking: attempt status " + String(status));
    if (status.indexOf("radio_tx_ok") == 0 )
    {
      result = true;
    } else {
      logA("Send ERROR: ");
      loglnA(String(status));
      result = false;
    }
  } else {
    result = false;
    logln("ERR: radio tx status: " + status);
  }
  return result;
}

//send command to radio module
String send_cmd_blocking(String cmd)
{
  log("CMD: ");
  log(cmd.c_str());
  loraSerial.println(cmd);
  String response = loraSerial.readStringUntil('\n');

  log(" \tRESPONSE: ");
  log(response.c_str());
  if (response == "") {
    logln(F("(Timed out)"));
  } else {
    logln(F(""));
  }
  return response;
}

void send_cmd_assert_ok(String cmd)
{
  String status = send_cmd_blocking(cmd);
  if (status.indexOf("ok") == 0) {
    return;
  } else {
    loglnA("ERROR: Status of command was " + String(status) + " instead of \"ok\"");

  }
}

//logAll
void logA(String s) {
  Serial.write(s.c_str());
}

void log(String s) {
#if LOG_VERBOSE
  Serial.write(s.c_str());
#endif
}

void logln(String s) {
  log(s);
  log("\n");
}

void loglnA(String s) {
  logA(s);
  logA(F("\n"));
}

String base16encode(String input)
{
  char charsOut[BUFFER_SIZE * 2];
  char charsIn[BUFFER_SIZE];
  input.trim();
  input.toCharArray(charsIn, BUFFER_SIZE);

  int i = 0;
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    if (charsIn[i] == '\0') break;

    int value = int(charsIn[i]);

    sprintf(buffer, "%02x", value);
    charsOut[2 * i] = buffer[0];
    charsOut[2 * i + 1] = buffer[1];
  }
  charsOut[2 * i] = '\0';
  String toReturn = String(charsOut);
  return toReturn;
}

String base16decode(String input)
{
  char charsIn[BUFFER_SIZE * 2];
  char charsOut[BUFFER_SIZE];
  input.trim();
  input.toCharArray(charsIn, BUFFER_SIZE * 2);

  int i = 0;
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    if (charsIn[i * 2] == '\0') break;
    if (charsIn[i * 2 + 1] == '\0') break;

    char toDo[2];
    toDo[0] = charsIn[i * 2];
    toDo[1] = charsIn[i * 2 + 1];
    int out = strtoul(toDo, 0, 16);

    if (out < 128)
    {
      charsOut[i] = char(out);
    }
  }
  charsOut[i] = '\0';
  return charsOut;
}

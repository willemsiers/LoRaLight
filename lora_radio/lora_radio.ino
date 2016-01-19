/*

LoRa CSMA
Using Microchip RN2483 LoRa module.

 */
#include <SoftwareSerial.h>
#include "TimerOne.h"

#define BUFFER_SIZE 32
#define PIN_PIR A0

#define DEBUG_DISCOVERY_ONLY true
#define LOG_VERBOSE true 

SoftwareSerial loraSerial(10, 11); // RX, TX

//base16 encoding/decoding
String str;
int length = 32;
char buffer[32];

//PIR and traffic detection
bool pir_state = LOW;
int pir_voltage = -1;
bool traffic_detected = false;
int traffic_ctr = 0;
int traffic_handled_ctr = 0;

//Discovery
int m_app_id = -1; //Id of this node in the network
int highest_id_in_network = -1; //Highest app_id known by this node

void pollPir()
{ //Interupt callback
  //Make sure this call is faster than the interupt time
  //TODO: maybe disable before sending receiving?
  pir_voltage = analogRead(PIN_PIR);
  bool pir_state_new = pir_voltage > 500;
  if (pir_state != pir_state_new) {
    if (pir_state_new == HIGH) {
      traffic_detected = true;
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
  loraSerial.setTimeout(10000);

  logA(F("Initing LoRa\r\n"));

  // char* radio_set_cmds[] = {"bt 0.5", "mod lora", "freq 869100000", "pwr -3", "sf sf12", "afcbw 41.7", "rxbw 25", "bitrate 50000", "fdev 25000", "prlen 8", "crc on", "iqi off", "cr 4/5", "wdt 0", "sync 12", "bw 125", NULL};

  str = send_cmd_blocking("sys get ver");
  str = send_cmd_blocking("mac pause");
  str = send_cmd_blocking(F("radio set bt 0.5"));
  str = send_cmd_blocking(F("radio set mod lora"));
  str = send_cmd_blocking(F("radio set freq 869100000"));
  str = send_cmd_blocking(F("radio set pwr -3"));
  str = send_cmd_blocking(F("radio set sf sf12"));
  str = send_cmd_blocking(F("radio set afcbw 41.7"));
  str = send_cmd_blocking(F("radio set rxbw 25"));
  str = send_cmd_blocking(F("radio set bitrate 50000"));
  str = send_cmd_blocking(F("radio set fdev 25000"));
  str = send_cmd_blocking(F("radio set prlen 8"));
  str = send_cmd_blocking(F("radio set crc on"));
  str = send_cmd_blocking(F("radio set iqi off"));
  str = send_cmd_blocking(F("radio set cr 4/5"));
  str = send_cmd_blocking(F("radio set wdt 0")); //watchdog time, disable for continuous reception
  //str = send_cmd_blocking("radio set sync 12")); for lorawan
  str = send_cmd_blocking(F("radio set bw 125")); //bandwidth kHz
  logA(F("Initing LoRa done!\r\n"));

  loglnA(F("starting discovery..."));


  for (int attempt = 0; highest_id_in_network == -1 && attempt < 3; attempt++) {
    bool channel_silent = receive_radio_for(NULL, 50); //timeout means channel is silent
    if (channel_silent)
    {
      logln(F("Channel was silent, try sending \"join\""));
      bool send_join_status = send_radio_blocking("join");
      if (send_join_status)
      {
        logln(F("sent join success"));
        String discovery_buffer = String();
        int highest_id_received = -1;
        bool wait_for_higher_id = true;

        delay(100);
        //bool send_initial_high = send_radio_blocking("disc high -1"); //TODO:

        while (wait_for_higher_id)
        {
          wait_for_higher_id = false;
          bool discovery_receive_status = receive_radio_for(&discovery_buffer, 100);
          loglnA("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
          if (discovery_receive_status)
          {
            String discovery_buffer_decoded = base16decode(discovery_buffer);
            loglnA(discovery_buffer_decoded);
            if (discovery_buffer_decoded.indexOf("disc high ") == 0 ) {
              highest_id_received = atoi(discovery_buffer_decoded.substring(10).c_str());
              highest_id_in_network = highest_id_received;
              wait_for_higher_id = true;
            }
          }

        }
      } else {
        loglnA(F("Radio error in discovery!"));
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
    logA(F("Try claiming app_id: "));
    loglnA(String(app_id_to_claim));

    String claim_app_id_msg = "disc claim "; //todo: maybe use something like: "disc claim " ?
    claim_app_id_msg += String(app_id_to_claim);

    bool app_id_claim_confirmed = false;
    while (!app_id_claim_confirmed)
    {
      bool send_claim_status = send_radio_blocking(claim_app_id_msg);

      if (send_claim_status) {
        String buffer_claim_response = String();
        bool discovery_receive_status = receive_radio_for(&buffer_claim_response, 300);
        String buffer_claim_response_decoded = base16decode(buffer_claim_response);
        if (discovery_receive_status && buffer_claim_response_decoded.indexOf("disc claim_confirm") == 0)
        {
          app_id_claim_confirmed = true;
          m_app_id = app_id_to_claim; //new app_id isn't in response, maybe do that? atoi(buffer_claim_response.substring(19).c_str());
          highest_id_in_network = m_app_id;
        }
      } else {
        loglnA(F("radio error during claim!"));
      }
    }

  }

  logA("STARTING WITH APP_ID=");
  loglnA(String(m_app_id));
  return;

#if !(DEBUG_DISCOVERY_ONLY)
  logA(F("Initing PIR + Interupt... "));
  pinMode(PIN_PIR, INPUT);
  Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(pollPir);  // attaches callback() as a timer overflow interrupt
  loglnA(F("Done initing interupt"));
#endif
}

int counter = 0;
void loop()
{
#if !(DEBUG_DISCOVERY_ONLY)
  log("pir volt: ");
  log(String(pir_voltage));
  log("traffic_ctr: ");
  logln(String(traffic_ctr));

  //return;
  if (traffic_handled_ctr != traffic_ctr) {
    int hops_remaining = 1;
    int time_on = 4;
    String msg_traffic = "traffic ";
    msg_traffic += String(hops_remaining - 1);
    msg_traffic += " ";
    msg_traffic += String(time_on);
    bool forward_succeeded = send_msg_radio(5, msg_traffic); //dst=myId+1
    if (forward_succeeded) {
      traffic_handled_ctr ++;
    }
  }
#endif

  //Listening
  String receive_buffer = String();
  bool status = receive_radio(&receive_buffer);
  log("status: ");
  logln(String(status));

  if (status)
  {
    String receive_buffer_decoded = base16decode(receive_buffer);
    log("Received :\t");
    logln(receive_buffer_decoded);
    if (receive_buffer_decoded.indexOf("join") == 0) {
      //HANDLE DISCOVERY MESSAGES
      //Be quiet, unless receiving a "disc high " message with a LOWER id than my m_app_id
      //Also respond initially (to the "join") if I have the highest id
      loglnA("received JOIN!" );

      if (m_app_id == highest_id_in_network)
      {
        //I am the highest id, so send initial "disc high " response
        bool respond_to_join_status = false;
        while (respond_to_join_status == false)
        {
          String msg_join_response = "disc high ";
          msg_join_response += String(highest_id_in_network);
          respond_to_join_status = send_radio_blocking(msg_join_response);
          if (!respond_to_join_status)
          {
            loglnA(F("Error in discovery join response, Trying again"));
            delay(150);
          }
        }
      }

      //TODO LOOP HERE?
      bool join_listen_status = receive_radio_for(&receive_buffer, 1000); //TODO: correct timing
      if ( join_listen_status ) {
        String receive_buffer_decoded = base16decode(receive_buffer);
        if (receive_buffer_decoded.indexOf("disc high ") == 0)
        {
          int received_highest_id = atoi(receive_buffer_decoded.substring(10).c_str());
          if ( received_highest_id > highest_id_in_network)
          {
            highest_id_in_network = received_highest_id;
          } else if (received_highest_id < highest_id_in_network && m_app_id == highest_id_in_network)
          {
            /**my id is higher than what someone else thinks is the highest. Correct this ONLY if I'm the highest id in the network, so that not everyone with a higher id will respond at the same time.
            Uncaught case is when someone that thinks it is highest does not receive this message correctly. In that case it breaks.
            **/
            String msg_highest_correction = "disc high ";
            msg_highest_correction += String(highest_id_in_network);
            bool highest_correction_status = send_radio_blocking(msg_highest_correction);
            if (!highest_correction_status)
            {
              loglnA(F("Error in discovery correction response!"));
            }
          }
        } else if (receive_buffer_decoded.indexOf("disc claim ") == 0) {
          String msg_claim_confirm = F("disc claim_confirm");
          highest_id_in_network = atoi(receive_buffer_decoded.substring(11).c_str());
          bool claim_confirm_status = send_radio_blocking(msg_claim_confirm);
          if (!claim_confirm_status)
          {
            loglnA(F("Error in confirming discovery claim."));
          }
        } else {
          //nothing received(, done?)
        }
      }
      else if (receive_buffer_decoded.indexOf("rts") == 0)
      {
        //HANDLE NORMAL MESSAGES
        int rts_target_id = atoi( receive_buffer_decoded.substring(4).c_str() );
        if (rts_target_id == m_app_id)
        {
          //TODO check here if rts is intended for me or someone else (identifier), i.e. am i the destination
          //Maybe also check source of the rts
          logln(F("received rts, sending cts response..."));
          bool cts_status = send_radio_blocking("cts");
          if (cts_status) {
            log("Sending cts done");
            String receive_buffer_data = String();
            bool data_status = receive_radio(&receive_buffer_data);
            if (data_status) {
              logA(F("received some data: "));

              String recieve_buffer_data_decoded = base16decode(receive_buffer_data);
              loglnA(recieve_buffer_data_decoded);

              bool rts_status = send_radio_blocking("ack");
            } else {
              logln(F("receiving data failed"));
            }
          } else {
            logln(F("Failed sending cts over radio"));
          }
        }
      } else {
        log("Expected rts, instead received: ");
        logln(receive_buffer_decoded);
      }
    }
  }
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
  rts_message += destination;
  bool rts_status = send_radio_blocking(rts_message);
  log("rts sending worked? ");
  logln(String(rts_status));

  if (!rts_status) {
    result = false;
  } else {
    String buffer_rts = String();
    bool cts_status = receive_radio(&buffer_rts);
    log("cts listening worked? ");
    logln(String(cts_status));
    if (cts_status) {
      if (buffer_rts != "") {

        String buffer_rts_decoded = base16decode(buffer_rts);
        //RTS should be responded to with CTS at this point
        //If buffer_rts is something else, then probably collision
        if (buffer_rts_decoded.indexOf("cts") != -1)
        {
          //Should be able to transmit data now
          //Still chance for future collisions in case the RTS/CTS were not properly received/processed by EVERYONE
          logln(F("Start sending data now!!!"));
          bool data_send_status = send_radio_blocking(msg);
          if (data_send_status) {
            logln(F("Transmission complete, waiting for ACK now"));
            String buffer_ack = String();
            bool ack_status = receive_radio(&buffer_ack);
            String buffer_ack_decoded = base16decode(buffer_ack);
            bool rts_status = send_radio_blocking("ack");
            if (ack_status) {
              if (buffer_ack_decoded.indexOf("ack") != -1)
              {
                loglnA(F("ACK RECEIVED!"));
                result = true;
              } else {
                log("Expected ACK, got: ");
                logln(buffer_ack);
              }
            } else {
              logln(F("Receiving ack failed"));
            }
          } else {
            logln(F("Sending data failed"));
          }
        } else {
          log("Expected cts, got: ");
          logln(buffer_rts_decoded);
          result = false;
        }
      } else {
        logln(F("Rts timed out, Expected Cts but got Nothing"));
      }
    } else {
      result = false;
    }
  }
  return result;
}

//put radio into receive mode for a certain time, and receive a line into receive_buffer
bool receive_radio_for(String * receive_buffer, int receive_time_millis) {
  if (receive_buffer == NULL) {
    receive_buffer = &str;
  }
  bool result = 0;
  int receive_mode_time = receive_time_millis;
  //int receive_read_delay = receive_mode_time + 500;
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
      logln(F("receive buffer == \"\", probably TIMED OUT?"));
    } else if ((*receive_buffer).indexOf("radio_err") == 0)
    {
      logln(F("receive buffer == \"radio_err\", (Error/Timeout)"));
    } else if ((*receive_buffer).indexOf("radio_rx") == 0)
    {
      *receive_buffer = (*receive_buffer).substring(9); //remove heading "readio_rx" and only keep data
      log("data only: ");
      logln(*receive_buffer);
    } else {
      logln(F("ERROR, no such response?"));
    }
    result = true;
  } else {
    result = false;
  }
  return result;
}

//put radio into receive mode for a certain time, and receive a line into receive_buffer
bool receive_radio(String * receive_buffer) {
  return receive_radio_for(receive_buffer, 100);
}

//send text over the radio and wait for completion
//to_send: plaintext message
bool send_radio_blocking(String to_send) {
  bool result = 0;
  String command = "radio tx ";
  command += base16encode(to_send);
  //command = "radio tx 48656c6C6F"; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  log("send_radio_blocking full command: ");
  logln(command);
  loraSerial.println(command);
  String status = loraSerial.readStringUntil('\n');  //ok, invalid_param, busy
  log(F("send_radio status: "));
  logln(status);
  if (status.indexOf("ok") == 0)
  {
    do {
      status = loraSerial.readStringUntil('\n');  //radio_tx_ok, radio_tx_err
      //logln(F("send_radio_blocking: attempt status " + status));
    } while (status.indexOf("radio_tx_ok") != 0);

    result = true;
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

void send_cmd_assert_ok(const char* cmd)
{
  String status = send_cmd_blocking(cmd);
  if (status.indexOf("ok") == 0) {
    return;
  } else {
    do {
      logln("Status of command was " + String(status) + " instead of \"ok\"");
      delay(1000);
    } while (true);

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

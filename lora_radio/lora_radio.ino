/*
Basic LoRa CSMA

Based on JP Meijers works

Polling network, using Microchip RN2483 LoRa module.

 */
#include <SoftwareSerial.h>
#define BUFFER_SIZE 32

SoftwareSerial loraSerial(10, 11); // RX, TX

String str;
int length = 32;
char buffer[32];

void setup()
{
 //output LED pin
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.setTimeout(10000);
  loraSerial.begin(57600);
  loraSerial.setTimeout(10000);

  Serial.print("Initing LoRa\r\n");

  str = send_cmd_blocking("sys get ver");
  str = send_cmd_blocking("mac pause");
  str = send_cmd_blocking("radio set bt 0.5");
  str = send_cmd_blocking("radio set mod lora");
  str = send_cmd_blocking("radio set freq 869100000");
  str = send_cmd_blocking("radio set pwr 14");
  str = send_cmd_blocking("radio set sf sf12");
  str = send_cmd_blocking("radio set afcbw 41.7");
  str = send_cmd_blocking("radio set rxbw 25");
  str = send_cmd_blocking("radio set bitrate 50000");
  str = send_cmd_blocking("radio set fdev 25000");
  str = send_cmd_blocking("radio set prlen 8");
  str = send_cmd_blocking("radio set crc on");
  str = send_cmd_blocking("radio set iqi off");
  str = send_cmd_blocking("radio set cr 4/5");
  str = send_cmd_blocking("radio set wdt 0"); //watchdog time, disable for continuous reception
  //str = send_cmd_blocking("radio set sync 12"); for lorawan
  str = send_cmd_blocking("radio set bw 125"); //bandwidth kHz

  //delay(1000000);

}

const bool sending = true;
void loop()
{
  delay(50);
  if(sending)
  {
    //Sending
    Serial.println("======================================================");
    bool status = send_msg_radio(5, "helloworld");
    Serial.write("Status: ");
    Serial.println(status);
    delay(8000);
  }else
  {
    //Listening
    String receive_buffer = String();
    bool status = receive_radio(&receive_buffer);
    Serial.write("status: ");
    Serial.println(status);
    //return; ////////////////////////////////////////////////////////////////////////////////////
    if(status)
    {
      String receive_buffer_decoded = base16decode(receive_buffer);
      Serial.write("Received :\t");
      Serial.println(receive_buffer_decoded);
      if(receive_buffer_decoded.indexOf("rts") != -1)
      {
        //TODO check here if rts is intended for me or someone else (identifier), i.e. am i the destination
        //Maybe also check source of the rts
        Serial.println("received rts, sending cts response...");
        bool cts_status = send_radio_blocking("cts");
        if(cts_status){
          Serial.write("Sending cts done"); 
          String receive_buffer_data = String();
          bool data_status = receive_radio(&receive_buffer_data);
          if(data_status){
            Serial.write("received some data: ");

            String recieve_buffer_data_decoded = base16decode(receive_buffer_data);
            Serial.println(recieve_buffer_data_decoded);

            bool rts_status = send_radio_blocking("ack");
          }else{
            Serial.println("receiving data failed");
          }
        }else{
          Serial.println("Failed sending cts over radio");
        }
      }else{
        Serial.write("Expected rts, instead received: ");
        Serial.println(receive_buffer_decoded);
      }
    }
  }
}

//send message over radio with sensing
bool send_msg_radio(int destination, String msg)
{
  bool result = false;

  String rts_message = "rts";
  rts_message += destination;
  bool rts_status = send_radio_blocking(rts_message);
  Serial.write("rts sending worked? ");
  Serial.println(rts_status);
  //return false; ////////////////////////////////////////////////////////////////////////////////////
  
  if (!rts_status) {
    result = false;
  } else {
    String buffer_rts = String();
    bool cts_status = receive_radio(&buffer_rts);
    Serial.write("cts listening worked? ");
    Serial.println(cts_status);
    if (cts_status) {
      if (buffer_rts != "") {

        String buffer_rts_decoded = base16decode(buffer_rts);
        //RTS should be responded to with CTS at this point
        //If buffer_rts is something else, then probably collision
        if (buffer_rts_decoded.indexOf("cts") != -1)
        {
          //Should be able to transmit data now
          //Still chance for future collisions in case the RTS/CTS were not properly received/processed by EVERYONE
          Serial.println("Start sending data now!!!");
          bool data_send_status = send_radio_blocking(msg);
          if(data_send_status){
            Serial.println("Transmission complete, waiting for ACK now");
            String buffer_ack = String();
            bool ack_status = receive_radio(&buffer_ack);
            String buffer_ack_decoded = base16decode(buffer_ack);
            bool rts_status = send_radio_blocking("ack");
            if(ack_status){
              if(buffer_ack_decoded.indexOf("ack") != -1)
              {
                Serial.println("ACK RECEIVED!");
                result = true;
              }else{
                Serial.write("Expected ACK, got: ");
                Serial.println(buffer_ack);
              }
            }else{
              Serial.println("Receiving ack failed");
            }
          }else{
            Serial.println("Sending data failed");
          }
        } else {
          Serial.write("Expected cts, got: ");
          Serial.println(buffer_rts_decoded);
          result = false;
        }
      } else {
        Serial.println("Rts timed out, Expected Cts but got Nothing");
      }
    } else {
      result = false;
    }
  }
  return result;
}

//put radio into receive mode for a certain time, and receive a line into receive_buffer
bool receive_radio(String* receive_buffer) {
  bool result = 0;
  int receive_mode_time = 100;
  //int receive_read_delay = receive_mode_time + 500;
  String cmd = "radio rx ";
  cmd += receive_mode_time;
  String status = send_cmd_blocking(cmd);
  if (status.indexOf("ok") == 0)
  {
    //delay(receive_read_delay); Not sure if "radio tx 1000" blocks or not
    *receive_buffer = loraSerial.readStringUntil('\n');  //ok, invalid_param, busy
    Serial.println("receive_radio buffer: " + *receive_buffer);
    if (*receive_buffer == "") {
      //not sure when this happens, maybe watchdog timeout?
      Serial.println("receive buffer == \"\", probably TIMED OUT?");
    }else if((*receive_buffer).indexOf("radio_err") == 0)
    {
      Serial.println("receive buffer == \"radio_err\", (Error/Timeout)");
    }else if((*receive_buffer).indexOf("radio_rx") == 0)
    {
      *receive_buffer = (*receive_buffer).substring(9); //remove heading "readio_rx" and only keep data
      Serial.write("data only: ");
      Serial.println(*receive_buffer);
    }else{
      Serial.println("ERROR, no such response?"); 
    }
    result = true;
  } else{
    result = false;
  }
  return result;
}

//send text over the radio and wait for completion
//to_send: plaintext message
bool send_radio_blocking(String to_send) {
  bool result = 0;
  String command = "radio tx ";
  command += base16encode(to_send);
  //command = "radio tx 48656c6C6F"; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Serial.write("send_radio_blocking full command: ");
  Serial.println(command);
  loraSerial.println(command);
  String status = loraSerial.readStringUntil('\n');  //ok, invalid_param, busy
  Serial.write("send_radio status: ");
  Serial.println(status);
  if (status.indexOf("ok") == 0)
  {
    do {
      delay(500);
      status = loraSerial.readStringUntil('\n');  //radio_tx_ok, radio_tx_err
      Serial.println("send_radio_blocking: attempt status " + status);
    } while (status.indexOf("radio_tx_ok") != 0);

    result = true;
  } else {
    result = false;
    Serial.println("ERR: radio tx status: " + status);
  }
  return result;
}

//send command to radio module
String send_cmd_blocking(String cmd)
{
  Serial.println("CMD: " + cmd);
  loraSerial.println(cmd);
  String response = loraSerial.readStringUntil('\n');

  Serial.println("RESPONSE: " + response);
  if (response == "") {
    Serial.println("(Timed out)");
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
      Serial.println("Status of command was " + status + " instead of \"ok\"");
      delay(1000);
    } while (true);

  }
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


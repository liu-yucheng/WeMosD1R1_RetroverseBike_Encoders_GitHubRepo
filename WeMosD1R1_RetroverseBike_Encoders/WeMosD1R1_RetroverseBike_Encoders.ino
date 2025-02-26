// If disclosed to the public,
//   Copyright (C) 2023-2025 Yucheng Liu. Under the GNU AGPL 3.0 License.
//   GNU AGPL 3.0 License: https://www.gnu.org/licenses/agpl-3.0.txt .
// Else,
//   Copyright (C) 2023-2025 Yucheng Liu. All rights reserved.
// End if.
//
// WeMos D1 R1.
// Retroverse Bike Encoders.
//
// Dev. 0:        Yucheng Liu
// Dev. 0 emails: yliu428@connect.hkust-gz.edu.cn
//
// Begin - References.
// 
// https://www.arduino.cc/reference/en/language/functions/communication/serial/
// https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html
// https://reference.arduino.cc/reference/en/libraries/wifi/
// https://docs.arduino.cc/library-examples/wifi-library/WiFiUdpSendReceiveString
// https://docs.arduino.cc/learn/built-in-libraries/software-serial
// https://github.com/LennartHennigs/ESPRotary
// 
// End - References.
// ========-========-========-========-========-========-========-========-

#include <any>
#include <cmath>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <ESPRotary.h>
#include <Ticker.h>

// ========-========-========-========-========-========-========-========-

#define DebugEnabled 0

#if DebugEnabled


#define Serial_Print Serial.print
#define Serial_Print_Format Serial.printf
#define Serial_Print_Line Serial.println
#define Serial_Flush Serial.flush

#define LED_Toggle LED_Toggle_


#else


#define Serial_Print Serial_NoOperation
#define Serial_Print_Format Serial_NoOperation
#define Serial_Print_Line Serial_NoOperation
#define Serial_Flush Serial.flush

#define LED_Toggle LED_NoOperation


#endif

// ========-========-========-========-========-========-========-========-


// (!) If build for any Retroverse Bike (bike),
//   Check all configurations with (!) in this region.
// End if.

// (!)
// unsigned int __WiFi_IP_Prefix_Sections_Count__ = 4;
unsigned int const __WiFi_IP_Prefix_Sections_Count__ = 4;

// (!) Check bike subnet prefix 192.168.31.* .
int const __WiFi_IP_Prefix_Sections__[__WiFi_IP_Prefix_Sections_Count__] = {
  192, 168, 31, 0
};

// (!)
// bool const __WiFi_Active_Enabled__ = false;
bool const __WiFi_Active_Enabled__ = false;

// (!) Check bike IP, default: 192.168.31.100 .
IPAddress const __WiFi_Local_IP__(
  __WiFi_IP_Prefix_Sections__[0],
  __WiFi_IP_Prefix_Sections__[1],
  __WiFi_IP_Prefix_Sections__[2],
  100
);

// (!) Check router IP, default: 192.168.31.1 .
IPAddress const __WiFi_Local_Gateway__(
  __WiFi_IP_Prefix_Sections__[0],
  __WiFi_IP_Prefix_Sections__[1],
  __WiFi_IP_Prefix_Sections__[2],
  1
);

// (!) Check router subnet, default: 255.255.255.0 .
IPAddress const __WiFi_Local_Subnet__(255, 255, 255, 0);

// (!) Fill in WiFi SSID "Retroverse-Bike_****"
//   where "****" is a 4 digit pseudo random hex.
char const __WiFi_Config_SSID__[] = "SSID"; 

// (!) Fill in WiFi password "0000****"
//   where "****" is a 4 digit pseudo random hex.
char const __WiFi_Config_Password__[] = "Password@0";

// (!)
// unsigned int const __UDP_Local_Port__ = 50010;
int const __UDP_Local_Port__ = 50010;

// (!) Check PC IP, default: 192.168.31.240 .
IPAddress const __UDP_Remote_IP__(
  __WiFi_IP_Prefix_Sections__[0],
  __WiFi_IP_Prefix_Sections__[1],
  31, // __WiFi_IP_Prefix_Sections__[2],
  240
);

// (!)
// int const __UDP_Remote_Ports_Count__ = 6;
unsigned int const __UDP_Remote_Ports_Count__ = 6;

// (!) Check PC ports, default: 10000, 20000, 30000, 40000, 50000, 60000.
int const __UDP_Remote_Ports__[__UDP_Remote_Ports_Count__] = {
  10000, 20000, 30000, 40000, 50000, 60000
};

// (!) If build for bike v0.3.*-v0.4.*, v0.6.2-alpha-1-*,
//   bool const __Handlebar_Value_Invert_Enabled__ = false;
// Else,
//   bool const __Handlebar_Value_Invert_Enabled__ = true;
// End if.
bool const __Handlebar_Value_Invert_Enabled__ = true;

// (!) If build for bike v0.3.*,
//   float const __Handlebar_Value_Factor__ = 1.5f;
// Else,
//   float const __Handlebar_Value_Factor__ = 1.0f;
// End if.
float const __Handlebar_Value_Factor__ = 1.0f;

// (!)
// bool const __Pedal_Value_Invert_Enabled__ = false;
bool const __Pedal_Value_Invert_Enabled__ = false;

// (!) If build for bike v0.3.0,
//   float const __Pedal_Value_Factor__ = 0.3f;
// Else,
//   float const __Pedal_Value_Factor__ = 1.0f;
// End if.
float const __Pedal_Value_Factor__ = 1.5f;

// (!)
// float const __Pedal_Value_Filter_ContinuousEMA_Alpha_Ideal__ = 0.5f;
float const __Pedal_Value_Filter_ContinuousEMA_Alpha_Ideal__ = 0.5f;


// ========-========-========-========-========-========-========-========-

unsigned int const Serial_BaudRate = 921600;

unsigned int const WatchDogTimer_TimeOut_MS = 5000;

bool LED_On = false;

bool const WiFi_Active_Enabled = __WiFi_Active_Enabled__;
unsigned int const WiFi_Retry_Count_Max = 60;
unsigned int const WiFi_Retry_Delay_MS = 500;
IPAddress const WiFi_Local_IP = __WiFi_Local_IP__;
IPAddress const WiFi_Local_Gateway = __WiFi_Local_Gateway__;
IPAddress const WiFi_Local_Subnet = __WiFi_Local_Subnet__;
char const * WiFi_Config_SSID = __WiFi_Config_SSID__;
char const * WiFi_Config_Password = __WiFi_Config_Password__;

WiFiUDP UDP_Local;
int const UDP_Local_Port = __UDP_Local_Port__;
unsigned long UDP_Packet_Index = 0L;
unsigned int const UDP_Packet_Out_Length_Bytes = 2048;
char UDP_Packet_Out[UDP_Packet_Out_Length_Bytes];
unsigned int const UDP_Remote_Ports_Count = __UDP_Remote_Ports_Count__;
int const * UDP_Remote_Ports = __UDP_Remote_Ports__;
IPAddress const UDP_Remote_IP = __UDP_Remote_IP__;
unsigned int const UDP_Packet_Send_Delay_MS = 1;


unsigned char const Handlebar_Pin_Reset = D5;
unsigned char const Handlebar_Pin_Serial_RX = D8;
unsigned char const Handlebar_Pin_Serial_TX = D9;
float Handlebar_Value_Degrees = 0.0f;
bool const Handlebar_Value_Invert_Enabled = __Handlebar_Value_Invert_Enabled__;
float const Handlebar_Value_Factor = __Handlebar_Value_Factor__;

int Handlebar_Position = 0;
int const Handlebar_Position_Min = 0x0;
int const Handlebar_Position_Max = 0x3ff;

int const Handlebar_Position_Range =
  (Handlebar_Position_Max + 1) - Handlebar_Position_Min
;

SoftwareSerial Handlebar_Serial(
  Handlebar_Pin_Serial_RX,
  Handlebar_Pin_Serial_TX
);

unsigned int const Handlebar_Serial_BaudRate = 9600;

EspSoftwareSerial::Config const Handlebar_Serial_Mode = SWSERIAL_8E1;

unsigned int const Handlebar_Request_BaudRate_Length_Bytes = 8;

unsigned char const Handlebar_Request_BaudRate[
  Handlebar_Request_BaudRate_Length_Bytes
] = {
  0x01, 0x06, 0x00, 0x45, 0x00, 0x02, 0x19, 0xde
};

unsigned int const Handlebar_Request_BaudRate_Delay_MS = 150;
unsigned int const Handlebar_Request_Delay_MS = 20;
bool const Handlebar_Request_BlockingEnabled = false;
unsigned int const Handlebar_Request_Reset_Length_Bytes = 13;

unsigned char const Handlebar_Request_Reset[
  Handlebar_Request_Reset_Length_Bytes
] = {
  0x01, 0x10, 0x00, 0x4a, 0x00, 0x02, 0x04, 0x00,
  0x00, 0x00, 0x00, 0x77, 0xe0
};

unsigned int const Handlebar_Request_Reset_Delay_MS = 150;
unsigned int const Handlebar_Request_Read_Length_Bytes = 8;

unsigned char const Handlebar_Request_Read[
  Handlebar_Request_BaudRate_Length_Bytes
] = {
  0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b
};

unsigned int const Handlebar_Reply_Length_Bytes = 16;
unsigned char Handlebar_Reply[Handlebar_Reply_Length_Bytes];
unsigned int const Handlebar_Reply_Read_Length_Bytes = 9;
bool const Handlebar_Init_Reset = false;
unsigned int const Handlebar_Read_Interval_Ideal_MS = 41;


ESPRotary Pedal_Encoder;
unsigned char const Pedal_Pin_PulseA = D6;
unsigned char const Pedal_Pin_PulseB = D7;
unsigned char const Pedal_Speed_StepsPerClick = 4;
int const Pedal_Speed_Increment = 1;
float Pedal_Speed_Current_RPS = 0.0f;
float Pedal_Speed_Previous_RPS = 0.0f;
int const Pedal_Speedup_Interval_MS = 75;
int const Pedal_Speedup_Increment = 5;
bool Pedal_Speedup_Enabled = false;
int const Pedal_Reset_Value = 0;
unsigned long Pedal_Time_Read_Current_MS = 0L;
unsigned long Pedal_Time_Read_Previous_MS = 0L;
int const Pedal_Time_MS_Per_Second = 1000;
long Pedal_Position_Current = 0L;
long Pedal_Position_Previous = 0L;
long const Pedal_Position_HalfInt16Min = INT16_MIN / 2;
long const Pedal_Position_HalfInt16Max = INT16_MAX / 2;
int const Pedal_Position_Pulses_Per_Revolution = 100;
float Pedal_Value_RPS = 0.0f;
float Pedal_Value_Previous_RPS = 0.0f;

float const Pedal_Value_Filter_ContinuousEMA_Alpha_Ideal =
  __Pedal_Value_Filter_ContinuousEMA_Alpha_Ideal__
;

bool const Pedal_Value_Invert_Enabled = __Pedal_Value_Invert_Enabled__;
float const Pedal_Value_Factor = __Pedal_Value_Factor__;
unsigned int const Pedal_Loop_Delay_Time_Interval_Loop_MS = 5;
unsigned long Pedal_Loop_Delay_Time_Current_Delay_MS = 0L;
unsigned long Pedal_Loop_Delay_Time_Current_Loop_MS = 0L;
unsigned int const Pedal_Read_Interval_Ideal_MS = 43;

float const Pedal_Read_Interval_Ideal_Seconds =
  (float) Pedal_Read_Interval_Ideal_MS
  / (float) Pedal_Time_MS_Per_Second
;


Ticker Ticker_Serial_Flush;
Ticker Ticker_WiFi_Status_Print;
Ticker Ticker_UDP_Packet_Send;
Ticker Ticker_Handlebar_Read;
Ticker Ticker_Pedal_Read;
Ticker Ticker_Pedal_Loop;
unsigned int const Ticker_Interval_Serial_Flush_MS = 100;
unsigned int const Ticker_Interval_WiFi_Status_Print_MS = 1000;
unsigned int const Ticker_Interval_UDP_Packet_Send_MS = 47;

unsigned int const Ticker_Interval_Handlebar_Read_MS =
  Handlebar_Read_Interval_Ideal_MS
;

unsigned int const Ticker_Interval_Pedal_Read_MS
  = Pedal_Read_Interval_Ideal_MS
;

unsigned int const Ticker_Interval_Pedal_Loop_MS = 5;


// ========-========-========-========-========-========-========-========-

void setup();
void loop();

void Setup_Serial();
void Setup_WatchDogTimer();
void Setup_LED();
void Setup_WiFi_Active();
void Setup_WiFi_Passive();
void Setup_UDP();
void Setup_Handlebar();
void Setup_Pedal();
void Setup_Interrupts();
void Setup_Tickers();

void IRAM_ATTR Loop_Pedal();
void Loop_WatchDogTimer();

void Serial_NoOperation(std::any _Ignored ...);
void Serial_Flush_();

void WatchDogTimer_Enabled_Set(bool Enabled);
void WatchDogTimer_Feed();

void LED_Toggle_();
void LED_NoOperation();

void WiFi_Status_Print();

void UDP_Packet_Send();
void UDP_Packet_Prepare();
void UDP_Packet_Send_();

void Handlebar_Setup_ResetPin();
void Handlebar_Setup_Serial();
void Handlebar_Setup_BaudRate();
void Handlebar_Reset();
void IRAM_ATTR Handlebar_Reset_RealTime();
void Handlebar_Read();


void Pedal_Loop();
void Pedal_Loop_Delay(unsigned long Time_ToDelay_MS);
void Pedal_Read();

void Pedal_Read_Filter_ContinuousEMA(
  float Alpha_Ideal,
  float Value_Raw_Current,
  float Value_Filtered_Previous,
  float Time_Interval_Actual,
  float Time_Interval_Ideal,
  float* Value_Filtered_Current
);


// ========-========-========-========-========-========-========-========-

void setup() {
  Setup_Serial();
  Serial_Print_Format("begin setup\n");
  Setup_WatchDogTimer();
  Setup_LED();
  
  if (WiFi_Active_Enabled) {
    Setup_WiFi_Active();
  } else {
    Setup_WiFi_Passive();
  }

  Setup_UDP();
  Setup_Handlebar();
  Setup_Pedal();
  Setup_Interrupts();
  Setup_Tickers();
  Serial_Print_Format("end setup\n");
  Serial_Flush();
  LED_Toggle();
}

void loop() {
  Serial_Print_Format("begin loop\n");
  Loop_Pedal();
  Loop_WatchDogTimer();
  Serial_Print_Format("end loop\n");
  Serial_Flush();
  LED_Toggle();
}

void Setup_Serial() {
  Serial.begin(Serial_BaudRate);
  Serial_Print_Format("\nbegin Setup_Serial\n");
  Serial_Flush();
  Serial_Print_Format("\end Setup_Serial\n");
}

void Setup_WatchDogTimer() {
  Serial_Print_Format("begin Setup_WatchDogTimer\n");
  // ESP.wdtEnable(WatchDogTimer_TimeOut_MS);
  WatchDogTimer_Enabled_Set(false);
  Serial_Print_Format("end Setup_WatchDogTimer\n");
}

void Setup_LED() {
  Serial_Print_Format("begin Setup_LED\n");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial_Print_Format("end Setup_LED\n");
}

void Setup_WiFi_Active() {
  Serial_Print_Format("begin Setup_WiFi_Active\n");
  unsigned int RetryCount = 0;
  bool Successful = false;

  while (!Successful && RetryCount < WiFi_Retry_Count_Max) {
    Serial_Print_Format("Will configure active WiFi\n");
    
    Successful = WiFi.softAPConfig(
      WiFi_Local_IP,
      WiFi_Local_Gateway,
      WiFi_Local_Subnet
    );
    
    Serial_Print_Format(
      Successful ?
        "Configuration successful\n" :
        "Configuration failed\n"
    );

    LED_Toggle();
    RetryCount += 1;
  }

  RetryCount = 0;
  Successful = false;

  while (!Successful && RetryCount < WiFi_Retry_Count_Max) {
    Serial_Print_Format(
      "Will setup active WiFi; SSID: %s\n",
      WiFi_Config_SSID
    );
    
    // WiFi.softAP(ssid);
    // WiFi.softAP(ssid, password, channel, hidden, max_connection);
    Successful = WiFi.softAP(WiFi_Config_SSID, WiFi_Config_Password);
    
    Serial_Print_Format(Successful ? "Setup successful\n" : "Setup failed\n");
    delay(WiFi_Retry_Delay_MS);
    LED_Toggle();
    RetryCount += 1;
  }

  Serial_Print_Format(
    "Active WiFi IP address: %s\n",
    WiFi.softAPIP().toString().c_str()
  );
  
  Serial_Print_Format("end Setup_WiFi_Active\n");
  LED_Toggle();
}

void Setup_WiFi_Passive() {
  Serial_Print_Format("begin Setup_WiFi_Passive\n");

  Serial_Print_Format(
    "Will setup passive WiFi; SSID: %s\n",
    WiFi_Config_SSID
  );

  WiFi.config(WiFi_Local_IP, WiFi_Local_Gateway, WiFi_Local_Subnet);
  WiFi.begin(WiFi_Config_SSID, WiFi_Config_Password);
  Serial_Print_Format("Connecting ");
  unsigned int RetryCount = 0;

  while (WiFi.status() != WL_CONNECTED && RetryCount < WiFi_Retry_Count_Max) {
    Serial_Print_Format(".");
    delay(WiFi_Retry_Delay_MS);
    LED_Toggle();
    RetryCount += 1;
  }

  Serial_Print_Format("end Setup_WiFi_Passive\n");
  LED_Toggle();
}

void Setup_UDP() {
  Serial_Print_Format("begin Setup_UDP\n");
  UDP_Local.begin(UDP_Local_Port);

  Serial_Print_Format(
    "IP: %s; Port: %d\n",
    WiFi.localIP().toString().c_str(),
    UDP_Local_Port
  );

  Serial_Print_Format("end Setup_UDP\n");
  LED_Toggle();
}

void Setup_Handlebar() {
  Serial_Print_Format("begin setup Setup_Handlebar\n");
  Handlebar_Setup_ResetPin();
  Handlebar_Setup_Serial();
  Handlebar_Setup_BaudRate();
  Handlebar_Reset();
  Serial_Print_Format("end Setup_Handlebar\n");
  LED_Toggle();
}

void Setup_Pedal() {
  Serial_Print_Format("begin Setup_Pedal\n");
  
  Pedal_Encoder.begin(
    Pedal_Pin_PulseA,
    Pedal_Pin_PulseB,
    Pedal_Speed_StepsPerClick
  );
  
  Pedal_Encoder.setIncrement(Pedal_Speed_Increment);
  Pedal_Encoder.setSpeedupInterval(Pedal_Speedup_Interval_MS);
  Pedal_Encoder.setSpeedupIncrement(Pedal_Speedup_Increment);
  Pedal_Encoder.enableSpeedup(Pedal_Speedup_Enabled);
  pinMode(Pedal_Pin_PulseA, INPUT_PULLUP);
  pinMode(Pedal_Pin_PulseB, INPUT_PULLUP);
  Pedal_Time_Read_Current_MS = millis();
  Pedal_Position_Current = Pedal_Encoder.getPosition();
  // Pedal_Encoder.resetPosition(Pedal_Reset_Value);
  Serial_Print_Format("Pedal current position: %ld\n", Pedal_Position_Current);
  Pedal_Speed_Current_RPS = 0.0f;
  
  Serial_Print_Format(
    "Pedal current read time (ms): %ld\n",
    Pedal_Time_Read_Current_MS
  );
  
  Serial_Print_Format(
    "Pedal current speed (rps): %.6f\n",
    Pedal_Speed_Current_RPS
  );
  
  Serial_Print_Format("end Setup_Pedal\n");
  LED_Toggle();
}

void Setup_Interrupts() {
  Serial_Print_Format("begin Setup_Interrupts\n");
  attachInterrupt(digitalPinToInterrupt(Pedal_Pin_PulseA), Pedal_Loop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pedal_Pin_PulseB), Pedal_Loop, CHANGE);
  
  attachInterrupt(
    digitalPinToInterrupt(Handlebar_Pin_Reset),
    Handlebar_Reset_RealTime, CHANGE
  );
  
  Serial_Print_Format("end Setup_Interrupts\n");
  LED_Toggle();
}

void Setup_Tickers() {
  Serial_Print_Format("begin Setup_Tickers\n");
  
  Ticker_Handlebar_Read.attach_ms(
    Ticker_Interval_Handlebar_Read_MS,
    Handlebar_Read
  );
  
  Ticker_Pedal_Read.attach_ms(
    Ticker_Interval_Pedal_Read_MS,
    Pedal_Read
  );

  Ticker_UDP_Packet_Send.attach_ms(
    Ticker_Interval_UDP_Packet_Send_MS,
    UDP_Packet_Send
  );

  Ticker_Pedal_Loop.attach_ms(
    Ticker_Interval_Pedal_Loop_MS,
    Pedal_Loop
  );

  if (WiFi_Active_Enabled) {
      Ticker_WiFi_Status_Print.attach_ms(
      Ticker_Interval_WiFi_Status_Print_MS,
      WiFi_Status_Print
    );
  }

  Ticker_Serial_Flush.attach_ms(
    Ticker_Interval_Serial_Flush_MS,
    Serial_Flush_
  );

  Serial_Print_Format("end Setup_Tickers\n");
  LED_Toggle();
}

void IRAM_ATTR Loop_Pedal() {
  Serial_Print_Format("begin Loop_Pedal\n");
  Pedal_Loop();
  Serial_Print_Format("end Loop_Pedal\n");
  LED_Toggle();
}

void Loop_WatchDogTimer() {
  Serial_Print_Format("begin Loop_WatchDogTimer\n");
  WatchDogTimer_Feed();
  Serial_Print_Format("end Loop_WatchDogTimer\n");
  LED_Toggle();
}

void Serial_NoOperation(std::any _Ignored ...) {
  // Do nothing.
}

void Serial_Flush_() {
  Serial_Flush();
}

void WatchDogTimer_Enabled_Set(bool Enabled) {
  Serial_Print_Format("begin WatchDogTimer_Enabled_Set\n");

  if (Enabled) {
    *((volatile uint32_t *) 0x60000900) |= 1;
  } else {
    *((volatile uint32_t *) 0x60000900) &= ~(1);
  }

  Serial_Print_Format("end WatchDogTimer_Enabled_Set\n");
  LED_Toggle();
}

void WatchDogTimer_Feed() {
  // Serial_Print_Format("begin WatchDogTimer_Feed\n");
  ESP.wdtFeed();
  // Serial_Print_Format("end WatchDogTimer_Feed\n");
  LED_Toggle();
}

void LED_Toggle_() {
  // Serial_Print_Format("begin LED_Toggle_\n");
  
  if (!LED_On) {
    digitalWrite(LED_BUILTIN, LOW);
    LED_On = true;
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    LED_On = false;
  }
  
  // Serial_Print_Format("end LED_Toggle_\n");
}

void LED_NoOperation() {
  // Do nothing.
}

void WiFi_Status_Print() {
  Serial_Print_Format("begin WiFi_Status_Print\n");
  Serial_Print_Format("WiFi IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial_Print_Format("end WiFi_Status_Print\n");
  LED_Toggle();
}

void UDP_Packet_Send() {
  Serial_Print_Format("begin UDP_Packet_Send\n");
  UDP_Packet_Prepare();
  UDP_Packet_Send_();
  UDP_Packet_Index += 1;
  Serial_Print_Format("end UDP_Packet_Send\n");
  LED_Toggle();
}

void UDP_Packet_Prepare() {
  Serial_Print_Format("begin UDP_Packet_Prepare\n");
  unsigned long Time_MS = millis();
  Serial_Print_Format("Time (ms): %ld\n", Time_MS);
  
  char const Format[] = 
    "packet_index %d "
    "handlebar_value_degrees %.6f "
    "pedal_value_rps %.6f "
    "handlebar_position %d "
    "pedal_position %ld\n"
  ;

  sprintf(
    UDP_Packet_Out,
    Format,
    UDP_Packet_Index,
    Handlebar_Value_Degrees,
    Pedal_Value_RPS,
    Handlebar_Position,
    Pedal_Position_Current
  );

  Serial_Print_Format("end UDP_Packet_Prepare\n");
  LED_Toggle();
}

void UDP_Packet_Send_() {
  Serial_Print_Format("begin UDP_Packet_Send_\n");
  unsigned int Sent;

  for (unsigned int Index = 0; Index < UDP_Remote_Ports_Count; Index += 1) {
    UDP_Local.beginPacket(UDP_Remote_IP, UDP_Remote_Ports[Index]);
    
    UDP_Local.write(
      (const unsigned char*)UDP_Packet_Out,
      strlen(UDP_Packet_Out)
    );

    Sent = UDP_Local.endPacket();

    Serial_Print_Format(
      "Sent: %d "
      "Receiver IP: %s "
      "Receiver port: %d\n",
      
      Sent,
      UDP_Remote_IP.toString().c_str(),
      UDP_Remote_Ports[Index]
    );

    delay(UDP_Packet_Send_Delay_MS);
  }

  Serial_Print_Format(
    "\n---- begin UDP_Packet_Out ----\n"
    "%s\n"
    "---- end UDP_Packet_Out ----\n\n",
    
    UDP_Packet_Out
  );

  Serial_Print_Format("end UDP_Packet_Send_\n");
  LED_Toggle();
}

void Handlebar_Setup_ResetPin() {
  Serial_Print_Format("begin Handlebar_Setup_ResetPin\n");
  pinMode(Handlebar_Pin_Reset, INPUT_PULLUP);
  Serial_Print_Format("end Handlebar_Setup_ResetPin\n");
  LED_Toggle();
}

void Handlebar_Setup_Serial() {
  Serial_Print_Format("begin Handlebar_Setup_Serial\n");
  pinMode(Handlebar_Pin_Serial_RX, INPUT);
  pinMode(Handlebar_Pin_Serial_TX, OUTPUT);
  Handlebar_Serial.begin(Handlebar_Serial_BaudRate, Handlebar_Serial_Mode);
  Serial_Print_Format("end Handlebar_Setup_Serial\n");
  LED_Toggle();
}

void Handlebar_Setup_BaudRate() {
  Serial_Print_Format("begin Handlebar_Setup_BaudRate\n");
  unsigned int Count;
  bool RequestAgain_Needed = false;

  do {
    Count = Handlebar_Serial.write(
      Handlebar_Request_BaudRate,
      Handlebar_Request_BaudRate_Length_Bytes
    );

    Serial_Print_Format(
      "Handlebar baud rate: %d; "
      "Request length (bytes): %d\n",
      
      Handlebar_Serial_BaudRate,
      Count
    );

    delay(Handlebar_Request_BaudRate_Delay_MS);

    if (Handlebar_Request_BlockingEnabled) {
      RequestAgain_Needed = !Handlebar_Serial.available();
    }
    else {
      RequestAgain_Needed = false;
    }
  } while (RequestAgain_Needed);

  Handlebar_Serial.begin(Handlebar_Serial_BaudRate, Handlebar_Serial_Mode);
  unsigned int Index = 0;

  while (
    Handlebar_Serial.available()
    && Index < Handlebar_Reply_Length_Bytes
  ) {
    unsigned char Byte = Handlebar_Serial.read();
    Handlebar_Reply[Index] = Byte;
    Index += 1;
  }

  Count = Index;
  Index = 0;
  Serial_Print_Format("Handlebar reply:");

  while (Index < Count) {
    Serial_Print_Format(" %02x", Handlebar_Reply[Index]);
    Index += 1;
  }

  Serial_Print_Format("\n");
  Serial_Print_Format("end Handlebar_Setup_BaudRate\n");
  LED_Toggle();
}

void Handlebar_Reset() {
  Serial_Print_Format("begin Handlebar_Reset\n");

  if (Handlebar_Init_Reset) {
    Handlebar_Reset_RealTime();
  }
  
  Serial_Print_Format("end Handlebar_Reset\n");
  LED_Toggle();
}

void IRAM_ATTR Handlebar_Reset_RealTime() {
  Serial_Print_Format("begin Handlebar_Reset_RealTime\n");
  delay(Handlebar_Request_Delay_MS);
  unsigned int Count;
  bool RequestAgain_Needed = false;

  do {
    Count = Handlebar_Serial.write(
      Handlebar_Request_Reset,
      Handlebar_Request_Reset_Length_Bytes
    );

    Serial_Print_Format("Request length (bytes): %d\n", Count);
    delay(Handlebar_Request_Reset_Delay_MS);

    if (Handlebar_Request_BlockingEnabled) {
      RequestAgain_Needed = !Handlebar_Serial.available();
    }
    else {
      RequestAgain_Needed = false;
    }
  } while (RequestAgain_Needed);

  delay(Handlebar_Request_Delay_MS);
  unsigned int Index = 0;

  while (
    Handlebar_Serial.available()
    && Index < Handlebar_Reply_Length_Bytes
  ) {
    unsigned char Byte = Handlebar_Serial.read();
    Handlebar_Reply[Index] = Byte;
    Index += 1;
  }

  Count = Index;
  Index = 0;
  Serial_Print_Format("Handlebar reply:");

  while (Index < Count) {
    Serial_Print_Format(" %02x", Handlebar_Reply[Index]);
    Index += 1;
  }

  Serial_Print_Format("\n");
  Serial_Print_Format("end Handlebar_Reset_RealTime\n");
  LED_Toggle();
}

void Handlebar_Read() {
  Serial_Print_Format("begin Handlebar_Read\n");
  unsigned long Time_MS = millis();
  Serial_Print_Format("Time (ms): %ld\n", Time_MS);
  unsigned int Count;
  bool RequestAgain_Needed = false;

  do {
    Count = Handlebar_Serial.write(
      Handlebar_Request_Read,
      Handlebar_Request_Read_Length_Bytes
    );

    Serial_Print_Format("Request length (bytes): %d\n", Count);
    Pedal_Loop_Delay(Handlebar_Request_Delay_MS);
    
    if (Handlebar_Request_BlockingEnabled) {
      RequestAgain_Needed = !Handlebar_Serial.available();
    }
    else {
      RequestAgain_Needed = false;
    }
  } while (RequestAgain_Needed);

  unsigned int Index = 0;

  while (
    Handlebar_Serial.available()
    && Index < Handlebar_Reply_Length_Bytes
  ) {
    unsigned char Byte = Handlebar_Serial.read();
    Handlebar_Reply[Index] = Byte;
    Index += 1;
  }

  Count = Index;
  Index = 0;
  Serial_Print_Format("Handlebar reply:");

  while (Index < Count) {
    Serial_Print_Format(" %02x", Handlebar_Reply[Index]);
    Index += 1;
  }

  Serial_Print_Format("\n");

  if (Count == Handlebar_Reply_Read_Length_Bytes) {
    unsigned int Byte1 = Handlebar_Reply[5];
    unsigned int Byte2 = Handlebar_Reply[6];
    unsigned int Bytes = (Byte1 << 8) | Byte2;
    Handlebar_Position = (int)Bytes;
    Serial_Print_Format("Handlebar position: %#x\n", Handlebar_Position);
    
    float Rotation_Degrees =
      (float)Handlebar_Position / (float)Handlebar_Position_Range * 360.0f
    ;

    if (Rotation_Degrees > 180.0f) {
      Rotation_Degrees -= 360.0f;
    }

    Serial_Print_Format("Rotation (degrees): %.6f\n", Rotation_Degrees);
    Handlebar_Value_Degrees = Rotation_Degrees;

    if (Handlebar_Value_Invert_Enabled) {
      Handlebar_Value_Degrees = -Handlebar_Value_Degrees;
      
      Serial_Print_Format(
        "Handlebar value (inverted, degrees): %.6f\n",
        Handlebar_Value_Degrees
      );
    }

    Handlebar_Value_Degrees *= Handlebar_Value_Factor;

    while (Handlebar_Value_Degrees < -180.0f) {
      Handlebar_Value_Degrees += 360.0f;
    }

    while (Handlebar_Value_Degrees > 180.0f) {
      Handlebar_Value_Degrees -= 360.0f;
    }

    Serial_Print_Format(
      "Handlebar value (factored, degrees): %.6f\n",
      Handlebar_Value_Degrees
    );
  } else {
    Serial_Print_Format("Handlebar reply format unknown\n");
  }

  Serial_Print_Format("end Handlebar_Read\n");
  LED_Toggle();
}

void IRAM_ATTR Pedal_Loop() {
  Serial_Print_Format("begin Pedal_Loop\n");
  Pedal_Encoder.loop();
  unsigned long Time_MS = millis();
  Serial_Print_Format("Time (ms): %ld\n", Time_MS);
  int Position = Pedal_Encoder.getPosition();

  if (Position == INT16_MAX) {
    Pedal_Encoder.resetPosition(INT16_MIN);
  } 
  
  if (Position == INT16_MIN) {
    Pedal_Encoder.resetPosition(INT16_MAX);
  }

  Serial_Print_Format("end Pedal_Loop\n");
  LED_Toggle();
}

void Pedal_Loop_Delay(unsigned long Time_ToDelay_MS) {
  delay(Time_ToDelay_MS);
  LED_Toggle();
  return;

  Serial_Print_Format("begin Pedal_Loop_Delay\n");
  unsigned long Time_MS = millis();
  Pedal_Loop_Delay_Time_Current_Delay_MS = Time_MS;
  unsigned long Time_Delayed_MS = 0L;
  unsigned long Time_Looped_MS = 0L;

  do {
    Time_MS = millis();
    Time_Delayed_MS = Time_MS - Pedal_Loop_Delay_Time_Current_Delay_MS;
    Time_Looped_MS = Time_MS - Pedal_Loop_Delay_Time_Current_Loop_MS;

    if (Time_Looped_MS >= Pedal_Loop_Delay_Time_Interval_Loop_MS) {
      Pedal_Loop();
      Pedal_Loop_Delay_Time_Current_Loop_MS = Time_MS;
    }
  } while (Time_Delayed_MS <= Time_ToDelay_MS);

  Serial_Print_Format("end Pedal_Loop_Delay\n");
  LED_Toggle();
}

void Pedal_Read() {
  Serial_Print_Format("begin Pedal_Read\n");
  unsigned long Time_MS = millis();
  Serial_Print_Format("Time (ms): %ld\n", Time_MS);
  Pedal_Time_Read_Current_MS = millis();
  Pedal_Position_Current = Pedal_Encoder.getPosition();
  Serial_Print_Format("Pedal current position: %ld\n", Pedal_Position_Current);
  long Position_Diff;

  if (
    Pedal_Position_Current < Pedal_Position_HalfInt16Min
    && Pedal_Position_Previous > Pedal_Position_HalfInt16Max
  ) {
    // Handle pedal position positive overflow.
    Position_Diff =
      (Pedal_Position_Current - INT16_MIN)
      + (INT16_MAX - Pedal_Position_Previous)
    ;
  } else if (
    Pedal_Position_Current > Pedal_Position_HalfInt16Max
    && Pedal_Position_Previous < Pedal_Position_HalfInt16Min
  ) {
    // Handle pedal position negative overflow.
    Position_Diff =
      - (INT16_MAX - Pedal_Position_Current)
      - (Pedal_Position_Previous - INT16_MIN);
  } else {
    Position_Diff = Pedal_Position_Current - Pedal_Position_Previous;
  }

  Serial_Print_Format("Pedal position difference: %ld\n", Position_Diff);
  
  long Time_Interval_MS =
    Pedal_Time_Read_Current_MS
    - Pedal_Time_Read_Previous_MS
  ;

  if (!Time_Interval_MS) {
    Time_Interval_MS = 100L;
  }

  Serial_Print_Format("Time interval (ms): %ld\n", Time_Interval_MS);

  float Time_Interval_Seconds =
    (float) Time_Interval_MS
    / (float) Pedal_Time_MS_Per_Second
  ;

  float Revolutions =
    (float) Position_Diff
    / (float) Pedal_Position_Pulses_Per_Revolution
  ;
  
  Pedal_Speed_Current_RPS = Revolutions / Time_Interval_Seconds;
  Pedal_Value_Previous_RPS = Pedal_Value_RPS;
  float Alpha_Ideal = Pedal_Value_Filter_ContinuousEMA_Alpha_Ideal;
  float Value_Raw_Current = Pedal_Speed_Current_RPS;
  float Value_Filtered_Previous = Pedal_Value_Previous_RPS;
  float Time_Interval_Actual = Time_Interval_Seconds;
  float Time_Interval_Ideal = Pedal_Read_Interval_Ideal_Seconds;
  float Value_Filtered_Current = 0.0f;

  Pedal_Read_Filter_ContinuousEMA(
    Alpha_Ideal,
    Value_Raw_Current,
    Value_Filtered_Previous,
    Time_Interval_Actual,
    Time_Interval_Ideal,
    &Value_Filtered_Current
  );
  
  Pedal_Value_RPS = Value_Filtered_Current;

  if (!Pedal_Value_Invert_Enabled) {
    Pedal_Value_RPS = -Pedal_Value_RPS;
  } else {
    Serial_Print_Format(
      "Pedal value (inverted, rps): %.6f\n",
      Pedal_Value_RPS
    );
  }

  Pedal_Value_RPS *= Pedal_Value_Factor;
  
  Serial_Print_Format(
    "Pedal value (factored, rps): %.6f\n",
    Pedal_Value_RPS
  );

  Serial_Print_Format("Pedal value (rps): %.6f\n", Pedal_Value_RPS);
  Pedal_Time_Read_Previous_MS = Pedal_Time_Read_Current_MS;
  Pedal_Position_Previous = Pedal_Position_Current;
  Pedal_Speed_Previous_RPS = Pedal_Speed_Current_RPS;
  Serial_Print_Format("end Pedal_Read\n");
  LED_Toggle();
}

void Pedal_Read_Filter_ContinuousEMA(
  float Alpha_Ideal,
  float Value_Raw_Current,
  float Value_Filtered_Previous,
  float Time_Interval_Actual,
  float Time_Interval_Ideal,
  float* Value_Filtered_Current
) {
  Serial_Print_Format("begin Pedal_Read_Filter_ContinuousEMA\n");

  float Exponent_Rectification = Time_Interval_Actual / Time_Interval_Ideal;
  float _1MinusAlpha_Actual = pow(1 - Alpha_Ideal, Exponent_Rectification);
  float Alpha_Actual = 1 - _1MinusAlpha_Actual;
  
  *Value_Filtered_Current =
    Alpha_Actual * Value_Raw_Current
    + _1MinusAlpha_Actual * Value_Filtered_Previous
  ;

  Serial_Print_Format(
    "Alpha_Ideal: %.6f \n"
      "Time_Interval_Actual: %.6f \n"
      "Time_Interval_Ideal: %.6f \n"
      "Exponent_Rectification: %.6f \n"
      "Alpha_Actual: %.6f \n"
      "_1MinusAlpha_Actual: %.6f \n"
      "*Value_Filtered_Current: %.6f \n",
    
    Alpha_Ideal,
    Time_Interval_Actual,
    Time_Interval_Ideal,
    Exponent_Rectification,
    Alpha_Actual,
    _1MinusAlpha_Actual,
    *Value_Filtered_Current
  );

  Serial_Print_Format("end Pedal_Read_Filter_ContinuousEMA\n");
  LED_Toggle();
}

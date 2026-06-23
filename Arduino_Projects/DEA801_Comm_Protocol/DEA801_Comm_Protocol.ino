/* 
UART Protocol 
Author: Eduardo H. Couto
Date: June/2026
 */

#define DRAIN_TYPE_BIT_FIELD 					  (0x0U)
#define CIRC_TYPE_BIT_FIELD 					  (0x2U)
#define DIVERTER_TYPE_BIT_FIELD 				(0x5U)

#define DIVERTER_3WAY_BITMASK   				(0x0U)

#define CIRC_W20007335_BITMASK   				(0x0U)
#define CIRC_W11715301_BITMASK   				(0x1U)
#define CIRC_W20016965_BITMASK 	  			(0x2U)
#define CIRC_W11652801_BITMASK 	  			(0x3U)

#define DRAIN_W11402566_BITMASK					(0x0U)
#define DRAIN_W11186148_BITMASK					(0x1U)
#define DRAIN_W11377410_BITMASK					(0x2U)

 const uint8_t crcTable[] = {
 0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
 0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
 0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
 0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
 0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
 0xfa, 0xfd, 0xf4, 0xf3
};


/* ##################################################
#################### Defines ######################
#####################################################*/
#define TX_SIZE 12
#define RX_SIZE 23
#define PREAMBLE  0x55U
#define SYSTICK_MS    100  //ms
#define SET_BIT(x)    (1 << x)
#define TX_LENGHT     (TX_SIZE-1)*sizeof(uint8_t)
#define RX_LENGHT     (RX_SIZE-1)*sizeof(uint8_t)

/* ##################################################
#################### Variables ######################
#####################################################*/
volatile bool led_status = false;
uint16_t wash_speed;
uint16_t drain_speed;
uint8_t selectComponent;
uint16_t Time_Cnt;
uint16_t Time_Sec;
uint16_t Time_Min;

//Comm Vars
volatile bool sendFlag = false;
uint8_t txBuffer[TX_SIZE];
uint8_t rxBuffer[RX_SIZE];
uint8_t indexRx;
uint8_t rsvByte1;
bool frameReady;


bool Enable_TX;

uint8_t crc;

/* ##################################################
################### Prototypes ######################
#####################################################*/
static uint16_t check_prescaler(void);
static uint8_t appCalcCRC(const uint8_t *data, uint8_t length);
static void buildTXPackage(void);
static void timeProcessing(void);
static void RXFrameBuilder(void);
static bool validateFrame(void);


/* ##################################################
################### INIT ######################
#####################################################*/
void setup() 
{
  uint16_t psc;

  Enable_TX = true;
  indexRx = 0;
  frameReady = false;

  //Data to be transmitted...
  wash_speed = 0;
  drain_speed = 0;
  selectComponent &= 0x00;
	selectComponent |= DRAIN_W11377410_BITMASK << DRAIN_TYPE_BIT_FIELD;
	selectComponent |= CIRC_W11652801_BITMASK  << CIRC_TYPE_BIT_FIELD;
	selectComponent |= DIVERTER_3WAY_BITMASK   << DIVERTER_TYPE_BIT_FIELD;

  //Time Processing Init
  Time_Cnt = 0;
  Time_Min = 0;
  Time_Sec = 0;
  crc = 0x00;

  //Peripheral Config.
  cli(); // disable interruptions
  Serial.begin(9600);
  
  // Config Timer1 (CTC mode)
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= SET_BIT(WGM12); // CTC mode

  // Prescaler 1024
  TCCR1B |= SET_BIT(CS12) | SET_BIT(CS10);

  psc = check_prescaler();

  //Sanity for prescaler before computing the OCR1A
  if(!psc)
  {
    TCCR1B &= 0x00;
    TCCR1B |= SET_BIT(CS12) | SET_BIT(CS10);
    psc = 1024;
  }

  OCR1A = (uint16_t)(((F_CPU / psc) * SYSTICK_MS) / (1000));
  
  // Enable interruption
  TIMSK1 |= SET_BIT(OCIE1A);

  sei(); // Enable interruptions

  // Enable Built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
}




/* ##################################################
################### LOOP ######################
#####################################################*/
void loop() 
{
  // RX Receiving Monitor
  RXFrameBuilder();

  // Process RX Data Frame
  if (frameReady)
  {
    frameReady = false;
        
   uint32_t fault_mcu =
    ((uint32_t)rxBuffer[4] << 24) |
    ((uint32_t)rxBuffer[3] << 16) |
    ((uint32_t)rxBuffer[2] << 8)  |
    ((uint32_t)rxBuffer[1]);
    
    if(fault_mcu & 0x01000000U)
    {
      rsvByte1 =  1U;
    }
    else
    {
      rsvByte1 = rxBuffer[21]; //idxAcu
    }

    Enable_TX = (bool)rxBuffer[5]; //Controlling TX from external uC.
  }


  // TX Sending Monitor (every systick interruption)
  if (sendFlag) 
  {
    sendFlag = false;

    // build TX Data bytes
    buildTXPackage();

    if(Enable_TX == true)
    {
      // Send it!
      Serial.write(txBuffer, TX_SIZE);
    }
  }

  // Time Processing
  timeProcessing();

  // Update LED Outputs
  if(led_status == true)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);  // change state of the LED by setting the pin to the HIGH voltage level
  }
}



/* ##################################################
  ################ Aux. Functions ###################
#####################################################*/

/* ##################################################
     ################### ISR ######################
#####################################################*/
ISR(TIMER1_COMPA_vect) 
{
  //Activate Communication.
  sendFlag = true;

  //Manage LED Status... 
  led_status = !led_status;

  //Increase Time
  Time_Cnt++;
}



/* ##################################################
  ############## TX Package Builder #################
#####################################################*/
void buildTXPackage() 
{
    //Updating TX Buffer...
    txBuffer[0]  = (uint8_t)(PREAMBLE);            // Preamble (always 0x55)
    txBuffer[1]  = (uint8_t)(0x00U);               // Command Diverter
    txBuffer[2]  = (uint8_t)(0x00U);               // Pilot Valves
    txBuffer[3]  = (uint8_t)(wash_speed & 0xFF);   // Wash_Speed[0]
    txBuffer[4]  = (uint8_t)(wash_speed >> 8);     // Wash_Speed[1]
    txBuffer[5]  = (uint8_t)(drain_speed & 0xFF);  // Drain_Speed[0]
    txBuffer[6]  = (uint8_t)(drain_speed >> 8);    // Drain_Speed[1]
    txBuffer[7]  = (uint8_t)(selectComponent);     // Select Component
    txBuffer[8]  = (uint8_t)(0x00U);               // Heater Cfg
    txBuffer[9]  = (uint8_t)(rsvByte1);            // rsvbyte1
    txBuffer[10] = (uint8_t)(Time_Sec);            // rsvbyte2
    
    //Calc buffer CRC
    crc = appCalcCRC(txBuffer, TX_LENGHT);
    txBuffer[11] = crc;                            // CRC
}




/* ##################################################
  ############## RX Frame Builder #################
#####################################################*/
void RXFrameBuilder(void)
{
    while (Serial.available() > 0)
    {
        //Serial.println("RX Available!");
        uint8_t byteRx = Serial.read();

        // Wait for the Preamble
        if (indexRx == 0)
        {
            if (byteRx == PREAMBLE)
            {
                rxBuffer[indexRx] = byteRx;
                indexRx++;
            }
        }
        else
        {
            rxBuffer[indexRx] = byteRx;
            indexRx++;

            // Completed Frame
            if (indexRx >= RX_SIZE)
            {
                if (validateFrame())
                {
                    frameReady = true;
                }

                // Reset Index...
                indexRx = 0;
            }
        }
    }
}


bool validateFrame(void)
{
    bool response = false;
    uint8_t crc_calc = 0x00;

    // Check Preamble:
    if (rxBuffer[0] != PREAMBLE)
    {
        return(response); //Return.. Frame is corrupted
    }

    // CRC Check
    crc_calc = appCalcCRC(rxBuffer, RX_LENGHT);

    if(crc_calc == rxBuffer[RX_SIZE-1])
    {
      response = true;
    }
    else
    {
      response = false;
    }
    
    return (response);
}



/* ##################################################
  ################ Check Prescaler ###################
#####################################################*/
uint16_t check_prescaler(void)
{
  uint8_t cs_bits;
    
  // extract CS12:CS10 (bits 2:0)
  cs_bits = TCCR1B & 0x07;

  switch (cs_bits)
  {
      case 0x00: return 0;     // Timer OFF
      case 0x01: return 1;
      case 0x02: return 8;
      case 0x03: return 64;
      case 0x04: return 256;
      case 0x05: return 1024;
      default:   return 0;     // Invalid
  }
}



/* ##################################################
  ################ Time Processing ##################
#####################################################*/
void timeProcessing(void)
{
    if(Time_Cnt >= (1000U/SYSTICK_MS))
    {
        Time_Cnt = 0;
        Time_Sec++;
  
        if(Time_Sec >= 60)
        {
          Time_Min++;
          Time_Sec = 0;
        }
      
        //Serial.print("Min: ");
        //Serial.print(Time_Min);
        //Serial.print("  Secs: ");
        //Serial.println(Time_Sec);
    }
}




/* ##################################################
  ################ CRC Algorithm ###################
#####################################################*/
uint8_t appCalcCRC(const uint8_t *data, uint8_t length)
{
    crc = 0x00;

    for (uint8_t i = 0; i < length; i++) 
    {
        crc = crcTable[crc ^ data[i]];
    }
    return crc;
}
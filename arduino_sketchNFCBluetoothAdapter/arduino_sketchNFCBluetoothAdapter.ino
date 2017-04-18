// the sensor communicates using SPI, so include the library:
#include <SPI.h>

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse
byte TXBuffer[40];    // transmit buffer
byte RXBuffer[40];    // receive buffer
byte NFCReady = 0;  // used to track NFC state


void setup() {
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); // Wake up pulse
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);

    Serial.begin(9600);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
 
 // The CR95HF requires a wakeup pulse on its IRQ_IN pin
 // before it will select UART or SPI mode.  The IRQ_IN pin
 // is also the UART RX pin for DIN on the BM019 board.
 
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
}

/* SetProtocol_Command programs the CR95HF for
ISO/IEC 15693 operation.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display successful programming. 
*/
void SetProtocol_Command()
 {
 byte i = 0;
 
// step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);
 
// step 2, poll for data ready

  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

// step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))
  {
     NFCReady = 1; // NFC is ready
  }
  else
  {
     NFCReady = 0; // NFC not ready
  }
}

/* Inventory_Command chekcs to see if an RF
tag is in range of the BM019.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display the the RF tag's universal ID.  
*/
void Inventory_Command()
 {
 byte i = 0;

// step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);
 
// step 2, poll for data ready
// data is ready when a read byte
// has bit 3 set (ex:  B'0000 1000')

  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);
// step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)
  {  
    NFCReady = 2;
    }
  else
    {
    NFCReady = 1;
  }
    
}

String bytesToHex(byte b[]){
   String str;
 
  return str;
}

String Read_Data_And_Send(){
  String str = ""; 
  for(int i = 3; i<40; i++){
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    SPI.transfer(0x04);  // Send Receive CR95HF command
    SPI.transfer(0x03);  // length of data that follows is 0
    SPI.transfer(0x02); // request Flags byte
    SPI.transfer(0x20);  // Inventory Command for ISO/IEC 15693
    SPI.transfer(i);  // mask length for inventory command
    digitalWrite(SSPin, HIGH);
    delay(1);
 
    digitalWrite(SSPin, LOW);
    while(RXBuffer[0] != 8)
    {
      RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
      RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
    digitalWrite(SSPin, HIGH);
    delay(1);

    digitalWrite(SSPin, LOW);
    SPI.transfer(0x02);   // SPI control byte for read         
    RXBuffer[0] = SPI.transfer(0);  // response code
    RXBuffer[1] = SPI.transfer(0);  // length of data
    for (byte i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data
    digitalWrite(SSPin, HIGH);
    delay(1);
    //Cut off header bytes
    for(int i=0; i<8; i++){
      RXBuffer[i] = RXBuffer[i+3];
    }

unsigned char * pin = RXBuffer;
  char string[14];
  char * hex = "0123456789ABCDEF";
  char * pout = string;
  for(; pin < RXBuffer+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pin[0] = 0;
    str +=string;
  }
  Serial.println(str);
  return str;
}

void loop() {
  if(Serial.available()>3){
    String s = Serial.readString();
    if(s.substring(0,4).equals("Send")){
      for(int j = 0; j<10; j++){
        if(NFCReady == 0)
        {
          SetProtocol_Command(); // ISO 15693 settings
          delay(1000);
        }
        else if (NFCReady == 1)
        {
          Inventory_Command();
          delay(1000);    
        }  else {
          Read_Data_And_Send();
          NFCReady =1;
          delay(1000);
          break;
          
        }
      }
      NFCReady = 0;
    }
  }
  
  
}


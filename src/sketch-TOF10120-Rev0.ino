
/*
 * 
 * Created by George R Canepa 1/25/21
 * Needed a project to practice using GitHub repository
 * Use as is, no restrictions, not warranty is provided
 * Just remember that you get what you pay for
 * 
 * TOF10120 is a good little sensor, but I could not find much I2C reference code other than the basic distance read
 * At some point I want to use a few in the same application, so need to be able to read multiple sensors on the same I2C bus
 * 
 * I ended up having to dechiper the Chinese in the datasheet (Google translate is amazing for this)
 * and figured out how to do all the I2C read/write commands. Not all versions of the datasheet online have the I2C table
 * I posted the version I used in the documents folder of this repository (listed below)
 * 
 * Surprised as to how some of the online reviews comparing this sensor to the Ultra Sound sensors did not recognize 
 * how much better this sensor is
 * 
 * I use it for my Robot, and it does an excellent job detecting walls, not matter at what angle
 * Ultra Sound sensors are useless when you approach the wall at an angle
 * 
 * This sensor can also be calibrated, making it extremly accurate
 * 
 * This code was created from my robot control code (for my own reference, as this reference code is not public)
 * "C:\Users\georg\Documents\Arduino\sketch-8266-Motor-VoltMeter-Rev1.3\sketch-8266-Motor-VoltMeter-Rev1.3.ino"
  - Changed to TOF10120
  - disabled checkWifi and checkondistance and any ultrasound call
  - Not getting data from the TOF10120, need to check on a scope (do I need the 10K pull up resistors? no)
  - Got it working with UART mode
  - Eventually got it working with I2C. Created a Git repository https://github.com/grcanepa/TOF10120 with the TOG10120 specific code
*  
* Rev 0.0
*   Initial rev
*   Includes an I2C scanner (in case you need a scanner to find the device)
*   Has code to write deviation and I2C address, but not active by default (writing is to EEprom and there is a cycle limit on EEprom)
*   
*   Code to set a new i2c address 
    *  Does to a new value of: 0xA8 which is 0x54 (b7-b1) which is 84 (int) value given to Wire.beginTransmission(i2c_Address);
    *  From a default i2c_Address of 82 (0xA4 which is 0x52 (b7-b1) which is what shows on an i2c scanner
* 
*   Has code to do all the read operations (RealTime and Filtered Data, Data Deviation, Various Data send/Receive modes, and I2C address)
*   The GitHub repository has the original Chinese version of the datasheet, and other notes I have captured along the way
*   In case you got the code from another source, my original version of this code is at: https://github.com/grcanepa/TOF10120
*  
*   I included some UART code that I had developed to use the UART mode (default read only)
*   I posted this primarily for the I2C code, as I could not find any I2C write code online
*   
*   I did get 3 sensors on the same bus one at 0x52, another at 0x53 and the third at 0x54
*/

//Library for I2C SDC/SDL uses D3/D4 on ESP8266

#include <Wire.h>
#include <SoftwareSerial.h>

//Initialize SoftwareSerial (for UART)
SoftwareSerial ss(D1); 

float tofDist = 0;

//Buffer used for I2C read operations
unsigned char i2c_rx_buf[16];

//UART Variables
char* distC = "01234567890123456789";
int countC = 0;
int countM = 0;

//I2C Variables
int i2c_Address = 82; //int value used by Wire.beginTransmission(i2cAddress)
                      //scanI2C() sees it as 0x52
                      //write 0xA4 using 0x0f command to set sensor to this address

void setup() { //----------------------------------------------------------------SETUP START -------------------------------------------

//To talk to the PC so as to display values
Serial.begin(9600);
delay(100);
Serial.println("Started Serial in Setup");

//Start Software serial for uArt
ss.begin(9600); 

//Initialize "Wire" for I2C
Wire.begin(D3,D4);  //start Wire I2C communication, and set SDA - SCL  pins  
                    //Can include an address as 3rd parameter, it would be the address of the master (Arduino)

//Check for I2C addresses (Any device connected to the Wire I2C pins D3, D4)
scanI2C();

//Option to set the distance deviation for the sensor (calibrate accuracy +/-99mm
//Sensors have a default calibration value set at the factory

int distDevSetResponse;
//To change the deviation value (must be a EEPROM bit so do sparingly
//Deviation a two byte write, should work the same way to write a new I2C address, a 1 byte write
//Should be GoSensorWrite(ox0f, true, 0x53) ; //to change to 0x53 from 0x52 or maybe 0xA6 since there is a shift)
if (false) {
    distDevSetResponse = GoSensorWriteRead(0x06, true, 0xff, 0xda); // 0xdd is -35, 0xda is -38 (original) to "deviation" address
    
    Serial.println("");
    Serial.print(" Set Distance Deviation: ");
    Serial.println(distDevSetResponse, HEX);                                            
    delay(10);
}

//Option to wrie the  tof10120 i2c address to 0xA8, which would scan as 0x54 and be set in Wire.beginTransmission(84)
int i2cAddressSetResponse=0;

if (false) {
    i2cAddressSetResponse = GoSensorWriteRead(0x0f, true, 0xA8); //  0xA6 b7-b0 is 0x53 b7-b1
                                                                 //  So write as 0xA6 and then access with 83 int on "wire"
                                                                 
                                                                 //  0xA8 is 0x54 b7-b1
                                                                 //  So write as 0xA8 and then access with 84 int on "wire"
                                                                 
                                                                 //  To get back to default, set i2c_Address to the int version 
                                                                 //  of the address from the scan and then
                                                                 //  write 0xA4 which is 0x52 b7-b1 and then access with 82 int on "wire"
    
    Serial.println("");
    Serial.print(" Set I2C address response: ");
    Serial.println(i2cAddressSetResponse, HEX);  
    
    //It takes a while for the device to respond to a scan, this delay is not calibrated                                          
    delay(2000);
    i2c_Address = 84; //New int location to give to "wire" to access i2c at 0xA8 that respondes with 0x54 on a scan
}

//Check for I2C addresses (of any device on Wire I2C pins)
scanI2C();

} //End of setup routine ----------------------------------------------- END OF SETUP ----------------------------------------

// the loop function runs over and over again forever
void loop() { // ------------------------------------------------------- LOOP START ------------------------------------------

//Setup to read data from 3 sensors that share the I2C pins, which scan at 0x52, 0x53, 0x54
//And have Wire.beginTransmission(address) set to 82,83,84

Serial.println("In Loop");

// Inc ase you want to scan every loop
if (false) scanI2C();

//Show data 
Serial.print("Data 0x52: ");
readTof10120Data(82);
Serial.print("Data 0x53: ");
readTof10120Data(83);
Serial.print("Data 0x54: ");
readTof10120Data(84);

delay(1);
    
Serial.println("End of Loop");

} //End of "Loop"  ----------------------------------------------------------------------


//FUNCTIONS

//Scans all possible I2C addresses, and reports the ones that are "alive"
void scanI2C() {

  byte error, address;
  int nDevices;
 
  Serial.println("Scanning for I2C addresses...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
 
    Wire.beginTransmission(address);      //Sends a request to address
    error = Wire.endTransmission();       //Captures response
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
   
}

void readTof10120Data(int i2c_Address_Local) {

//Set the i2cAddress to read per request
i2c_Address=i2c_Address_Local;

//Display I2C TOF10120 Readable parameters for i2c_Address

int xfilt = GoSensorRead(0x04);
  Serial.print(" Filt Dist: ");
    Serial.print(xfilt);
    delay (1);

//For quick mode change to if (true). I was getting a loop that printed all three values in 324ms
if (false) {Serial.print(""); return; } 
  
int x_mm = GoSensorRead(0x00);
    Serial.print("Real time Dist: ");
    Serial.print(x_mm);
    delay(1);
    
int xdev = GoSensorWriteRead(0x06,false,0x00,0x00);
    Serial.print(" Dist Dev: ");
    Serial.print(xdev);  
    delay(1);
  
int addI2C = GoSensorWriteRead(0x0f,false,0x00,0x00);
    Serial.print(" I2C Addr: ");
    Serial.print(addI2C);
    delay(1);

int distSmode = GoSensorWriteRead(0x09,false,0x00,0x00);
    Serial.print(" Dist Send Mode: ");
    Serial.print(distSmode);
    delay(1);

int distDmode = GoSensorWriteRead(0x08,false,0x00,0x00);
    Serial.print(" Dist Data Mode: ");
    Serial.print(distDmode);
    delay(1);

int distMax = GoSensorWriteRead(0x0c,false,0x00,0x00);
    Serial.print(" Dist Max Value: ");
    Serial.println(distMax);
    delay(10);

}


//Default i2c_Address is set to "82", Takes command "addr" , expects cnt bytes on the read, content goes into datbuf
//There is a read after the write to get the data from the read operation
//This is the lowest level read/write function
//There are other functions that make it easier to setup calls to this function
void SensorWriteRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt, boolean writeOp, byte hByte, byte lByte) 
{
  unsigned short result=0;
  // step 1: Set the I2C address
  Wire.beginTransmission(i2c_Address); // I2C address is set by the top 7 bits in the byte (8th bit determines if doing a read or write)
                              // the address specified in the TOF10120 datasheet is 164 (0xa4) but is vs an 8bit reference
                              // Wire library wants the address on a 7bit reference
                              // So need to shift the 0xa4 right by 1 bit which gives you 0x52 or 82 decimal (164/2)
                              // Can find the I2C address by using i2c_scanner code
                              // Reads back as 164 as it reads back shifted by a bit (1 bit shift left is *2 so 82 => 164)
                              
  // step 2: Set the TOF10120 register address                            
  Wire.write(byte(addr));      // sets (sends) write data address (addr) within the TOF10120
                               // addr = 0x00 ( for distance) (no write bytes, 2 read bytes) (returns distance)
                               // addr = 0x04 ( for filtered distance) (no write bytes, 2 read bytes) (returns distance)
                               // addr = 0x06 ( for setting "deviation" (i.e. offset calibration)(2 write bytes, 2 read bytes) (returns deviation)
                               // addr = 0x08 ( for distance data mode (1 write byte 1 read byte) (0=filtered 1=RealTime) (returns 0 or 1)
                               // addr = 0x09 ( for distance sending method (1 write byte, 1 byte read) (0=uart, 1=uart&&I2C) (returns 0 or 1)
                               // addr = 0x0f ( for I2C address (1 write byte, 1 read byte) (returns I2C address (on b7-b1 b0=0)
                               
  

  // step 3: If a write Operation write the data bytes (none on read, 1 or 2 as specified for addr)
  if (writeOp) {

    //Read provides higher order byte 1st, then lower order byte, write is the same;
    
    //These bytes are to set "deviation" to -37mm (0xffxb)
    //byte hByte = 0xff;  //range is +/- 99 but wants two bytes so high byte is always 0xff
    //byte lByte = 0xdc;  //0xd9 would be -39 0xda would be -38 0xdb is -37 0xdc is -36
                        //+99 is 0x63 to -99 should be 0x9d (As 0xff is -1 so 0xff - (98=0x62) is 0x9d)
  
    //If writing always write the  hByte  
    Wire.write(hByte);
    
    //if writing 2 bytes, write the lByte
    if (cnt==2)
        Wire.write(lByte);
  }

  // step 4: End the write sequence (which is what triggers the Wire to initiate the write sequence)
                  
  Wire.endTransmission();      // stop transmitting (actually all the .write are stored in a buffer
                               // the .end triggers the write of the entire buffer 
                               
  // step 5: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  
  // step 6: request reading from sensor
  Wire.requestFrom(i2c_Address, cnt);    // request cnt bytes from slave device at 82 (0x52) same 8 vs 7 bit shift vs datasheet
                                // cnt = 2 (2 bytes is enough to get 0-2000 or 1 for other registered as set in step2)
                                // These must go into an internal buffer
                                
  // step 7: process data received from the sensor
  if (cnt <= Wire.available()) {// if two bytes were received issue 2 Wire.read() commands
    *datbuf++ = Wire.read();    // receive high byte (overwrites any previous reading in the buffer)
    if (cnt == 2) 
      *datbuf++ = Wire.read();  // If expecting 2 bytes, read the 2nd byte
      
                                // receive low byte as lower 8 bits
                                // There may be two .read() since it may receive 2 bytes
                                // It must just be pulling the bytes from a buffer internalto Wire
                                // *databuf is assigned to char i2c_rx_buf[16]
                                // *databuf++ must point to i2c_rx_buf[0]
                                // The 2nd *databuf must point to i2c_rxbuf[1]
                                // Not sure why *databuf is not already pointing to i2c_rxbuf[0]
  }
}

//Simpler read only sensor call
int GoSensorRead(unsigned char addrS) {
  //A read has write as "false" and data bytes as 0x00
  //The function figures out the "byte count" that goes with each address
  return GoSensorWriteRead(addrS, false, 0x00, 0x00);
}

//Single byte sensor call (for write but can be used to read)
int GoSensorWriteRead(unsigned char addrS, boolean writeOp, byte byteH) {
  //A read has write as "false" and data bytes as 0x00
  //The function figures out the "byte count" that goes with each address
  return GoSensorWriteRead(addrS, writeOp, byteH, 0x00);
  
}

//Dual byte sensor call (bytes are for write but can also be used to read, as bytes are ignored when in read mode)
//read/write byteCnt are always the same. Set by address. readOp if false writeOp if true
int GoSensorWriteRead(unsigned char addrS, boolean writeOp, byte byteH, byte byteL) {

  short length_S = 0;        //A short is 2 bytes (use signed for "deviation")
  
  unsigned short length_U = 0; //A short is 2 bytes  (use unsigned for "distance")
  
  //If unsigned two byte commands (read distance real time or filtered)
  if (addrS==0x00 || addrS==0x04) {
      SensorWriteRead(addrS,i2c_rx_buf,2, false, byteH, byteL);
      length_U=i2c_rx_buf[0];
      length_U=length_U<<8;
      length_U|=i2c_rx_buf[1];
      delay(100); 
      return length_U;
    }
  //else if 2 byte signed read/write deviation (calibration offset)
  else if (addrS==0x06) {
      SensorWriteRead(addrS,i2c_rx_buf,2, writeOp, byteH, byteL);
      length_S=i2c_rx_buf[0];
      length_S=length_S<<8;
      length_S|=i2c_rx_buf[1];
      delay(100); 
      return length_S;
  
    }
  //else if 1 byte unsigned (read/write mode/I2C address)
  else if (addrS==0x08 || addrS==0x09 || addrS==0x0c || addrS==0x0f) {
      SensorWriteRead(addrS,i2c_rx_buf,1, writeOp, byteH, byteL);
      length_U=i2c_rx_buf[0];
      delay(100); 
      return length_U;
    
    }
}

//UART based distance call not currently called, but may eventually want to have uart reference code
//UART read is a bit messy as there is a need to synch with the character stream. They is probably a better way
int tofDistanceUART() {

boolean tofTrace = false;
int distI = 0;

  if (tofTrace) Serial.println("PreWPreDelay:");
    
  //delay (500);
  
  //Reset the char* variable that captures the distance readings
  for (int i = 0; i<20; i++) distC[i]=' ';
  
  //Reset the counters (M for the number of 'm' and C for overall character count)
  countM=0;
  countC=0;

  //ss.write("s20#");should be able to issue write commands on the rx/tx lines and listen for response
  
  //If there are characters waiting
  if (ss.available()>=6) {
    if (tofTrace) Serial.println(ss.available());

    while (ss.available()>0)
      {
            char c = ss.read(); 
            if (tofTrace) Serial.print(c); //to see the raw gps data, should print it even if it does not encode
  
            //count the 'm' as 'mm' is the terminating toke
            if (c=='m') 
              countM++;
  
            //keep add to distC if pointer is in range
            if (countC < 20) 
              distC[countC]=c;
  
            //Increment the counter  
            countC++;
  
            //If charatcer is a linefeed and m count indicates 'mm' reset the counters
            if (c==0x0a&&countM==2) {
              countC=0;
              countM=0;
            }
            
      }

      if (tofTrace) {
        Serial.println("PostW:");
        Serial.println(distC);
        Serial.println("PostPrintDistC");
      }
  
  //Extract the distance from the char*
  
  //pointer for first 'm' in distC
  int endDist=0;
  
  //Find the first 'm'
  for (int i = 0; i<20; i++) {
    if (distC[i]=='m') {
      endDist=i;
      break;
      }
  }
  
  //4 character variable to store the distance as char*
  char* distO = "0000";
  
  //Set the "active" portion of the variable
  for (int i = 0; i<endDist; i++)
    distO[4-endDist+i] = distC[i];
  
  //Set the '0' portion of the variable
  for (int j = 0; j <4-endDist; j++)
    distO[j]='0';
  
  //Convert the variable to an int
    for (int j = 0; j<4; j++)
      distI = distI + (distO[j]-'0')*pow(10,3-j);
  
  if (tofTrace) {
      Serial.print("endDist: "); Serial.print(endDist);
      Serial.print("distO: ") ; Serial.print(distO);
      Serial.print("IntDist: "); Serial.println(distI);
  }
  
  }

  if (distI >1 && distI < 2001) 
    return distI;
  else 
    return (int) tofDist;
}

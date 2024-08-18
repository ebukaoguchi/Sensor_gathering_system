#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//Need to be able to store error codes during a reboot
#include <FlashStorage.h>

//define the parameters here
int idPin = A0;    // Analog input pin for reading sensor ID voltage
int idValue = 0;  // Variable to store the direct (unprocessed, before ADC) ID voltage coming from the sensor
int id_send = 0; // Representing sensor type with simpler integer ID for conveniently fitting in the message and sending  
int sensorvalue = 0; // Variable to store the processed (after ADC) sensor output value
int sensorid = 0; // Variable to store the processed (after ADC) ID voltage of the sensor
int sensor_intake = 0; // Variable to store the direct (unprocessed, before ADC) sensor output value
int sensorPin = A2; // Analog input pin for reading sensor output
uint16_t errorcode = 0x0000; //reset error code
int ID = 123; // set up for identification ID
unsigned long timeStamp; // declaring timestamp
int packetCounter = 0; //set up packet ID
int packet[7]; //packet set up
int NodeID = 1; //set up node ID
//#include <FlashStorage.h>
//Reserver a place in the flash memory
FlashStorage(error_store, uint16_t);

#define TX_INTERVAL 4000 //Delay between each message in millidecond.

  // Pin mapping for SAMD21
 const lmic_pinmap lmic_pins = {
 .nss = 12,//RFM Chip Select
 .rxtx = LMIC_UNUSED_PIN,
 .rst = 7,//RFM Reset
 .dio = {6, 10, 11}, //RFM Interrupt, RFM LoRa pin, RFM LoRa pin
 };
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void onEvent (ev_t ev) {
}
osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);
// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
 os_radio(RADIO_RST); // Stop RX first
 delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
 LMIC.dataLen = 0;
 while (*str)
 LMIC.frame[LMIC.dataLen++] = *str++;
 LMIC.osjob.func = func;
 os_radio(RADIO_TX);
 SerialUSB.println("TX");
}
// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
 LMIC.osjob.func = func;
 LMIC.rxtime = os_getTime(); // RX now
 // Enable "continuous" RX (e.g. without a timeout, still stops after
 // receiving a packet)
 os_radio(RADIO_RXON);
 SerialUSB.println("RX");
}
static void rxtimeout_func(osjob_t *job) {
 digitalWrite(LED_BUILTIN, LOW); // off
}
static void rx_func (osjob_t* job) {
 // Blink once to confirm reception and then keep the led on
 digitalWrite(LED_BUILTIN, LOW); // off
 delay(10);
 digitalWrite(LED_BUILTIN, HIGH); // on
 // Timeout RX (i.e. update led status) after 3 periods without RX
 os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);
 // Reschedule TX so that it should not collide with the other side's
 // next TX
 os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);
 SerialUSB.print("Got ");
 SerialUSB.print(LMIC.dataLen);
 SerialUSB.println(" bytes");
 SerialUSB.write(LMIC.frame, LMIC.dataLen);
 SerialUSB.println();
 // Restart RX
 rx(rx_func);
}
static void txdone_func (osjob_t* job) {
  //rx(rx_func);
}
// log text to USART and toggle LED
static void tx_func (osjob_t* job) {
////start sending package////
 SerialUSB.println("Sending message");
 
 int ecod2 = 0; //leave a place for future value in the packet
 int temp = 0;
 int soil = 0;
// using counter to write errorcode of sensor value greater than 4000 and less than 0
  packetCounter = packetCounter + 1;
  if (sensorvalue > 4000 || sensorvalue < 0){
      bitWrite(errorcode,14,1);
    }
 timeStamp = millis(); //timestamp as a millis function
 //set up packet  posizition
 packet[0] - ID; //Identification code
 packet[1] = timeStamp; // timestamp in millis
 packet[2] = NodeID; // node identification
 packet[3] = id_send; //sensor type
 packet[4] = packetCounter; //packet id
 packet[5] = sensorvalue; //sensor reading
 packet[6] = 1; //error code
 packet[7] = 0; //extra space for error code
 char test[100];

if ((sensorid>=1980) && (sensorid <=2980)){ // We set 2480 mV as ID voltage of Temperature sensor. Using a +/- threshold of 500 mV, we get the range: 1980 mV to 2980 mV
    sensor_intake = analogRead(sensorPin); // Reading raw sensor output value (from A2)
    sensorvalue = map(sensor_intake, 0, 1023, 0, 3300); // Converting raw sensor output value using ADC, format: map(raw_value,0,ADC_resolution,0,system_voltage). system_voltage = 3300 mV for SparkFun SAMD21 RF Pro
    temp = ((sensorvalue-100.00)/10.00)-40.00; //equation to convert analog input to degree celsius
    sprintf (test,"ID%i,Time(ms)%i,Node%i,Temp,Packet%i,value%i,error1 %i,error2 %i", ID, timeStamp,NodeID,packetCounter,temp,errorcode, ecod2);//generate packet
    SerialUSB.println(test);//reading the packet
    temp = 0.00;//reset temperature to 0
    tx(test,txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}
// application entry point
// Then checking if it is a Soil Moisture sensor
else if ((sensorid>=300) && (sensorid <=1300)){ // We set 812 mV as ID voltage of Soil Moisture sensor. Using a +/- threshold of 500 mV, we get the range: 300 mV to 1300 mV
    SerialUSB.println("Soil Moisture Sensor"); // Print the sensor type
    id_send = 2; // Setting Soil Moisture sensor ID as 2 for message sending convenience
    sensor_intake = analogRead(sensorPin); // Reading raw sensor output value (from A2)
    sensorvalue = map(sensor_intake, 0, 1023, 0, 3300); // Converting raw sensor output value using ADC, format: map(raw_value,0,ADC_resolution,0,system_voltage). system_voltage = 3300 mV for SparkFun SAMD21 RF Pro
    SerialUSB.println(sensorvalue); // Continupusly printing sensor readings
    soil = ((55.7 - sqrt(3102.49-(3000-sensorvalue)))/(2*1.3)); //equation to convert analog input to soil moisture
    sprintf (test,"ID%i,Time(ms)%i,Node%i,Soil,Packet%i,value%i,error1 %i,error2 %i", ID, timeStamp,NodeID,packetCounter,soil,errorcode, ecod2); //generate packet
    SerialUSB.println(test);//reading the packet
    soil = 0;//reset soil moisture to 0
    tx(test,txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
  }
  }

void setup() {
 SerialUSB.begin(115200);
 while(!SerialUSB);
 SerialUSB.println("Starting");

 pinMode(LED_BUILTIN, OUTPUT);
 // initialize runtime env
 os_init();
 // this is automatically set to the proper bandwidth in kHz,
 // based on the selected channel.
 uint32_t uBandwidth;
 LMIC.freq = 903900000;
 uBandwidth = 125;
 LMIC.datarate = US915_DR_SF7; // DR4
 LMIC.txpow = 21;
 // disable RX IQ inversion
 LMIC.noRXIQinversion = true;
 // This sets CR 4/5, BW125 (except for EU/AS923 DR_SF7B, which uses BW250)
 LMIC.rps = updr2rps(LMIC.datarate);
 SerialUSB.print("Frequency: "); SerialUSB.print(LMIC.freq / 1000000);
           SerialUSB.print("."); SerialUSB.print((LMIC.freq / 100000) % 10);
           SerialUSB.print("MHz");
 SerialUSB.print(" LMIC.datarate: "); SerialUSB.print(LMIC.datarate);
 SerialUSB.print(" LMIC.txpow: "); SerialUSB.println(LMIC.txpow);
 // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
 LMIC.rps = updr2rps(LMIC.datarate);
 // disable RX IQ inversion
 LMIC.noRXIQinversion = true;
 SerialUSB.println("Started");
 SerialUSB.flush();
 // setup initial job
 os_setCallback(&txjob, tx_func);
//}


delay(2000); // Giving analog input pins a couple of seconds to read the analog voltages properly

idValue = analogRead(idPin); // Reading raw ID voltage (from A0)
sensorid = map(idValue, 0, 1023, 0, 3300); // Converting raw ID voltage using ADC, format: map(raw_value,0,ADC_resolution,0,system_voltage). system_voltage = 3300 mV for SparkFun SAMD21 RF Pro
//// Detecting sensor type based on the ID voltage ////

// First checking if it is a Temperature sensor
if ((sensorid>=1980) && (sensorid <=2980)){ // We set 2480 mV as ID voltage of Temperature sensor. Using a +/- threshold of 500 mV, we get the range: 1980 mV to 2980 mV
    SerialUSB.println("Temperature Sensor"); // Print the sensor type
    id_send = 1; // Setting Temperature sensor ID as 1 for message sending convenience

  }
 
// Then checking if it is a Soil Moisture sensor
else if ((sensorid>=300) && (sensorid <=1300)){ // We set 812 mV as ID voltage of Soil Moisture sensor. Using a +/- threshold of 500 mV, we get the range: 300 mV to 1300 mV
    SerialUSB.println("Soil Moisture Sensor"); // Print the sensor type
    id_send = 2; // Setting Soil Moisture sensor ID as 2 for message sending convenience
  }

else {
  SerialUSB.println("Sensor unidentified !");
  }
}




void loop() {
  // execute scheduled jobs and events
  printerrorcode(packet,errorcode); 
os_runloop_once();// run code
sensor_intake = analogRead(sensorPin); // Reading raw sensor output value (from A2)
sensorvalue = map(sensor_intake, 0, 1023, 0, 3300); // Converting raw sensor output value using ADC, format: map(raw_value,0,ADC_resolution,0,system_voltage). system_voltage = 3300 mV for SparkFun SAMD21 RF Pro
//SerialUSB.println(sensorvalue); // Continupusly printing sensor readings
//error code
delay(2000); // Delay between two sensor reading values. Can be varied based on application

    //pet the dog
    WDT->CLEAR.reg = 0xA5;

    os_runloop_once();// run code
  }


//function for the resetcause
void resetcause(){
    //determine reset type
    uint8_t ireset = PM->RCAUSE.reg;
    bitWrite(errorcode, 0, bitRead(ireset, 6));
    bitWrite(errorcode, 1, bitRead(ireset, 5));
    bitWrite(errorcode, 2, bitRead(ireset, 4));
}

uint16_t readstatusreg() {
    uint8_t statusareg = REG_DSU_STATUSA; // DSU Registers A assigned to 8 bit Unsigned int
    uint8_t statusbreg = REG_DSU_STATUSB; // DSU Registers B assigned to 8 bit Unsigned int

    uint16_t errorcodes = 0x0000; //reset error codes

    errorcodes = statusareg; //assign status register to the error codes
    errorcodes = errorcodes << 8;// left shift
    errorcodes |= statusbreg; // bitwise OR

    return errorcodes; //return value                                
}

// function to read the error code stored in the flash storage
void readstorederror(){
    uint16_t tempcode = error_store.read();
    bitWrite(errorcode, 3, bitRead(tempcode, 12));
    bitWrite(errorcode, 4, bitRead(tempcode, 11));
    bitWrite(errorcode, 5, bitRead(tempcode, 10));
    bitWrite(errorcode, 6, bitRead(tempcode, 9));
    bitWrite(errorcode, 7, bitRead(tempcode, 4));
    bitWrite(errorcode, 8, bitRead(tempcode, 3));
    bitWrite(errorcode, 9, bitRead(tempcode, 2));
    bitWrite(errorcode, 10, bitRead(tempcode, 1));
}
 
void WDT_Handler() {
    //SerialUSB.println("WDT Interrupt");
    error_store.write(readstatusreg());// write the error into the reead status register
}

// splitting the errorcode, takes in 2 arguments
int spliterrorcode(uint16_t ierror, uint8_t firstlast){
    uint8_t xlow = ierror & 0xff; 
    uint8_t xhigh = (ierror >> 8);// left shift
    //using if state to return a high or a low
    if (firstlast == 1){
        return xlow;
    } else if (firstlast == 2){
        return xhigh;
    }
}
//print the error code which takes in 2 argument, packet at position7 and the error 16 bits for errors greater than 0
void printerrorcode(int packet[7], uint16_t ierror){
    if (ierror > 0){
        SerialUSB.print("Node "); // print the node
        SerialUSB.print(packet[0]);// packet position
        SerialUSB.println(" had the following error(s):"); //print the string
        //using for loop to switch between different cases for different errors to log
        for (uint8_t i = 0; i < 16; i++){
            if bitRead(ierror, i){
                switch (i) {
                    case 0:
                        SerialUSB.print("Power on reset = ");
                        break;
                    case 1:
                        SerialUSB.print("WDT reset = ");
                        break;
                    case 2:
                        SerialUSB.print("External Reset = ");
                        break;
                    case 3:
                        SerialUSB.print("Protection Error = ");
                        break;
                    case 4:
                        SerialUSB.print("DSU Failure = ");
                        break;
                    case 5:
                        SerialUSB.print("Bus Error = ");
                        break;
                    case 6:
                        SerialUSB.print("CPU Reset Phase Extension = ");
                        break;
                    case 7:
                        SerialUSB.print("Hot-Plugging Enabled = ");
                        break;
                    case 8:
                        SerialUSB.print("Debug Comm 1 = ");
                        break;
                    case 9:
                        SerialUSB.print("Debug Comm 2 = ");
                        break;
                    case 10:
                        SerialUSB.print("Debugger Present = ");
                        break;
                    case 11:
                        SerialUSB.print("Missing Packet = ");
                        break;
                    case 12:
                        SerialUSB.print("Reception Failure = ");
                        break;
                    case 13:
                        SerialUSB.print("Transmit Failure = ");
                        break;
                    case 14:
                        SerialUSB.print("Temp. above 100 or below 0 = ");
                        break;
                    case 15:
                        SerialUSB.print("reserved");
                        break;
                    }
                SerialUSB.println(bitRead(ierror, i));
            }
        }
    }
}
//function to setup watchdog timer
void WatchDogSetup(){
    //Disable the WDT
    WDT->CTRL.reg = 0; // Disable watchdog for config

    // Generic clock generator 2, divisor = 32 (2^(DIV+1))
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
    // Enable clock generator 2 using low-power 32KHz oscillator.
    // With /32 divisor above, this yields 1024Hz(ish) clock.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
    while(GCLK->STATUS.bit.SYNCBUSY); // Think about why this is used.
    // The above while loop is basically used as a hold to have the program wait for the clocks to syncronize.
    // WDT clock = clock gen 2
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;

    // Enable WDT early-warning interrupt
    NVIC_DisableIRQ(WDT_IRQn); // first disable interupt before configuration
    NVIC_ClearPendingIRQ(WDT_IRQn);// clear pending interrupt
    NVIC_SetPriority(WDT_IRQn, 0); // Top priority
    NVIC_EnableIRQ(WDT_IRQn); // enable interupt

    WDT->INTENCLR.bit.EW = 1; // Enable early warning interrupt
    // Set the early warning interrupt control
    WDT->EWCTRL.bit.EWOFFSET = 0x8;
    WDT->INTENSET.bit.EW = 1; // Enable early warning interrupt

    WDT->CONFIG.bit.PER = 0x9; // Set period for chip reset from the datasheet
    WDT->CTRL.bit.WEN = 0; // Disable window mode

    //Enable the WDT
    WDT->CTRL.bit.ENABLE = 1;

    //pet the dog
    WDT->CLEAR.reg = 0xA5;

    
}

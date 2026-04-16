//==============================================================================================================================================================
//==============================================================================================================================================================
//        This is the firmware-script for the PolArStat was written by Tim Tichter.
//
//==============================================================================================================================================================
//==============================================================================================================================================================
//      Here, the libraries for the external ADC (ADS1115) and DAC(MCP4725) are loaded and the I2C adresses are selected for communication
//==============================================================================================================================================================
//==============================================================================================================================================================
#include "Wire.h"                                                                   //  Include library for setting wire clockspeed of the I2C communication
#include "ADS1X15.h"                                                                //  Include the library for the ADS1115 external ADC
#include "MCP4725.h"                                                                //  Include the library for the MCP4725 external DAC
MCP4725 MCP(0x60);                                                                  //  Initialize the MCP4725 at the 0x60 address
ADS1115 ADS(0x48);                                                                  //  Initialize the ADS1115 at the 0x48 address
//==============================================================================================================================================================
//==============================================================================================================================================================
//      Here, the variables for the INPUTS of the CV are initialized
//==============================================================================================================================================================
//==============================================================================================================================================================
float         E_in                   =  0;                                          //  0x10  = assotiated byte to initial CV potential
float         E_first_vertex         =  0;                                          //  0x11  = assotiated byte to first vertex CV potential
float         E_second_vertex        =  0;                                          //  0x12  = assotiated byte to second vertex CV potential
float         E_final                =  0;                                          //  0x13  = assotiated byte to final CV potential
int           Cycles                 =  0;                                          //  0x14  = assotiated byte for cycle number in CV
float         Scanrate               =  0;                                          //  0x15  = assotiated byte for potential sweep rate dE/dT in mV
float         Conditime              =  0;                                          //  0x16  = assotiated byte for conditioning time (time at E_in before CV start)
//==============================================================================================================================================================
//==============================================================================================================================================================
//      These are internal variables for the CV
//==============================================================================================================================================================
//==============================================================================================================================================================
int           IDX_in                 = 0;                                           //  (zero as initial default) initial index  for pot. on 11-bit scale
int           IDX_first_Vertex       = 0;                                           //  (zero as initial default) 1st-vertex idx for pot. on 11-bit scale
int           IDX_second_Vertex      = 0;                                           //  (zero as initial default) 2nd-vertex idx for pot. on 11-bit scale
int           IDX_final              = 0;                                           //  (zero as initial default) final idx      for pot. on 11-bit scale
float         BitPoints              = 0;                                           //  (zero as initial default) number of bit-points of the first up-ramp, required to define delay-time
float         PotWindow              = 0;                                           //  (zero as initial default) potential-window of the  first up-ramp, required to define delay-time
float         DELAYTIME              = 0;                                           //  (zero as initial default) Time at each potential of the ramp according to sweep-rate
float         DelayReducer           = 7575;                                        //  empirically found value to reduce the delay time to achieve precise timig for scanrates
float         ReducedDelay           = 0;                                           //  This is the variable which will be the actual delay later on. (So DELAYTIME-DelayReducer)
float         ElTime_S               = 0;                                           //  Elapsed time Start (start at beginning of experiment)
float         ElTime_E               = 0;                                           //  Elapsed time End   (maesure after writing the DAC at each time)
float         Inner_t_S              = 0;                                           //  time of the inner data step - start
float         Inner_t_E              = 0;                                           //  time of the inner data step - end
//==============================================================================================================================================================
//==============================================================================================================================================================
//      Here, the variables for the INPUTS of the Chronoamperometry are initialized
//==============================================================================================================================================================
//==============================================================================================================================================================
float         E_CA_1                 =  0;                                          //  0x17  = assotiated byte for first potential of CA
float         E_CA_2                 =  0;                                          //  0x18  = assotiated byte for second potential of CA
float         E_CA_3                 =  0;                                          //  0x19  = assotiated byte for third potential of CA
float         E_CA_4                 =  0;                                          //  0x20  = assotiated byte for fourth potential of CA
float         E_CA_5                 =  0;                                          //  0x21  = assotiated byte for fifth potential of CA
float         time_1                 =  0;                                          //  0x22  = assotiated byte  - time of the first step
float         time_2                 =  0;                                          //  0x23  = assotiated byte  - time of the first step
float         time_3                 =  0;                                          //  0x24  = assotiated byte  - time of the first step
float         time_4                 =  0;                                          //  0x25  = assotiated byte  - time of the first step
float         time_5                 =  0;                                          //  0x26  = assotiated byte  - time of the first step
int           CA_Rep                 =  0;                                          //  0x27  = assotiated byte for repetitions of CA cycles
float         El_CA_Time_S           =  0;                                          //  Elapsed time for CA Start
float         El_CA_Time_Aux1        =  0;                                          //  Elapsed time for CA - auxiliary
float         El_CA_Time_Aux2        =  0;                                          //  Elapsed time for CA - auxiliary
float         El_CA_Time_E           =  0;                                          //  Elapsed time for CA End
//==============================================================================================================================================================
//==============================================================================================================================================================
//      These are internal variables for the Chronoamperometry
//==============================================================================================================================================================
//==============================================================================================================================================================
int           IDX_CA_1               =  0;                                          //  Index for the 1st potential setp of the CA - will be defined later
int           IDX_CA_2               =  0;                                          //  Index for the 2nd potential setp of the CA - will be defined later
int           IDX_CA_3               =  0;                                          //  Index for the 3rd potential setp of the CA - will be defined later
int           IDX_CA_4               =  0;                                          //  Index for the 4th potential setp of the CA - will be defined later
int           IDX_CA_5               =  0;                                          //  Index for the 5th potential setp of the CA - will be defined later
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Inherent stuff to check if the device is talking to the PC properly
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int           Intrinsic_Offset_I     =  0;                                          //  Find out this value by calibration of the device
int           Intrinsic_Offset_E     =  0;                                          //  Find out this value by calibration of the device
const int     BUFFER_SIZE            =  8;                                           //  integer for size of array-like char for read/send bytes
char          buf[BUFFER_SIZE];                                                      //  array-like char for storing read/send bytes
//==============================================================================================================================================================
//==============================================================================================================================================================
//    This function sets up the communication with the Arduino and declares in/output pins
//==============================================================================================================================================================
//==============================================================================================================================================================
void setup() {                                                                      //  setup-communication between Arduino and PC
    Serial.begin(115200);                                                           //  define baudrate for serial communication
    TCCR1B = TCCR1B & B11111000 | B00000001;                                        //  set internal clock divisor to 1 for PWM frequency of 31372.55 Hz
    Wire.begin();                                                                   //  Begin the wire-clock for I2C communication
    Wire.setClock(800000);                                                          //  Set clock-speed of the wire
    MCP.begin();                                                                    //  Begin communiction with DAC
    MCP.setValue(0);                                                                //  Wirte o V at the DAC output
    ADS.begin();                                                                    //  Begin communication witth the ADS1115
    ADS.setGain(1);                                                                 //  Choose the gain of the PGA of the ADS1115 (to be 1) pm 4096 mV
    ADS.setDataRate(7);                                                             //  Set the ADS1115 to the highest sampling speed mode
    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Assign the output/input pins
    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    pinMode(2,OUTPUT);                                                              //  Relay     --> cell on
    pinMode(5,OUTPUT);                                                              //  Green LED --> cell on
    pinMode(A0,INPUT);                                                              //  Additional_input (not used further here)
    pinMode(A1,INPUT);                                                              //  Additional_input (not used further here)
    pinMode(A2,INPUT);                                                              //  Additional_input (not used further here)
    pinMode(A3,INPUT);
 //   Serial.println("alive!");

 }                                                             //  Additional_input (not used further here)
//==============================================================================================================================================================
//==============================================================================================================================================================
//    These three functions are used for controlling the cell on/off LED
//==============================================================================================================================================================
//==============================================================================================================================================================
void LED_On(){
    digitalWrite(5, HIGH);
    delay(500);}
void LED_Off(){
    digitalWrite(5, LOW);
    delay(500);}
//==============================================================================================================================================================
//==============================================================================================================================================================
// This function is used to switch on the cell.
//==============================================================================================================================================================
//==============================================================================================================================================================
void CellOn(){
    digitalWrite(2, HIGH);            // switch on the  Cell-on LED
    digitalWrite(5, 255);}            // switch on the  Cell-on Relay

//==============================================================================================================================================================
//==============================================================================================================================================================
// This function is used to switch off the cell.
//==============================================================================================================================================================
//==============================================================================================================================================================
void CellOff(){
    digitalWrite(2, LOW);            // switch on the  Cell-on LED
    digitalWrite(5, 0);              // switch on the  Cell-on Relay
    //-------------------------------------------------------
    // blink three times to state that measurement is done
    //-------------------------------------------------------
    for(int Blinker = 0; Blinker <= 2; Blinker++){
          digitalWrite(5, HIGH);
          delay(200);
          digitalWrite(5, LOW);
          delay(200);}}
//==============================================================================================================================================================
//==============================================================================================================================================================
// This function is used to write the serial data from the potentiostat for CV and send it to the PC. Should be self-explaning :)
//==============================================================================================================================================================
//==============================================================================================================================================================
void WriteSerialOutput(int rampindex, float times, int Read_Ebit, int Read_Ibit, int CycNo) {
    Serial.print(rampindex);
    Serial.print("\t");
    Serial.print(times);
    Serial.print("\t");
    Serial.print(Read_Ebit+Intrinsic_Offset_E);
    Serial.print("\t");
    Serial.print(Read_Ibit+Intrinsic_Offset_I);
    Serial.print("\t");
    Serial.println(CycNo);}      // the last has to be a println, to get a return and linebreak. Otherwise, the data will not be real line by line in python.
//==============================================================================================================================================================
//==============================================================================================================================================================
// This function is used to write the serial data from the potentiostat for Chronoamperometry and send it to the PC. Should be self-explaning :)
//==============================================================================================================================================================
//==============================================================================================================================================================
void WriteSerialOutput_CA(int LoopNo, int SubNo, float times, int EBit, int IBit) {
    Serial.print(LoopNo);
    Serial.print("\t");
    Serial.print(SubNo);
    Serial.print("\t");
    Serial.print(times);
    Serial.print("\t");
    Serial.print(EBit+Intrinsic_Offset_E);
    Serial.print("\t");
    Serial.println(IBit+Intrinsic_Offset_I);}      // the last has to be a println, to get a return and linebreak. Otherwise, the data will not be real line by line in python.

//==============================================================================================================================================================
//==============================================================================================================================================================
// These two functions specify the ramp-function, whether it is ascending or decreasing.
//==============================================================================================================================================================
//==============================================================================================================================================================
void UpRamp(int FromIDX, int ToIDX, int CycNo){                                           // create function for positive going sweep
     for(int RampIDX = FromIDX; RampIDX >= ToIDX ; RampIDX--){                            // for loop of the negative going sweep
          Inner_t_S = micros();                                                           // start counting the inner time loop for each potential step
          while(Inner_t_E-Inner_t_S < ReducedDelay){                                      // as long as there is enaugh time for collecting a datapoint and writing the output, do it
              MCP.setValue(RampIDX);                                                      // write to the DAC
              int16_t val_3 = ADS.readADC(3);                                             // read the bit-index of the ADS1115 at A3
              int16_t val_1 = ADS.readADC(1);                                             // read the bit-index of the ADS1115 at A1
              ElTime_E      = micros();                                                   // catch time directly after writing the DAC - this is the time to be written later on
              WriteSerialOutput(RampIDX, (ElTime_E-ElTime_S), val_3, val_1, CycNo);       // writing the outputs with the function designed for that purpose
              Inner_t_E = micros();}                                                      // catch time inside of the while loop to decide if another conversion is possible
          Inner_t_E = micros();                                                           // catch time immediately after the while-loop is violated and -->
          delayMicroseconds(DELAYTIME - (Inner_t_E-Inner_t_S));                           // if no conversion in the step is possible anymore, waint until we reach the delaytime
          }
}
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void DownRamp(int FromIDX, int ToIDX, int CycNo){                                         // create function for positive going sweep
    for(int RampIDX = FromIDX; RampIDX <= ToIDX; RampIDX++){                              // for loop of the negative going sweep
        Inner_t_S = micros();                                                             // start counting the inner time loop for each potential step
        while(Inner_t_E-Inner_t_S < ReducedDelay){                                        // as long as there is enaugh time for collecting a datapoint and writing the output, do it
            MCP.setValue(RampIDX);                                                        // write to the DAC
            int16_t val_3 = ADS.readADC(3);                                               // read the bit-index of the ADS1115 at A3
            int16_t val_1 = ADS.readADC(1);                                               // read the bit-index of the ADS1115 at A1
            ElTime_E      = micros();                                                     // catch time directly after writing the DAC - this is the time to be written later on
            WriteSerialOutput(RampIDX, (ElTime_E-ElTime_S), val_3, val_1, CycNo);         // writing the outputs with the function designed for that purpose
            Inner_t_E = micros();}                                                        // catch time inside of the while loop to decide if another conversion is possible
        Inner_t_E = micros();                                                             // catch time immediately after the while-loop is violated and -->
        delayMicroseconds(DELAYTIME - (Inner_t_E-Inner_t_S));                             // if no conversion in the step is possible anymore, waint until we reach the delaytime
    }
}
//########################################################################################
//  Create the same ramps - just in slow for no data overflow at slow measurements
//########################################################################################
void UpRamp_Slow(int FromIDX, int ToIDX, int CycNo){                                      // create function for positive going sweep
     for(int RampIDX = FromIDX; RampIDX >= ToIDX ; RampIDX--){                            // for loop of the negative going sweep
          Inner_t_S = micros();                                                           // start counting the inner time loop for each potential step
          Inner_t_E = micros();                                                           // start counting the inner time loop for each potential step
          //while(Inner_t_E-Inner_t_S < ReducedDelay-20000){                              // as long as there is enaugh time for collecting a datapoint and writing the output, do it
          while(DELAYTIME - (Inner_t_E-Inner_t_S) > DelayReducer+20000){                  // as long as there is enaugh time for collecting a datapoint and writing the output, do it
              MCP.setValue(RampIDX);                                                      // write to the DAC
              int16_t val_3 = ADS.readADC(3);                                             // read the bit-index of the ADS1115 at A3
              int16_t val_1 = ADS.readADC(1);                                             // read the bit-index of the ADS1115 at A1
              ElTime_E      = micros();                                                   // catch time directly after writing the DAC - this is the time to be written later on
              WriteSerialOutput(RampIDX, (ElTime_E-ElTime_S), val_3, val_1, CycNo);       // writing the outputs with the function designed for that purpose
              delay(20);                                                                  // IN THE SLOW RAMP, DELAY BY 20 milliseconds
              Inner_t_E = micros();}                                                      // catch time inside of the while loop to decide if another conversion is possible
          Inner_t_E = micros();                                                           // catch time immediately after the while-loop is violated and -->
          while((Inner_t_E-Inner_t_S) < DELAYTIME){                                       // Take another while loop in steps of 2 microseconds
              delayMicroseconds(2);                                                       // thus, wait 2 microseconds. This has to be done like this, since the usual delay would be a number,
              Inner_t_E = micros(); }                                                     // too large for the delay. If delaytime is surpassed, proceed with next ramp index
          }
}
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void DownRamp_Slow(int FromIDX, int ToIDX, int CycNo){                                    // create function for positive going sweep
    for(int RampIDX = FromIDX; RampIDX <= ToIDX; RampIDX++){                              // for loop of the negative going sweep
        Inner_t_S = micros();                                                             // start counting the inner time loop for each potential step
        Inner_t_E = micros();                                                             // start counting the inner time loop for each potential step
        while(Inner_t_E-Inner_t_S < ReducedDelay-20000){                                  // as long as there is enaugh time for collecting a datapoint and writing the output, do it
            MCP.setValue(RampIDX);                                                        // write to the DAC
            int16_t val_3 = ADS.readADC(3);                                               // read the bit-index of the ADS1115 at A3
            int16_t val_1 = ADS.readADC(1);                                               // read the bit-index of the ADS1115 at A1
            ElTime_E      = micros();                                                     // catch time directly after writing the DAC - this is the time to be written later on
            WriteSerialOutput(RampIDX, (ElTime_E-ElTime_S), val_3, val_1, CycNo);         // writing the outputs with the function designed for that purpose
            delay(20);                                                                    // IN THE SLOW RAMP, DELAY BY 20 milliseconds
            Inner_t_E = micros();}                                                        // catch time inside of the while loop to decide if another conversion is possible
        Inner_t_E = micros();                                                             // catch time immediately after the while-loop is violated and -->
        while((Inner_t_E-Inner_t_S) < DELAYTIME){                                         // Bitten lassen Sie mich wissen ob sich dies Ihrerseits
              delayMicroseconds(2);                                                       // thus, wait 2 microseconds. This has to be done like this, since the usual delay would be a number,
              Inner_t_E = micros(); }                                                     // too large for the delay. If delaytime is surpassed, proceed with next ramp index
    }
}
//==============================================================================================================================================================
//==============================================================================================================================================================
// This function is called, to run the CV. Basically, it calls the previously defined functions
//==============================================================================================================================================================
//==============================================================================================================================================================
void RunCV(){
    //----------------------------------------------------------------------------------------------------------------------------------
    // Set the initial-bits and define delays etc.
    //----------------------------------------------------------------------------------------------------------------------------------
    IDX_in             = -floor(4095*(E_in-3.3)/6.6);                       // initial index  for pot. on 12-bit scale
    IDX_first_Vertex   = -floor(4095*(E_first_vertex-3.3)/6.6);             // 1st-vertex idx for pot. on 12-bit scale
    IDX_second_Vertex  = -floor(4095*(E_second_vertex-3.3)/6.6);            // 2nd-vertex idx for pot. on 12-bit scale
    IDX_final          = -floor(4095*(E_final-3.3)/6.6);                    // final idx      for pot. on 12-bit scale
    BitPoints          = float(IDX_in-IDX_first_Vertex);                    // number of bit-points of the first up-ramp, required to define delay-time
    PotWindow          = (E_first_vertex-E_in);                             // potential-window of the  first up-ramp, required to define delay-time
    DELAYTIME          = (PotWindow*1000000000L/((Scanrate)*BitPoints));    // Time (in microseconds) at each potential of the ramp according to sweep-rate
    ReducedDelay       = DELAYTIME - DelayReducer;                          // Define the delay for each data point as difference of the time per step (delaytime) and the time required for conversion

    //###################################
    // CONDITIONING BEFORE THE CV
    //###################################
    MCP.setValue(IDX_in);
    delay(Conditime);
    //###################################
    Serial.println(10101010); // The start-sequence
    //###################################
    ElTime_S  = micros();
    Inner_t_S = micros();
    Inner_t_E = micros();
    //----------------------------------------------------------------------------------------------------------------------------------
    // START THE POSITIVE GOING SWEEP
    //----------------------------------------------------------------------------------------------------------------------------------
    if(IDX_first_Vertex < IDX_in){
        if(Scanrate >= 20){
            UpRamp(IDX_in, IDX_first_Vertex, 1);
            DownRamp(IDX_first_Vertex+1, IDX_second_Vertex, 1);
            if(Cycles > 1){
                for(int Cyclecounter = 1; Cyclecounter < Cycles; Cyclecounter++){
                    UpRamp(IDX_second_Vertex-1, IDX_first_Vertex, Cyclecounter+1);
                    DownRamp(IDX_first_Vertex+1, IDX_second_Vertex, Cyclecounter+1);}
                UpRamp(IDX_second_Vertex-1, IDX_final, Cycles+1);}
            if(Cycles == 1){
                UpRamp(IDX_second_Vertex-1, IDX_final,1);}}
        if(Scanrate < 20){
            UpRamp_Slow(IDX_in, IDX_first_Vertex, 1);
            DownRamp_Slow(IDX_first_Vertex+1, IDX_second_Vertex, 1);
            if(Cycles > 1){
                for(int Cyclecounter = 1; Cyclecounter < Cycles; Cyclecounter++){
                    UpRamp_Slow(IDX_second_Vertex-1, IDX_first_Vertex, Cyclecounter+1);
                    DownRamp_Slow(IDX_first_Vertex+1, IDX_second_Vertex, Cyclecounter+1);}
                UpRamp_Slow(IDX_second_Vertex-1, IDX_final, Cycles+1);}
            if(Cycles == 1){
                UpRamp_Slow(IDX_second_Vertex-1, IDX_final,1);}}
    }
    //----------------------------------------------------------------------------------------------------------------------------------
    // START THE NEGATIVE GOING SWEEP
    //----------------------------------------------------------------------------------------------------------------------------------
    if(IDX_first_Vertex > IDX_in){
        if(Scanrate >= 20){
            DownRamp(IDX_in, IDX_first_Vertex, 1);
            UpRamp(IDX_first_Vertex-1, IDX_second_Vertex, 1);
            if(Cycles > 1){
                for(int Cyclecounter = 1; Cyclecounter < Cycles; Cyclecounter++){
                    DownRamp(IDX_second_Vertex+1, IDX_first_Vertex, Cyclecounter+1);
                    UpRamp(IDX_first_Vertex-1, IDX_second_Vertex, Cyclecounter+1);}
                DownRamp(IDX_second_Vertex+1, IDX_final, Cycles+1);}
            if(Cycles == 1){
                DownRamp(IDX_second_Vertex-1, IDX_final,1);}}
        if(Scanrate < 20){
            DownRamp_Slow(IDX_in, IDX_first_Vertex, 1);
            UpRamp_Slow(IDX_first_Vertex-1, IDX_second_Vertex, 1);
            if(Cycles > 1){
                for(int Cyclecounter = 1; Cyclecounter < Cycles; Cyclecounter++){
                    DownRamp_Slow(IDX_second_Vertex+1, IDX_first_Vertex, Cyclecounter+1);
                    UpRamp_Slow(IDX_first_Vertex-1, IDX_second_Vertex, Cyclecounter+1);}
                DownRamp_Slow(IDX_second_Vertex+1, IDX_final, Cycles+1);}
            if(Cycles == 1){
                DownRamp_Slow(IDX_second_Vertex-1, IDX_final,1);}}
    }
    //###################################
    Serial.println(999999); // The stop-sequence
    //###################################
}

//==============================================================================================================================================================
//==============================================================================================================================================================
// This function defines the sub-step for a chronoamperometric protocol
//==============================================================================================================================================================
//==============================================================================================================================================================
void Chronoamperometric_Sub_Step(int Loop_Position, int Sub_Loop_Position, int CA_sub_time, int CA_DAC_set){
    MCP.setValue(CA_DAC_set);                                                                                //  set DAC value for the sub-step of the CA of the nth- loop repetition
    while(El_CA_Time_Aux2 - El_CA_Time_Aux1 < CA_sub_time){                                                  //  while loop time is lower than set time, measure and write output
        int16_t val_3    = ADS.readADC(3);                                                                   //  read the bit-index of the ADS1115 at A3
        int16_t val_1    = ADS.readADC(1);                                                                   //  read the bit-index of the ADS1115 at A1
        delay(50);                                                                                           //  delay of 50 ms
        El_CA_Time_E     = millis();                                                                         //  Update end step time
        WriteSerialOutput_CA(Loop_Position, Sub_Loop_Position, (El_CA_Time_E-El_CA_Time_S), val_3, val_1);   //  write outputs
        El_CA_Time_Aux2  = millis();}                                                                        //  update axiliary time2
    El_CA_Time_Aux1 = millis();                                                                              //  if while loop is broken, update axiliary time1
    El_CA_Time_Aux2 = millis();}                                                                             //  if while loop is broken, update axiliary time2

//==============================================================================================================================================================
//==============================================================================================================================================================
// This function defines how to run the CA experiment - i.e. how many repetitions and so on
//==============================================================================================================================================================
//==============================================================================================================================================================
void RunCA(){
    //----------------------------------------------------------------------------------------------------------------------------------
    // Set the initial-bits and define delays etc.
    //----------------------------------------------------------------------------------------------------------------------------------
    IDX_CA_1             = -floor(4095*(E_CA_1-3.3)/6.6);                   // initial index  for pot. on 12-bit scale
    IDX_CA_2             = -floor(4095*(E_CA_2-3.3)/6.6);                   // initial index  for pot. on 12-bit scale
    IDX_CA_3             = -floor(4095*(E_CA_3-3.3)/6.6);                   // initial index  for pot. on 12-bit scale
    IDX_CA_4             = -floor(4095*(E_CA_4-3.3)/6.6);                   // initial index  for pot. on 12-bit scale
    IDX_CA_5             = -floor(4095*(E_CA_5-3.3)/6.6);                   // initial index  for pot. on 12-bit scale
    //###################################
    Serial.println(10101010); // The start-sequence
    //###################################
    El_CA_Time_S    = millis();
    El_CA_Time_Aux1 = millis();
    El_CA_Time_Aux2 = millis();
    El_CA_Time_E    = millis();
    for(int CA_Num = 1; CA_Num <= int(CA_Rep); CA_Num++){
        Chronoamperometric_Sub_Step(CA_Num, 1, time_1, IDX_CA_1);
        Chronoamperometric_Sub_Step(CA_Num, 2, time_2, IDX_CA_2);
        Chronoamperometric_Sub_Step(CA_Num, 3, time_3, IDX_CA_3);
        Chronoamperometric_Sub_Step(CA_Num, 4, time_4, IDX_CA_4);
        Chronoamperometric_Sub_Step(CA_Num, 5, time_5, IDX_CA_5);}
    //###################################
    Serial.println(999999);} // The stop-sequence
    //###################################



//==============================================================================================================================================================
//==============================================================================================================================================================
// Now, all required functions for the Potentiostat are defined
//==============================================================================================================================================================
//==============================================================================================================================================================











//#######################################################################################################################################
//#######################################################################################################################################
//#######################################################################################################################################
//#######################################################################################################################################
//                                     Start the main loop (executing the code) on the Arduino
//#######################################################################################################################################
//#######################################################################################################################################
//#######################################################################################################################################
//#######################################################################################################################################
void loop(){
    while(!Serial.available());                        //  !Serial.available is like Serial.available, but waits until a serial input is provided
    Serial.readBytes(buf, BUFFER_SIZE);                //  read the input from the python script, which is send in line 100 via  arduino.write(SEND_BYTES)
    char fBuffer[] = {buf[2],buf[3],buf[4],buf[5]};    //  initialize another char, called fBuffer and write the second, third, fourth and fifth byte
                                                       //  (so the float which is transmitted from python). Note, the zeroth byte is the check_byte and the
                                                       //  first byte is the float_byte (so both are not the information of the float number transmitted)
    float x = *(float *)&fBuffer;                      //  convert byte buffer to float buffer
    //======================================================================================================================
    //======================================================================================================================
    switch(buf[0]){                                    //  Like if statements, switch case controls the flow of programs
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //   CHECKING CASE
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x44: // CHECK                              //  This is analogue to if buf[0] (so the first byte) is 0x44, the check byte, do the following
            Serial.println(buf);                         //  send back, what was obtained, without converting it to a float
            switch(buf[1]){                              //  if the buf[0] was the check byte, check the next, so buf[1]
                case 0x66:                               //  see, if the buf[1] contains a float byte
                    Serial.println(x);                   //  if so, print the fBuffer, which is the previously read four byte float (so float32)
                    break;}                              //  break the buf[1]
            break;                                       //  break the buf[0] and start again at the top of void loop()
        //############################################################################################################################################
        //############################################################################################################################################
        //    CYCLIC VOLTAMMETRY
        //############################################################################################################################################
        //############################################################################################################################################
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    Potentiostat_Code, First, set the bytes for the CV parameters
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x11:     // SET CV params                  //  In case it is 0x11, so the set byte, enter this loop
            switch(buf[1]){                              //  If statement for the second (first) byte
                case 0x10:                               //  In case of  0x11_0x10_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_in                   = x;          //  Take the x-value float as Ein and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x11:                               //  In case of  0x11_0x11_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_first_vertex         = x;          //  Take the x-value float as E_first_vertex in and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x12:                               //  Take the x-value float as Ein and -->
                    E_second_vertex        = x;          //  In case of  0x11_0x12_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x13:                               //  Take the x-value float as E_final and -->
                    E_final                = x;          //  In case of  0x11_0x13_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x14:                               //  Take the x-value float, convert it to an integer and assign it as Cycles and -->
                    Cycles                 = int(x);     //  In case of  0x11_0x14_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x15:                               //  Take the x-value float as Scanrate and -->
                    Scanrate               = x;          //  In case of  0x11_0x15_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x16:                               //  Take the x-value float as Conditime and -->
                    Conditime              = 1000*x;     //  In case of  0x11_0x16_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    break;}                              //  Break the buf[1] Leave back to the start of the void to receive the next value/command
            break;
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    CV_Code, Return/print the parameters, which were set
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x22:                                        //  READ the inputs which were send from PC and send them back TO PC
            Serial.println(E_in);                         //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_first_vertex);               //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_second_vertex);              //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_final);                      //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(float(Cycles));                //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(Scanrate);                     //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(0.001*Conditime);              //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            break;                                        //  Break the buf[0] - case
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    Potentiostat_Code, Run the measurement
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x33:                                                        // DO something in buf[0]
            //----------------------------------------------------------------------------------------------------------------------------------------
            // Running a CV
            //----------------------------------------------------------------------------------------------------------------------------------------
            switch(buf[1]){
                case 0x01:                                //  CV
                    CellOn();                             //  call the Cell-on function to switch on the relay AND the cell
                    RunCV();                              //  call the CV function to run the CV
                    CellOff();                            //  call the Cell-off function to switch off the relay AND the cell
                    delay(1000);                          //  delay to settle everything
                    break;}                               //  break the buf[1] case
            break;                                        //  Break the buf[0] - case once CV is done



        //############################################################################################################################################
        //############################################################################################################################################
        //    CHRONOAMPEROMETRY
        //############################################################################################################################################
        //############################################################################################################################################
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    Chronoamperometry_Code, First, set the bytes for the CA parameters if buf[0] equals 0x12, chronoamperometry setting is addressed
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x12:                                       //  DO something if buf[0]=0x12
            switch(buf[1]){                              //  If statement for the second (first) byte
                case 0x17:                               //  In case of  0x10_0x17_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_CA_1                   = x;        //  Take the x-value float as E_CA_1 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x18:                               //  In case of  0x10_0x18_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_CA_2                   = x;        //  Take the x-value float as E_CA_2 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x19:                               //  In case of  0x10_0x19_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_CA_3                   = x;        //  Take the x-value float as E_CA_3 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x20:                               //  In case of  0x10_0x20_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_CA_4                   = x;        //  Take the x-value float as E_CA_4 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x21:                               //  In case of  0x10_0x21_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    E_CA_5                   = x;        //  Take the x-value float as E_CA_5 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x22:                               //  In case of  0x10_0x22_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    time_1                   = x;        //  Take the x-value float as time_1 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x23:                               //  In case of  0x10_0x23_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    time_2                   = x;        //  Take the x-value float as time_2 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x24:                               //  In case of  0x10_0x24_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    time_3                   = x;        //  Take the x-value float as time_3 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x25:                               //  In case of  0x10_0x25_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    time_4                   = x;        //  Take the x-value float as time_4 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x26:                               //  In case of  0x10_0x26_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    time_5                   = x;        //  Take the x-value float as time_5 and -->
                    break;                               //  Break the buf[1] Leave back to the start of the void to receive the next value/command
                case 0x27:                               //  In case of  0x10_0x27_0x--_0x--_0x--_0x--_ , where the 0x-- specify the float
                    CA_Rep                   = int(x);   //  Take the x-value float as CA_Rep and transfer it to an integer and -->
                    break;}                              //  Break the buf[1] Leave back to the start of the void to receive the next value/command
            break;
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    Chronoamperometry_Code, First, set the bytes for the CA parameters if buf[0] equals 0x13, chronoamperometry returning is addressed
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x13:                                         //  DO something if buf[0]=0x13
            Serial.println(E_CA_1);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_CA_2);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_CA_3);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_CA_4);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(E_CA_5);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(time_1);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(time_2);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(time_3);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(time_4);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(time_5);                        //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            Serial.println(float(CA_Rep));                 //  If any reading bit for the inputs is trasmitted, print all transmitted parameters
            break;                                         //  Break the buf[0] - case
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //    Chronoamperometry_Code, Run the measurement if buf[0] equals 0x14
        //--------------------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------------------------------
        case 0x14:                                         //  DO something in buf[0]
            CellOn();
            RunCA();
            CellOff();
            delay(1000);
            break;}                                         //  Break the buf[0] - case

        }

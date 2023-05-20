////////////////////////////////////////////////////////////////////////////////
//State Machine V.1.10
//
//Maquina de estados para Open Space
//Programa a usar con arduino pro mini 5V/16MHz
//
//Fecha: 22/01/21
//Edito: JE.
//
//Notas: - Probada con STM32. Queda enviando datos validados, faltan codigos y
//         formato final de datos, junto con comandos.
//
//Siguientes pasos: -
//
//V.1.00: -Simple maquina de estados -> Cambio de estado muy simple funcionando
//V.1.01: -Maquina de estado con la lectura de las curvas de transistores.
//V.1.02: -Se agrega el CRC y se comenta mejor
//V.1.03: -Se le agrega el CRC a los datos enviados
//V.1.04: -Se agregar el control de la maquina de estados via mensajes 
//V.1.05: -Faltan agregar maquinas de estados pero se guardan datos en SD
//         SD a traves del STM y se leen y analizan los datos con MATLAB.
//         Se agrega el envio del tiempo. -> FALLA GRAVE AL ENVIAR MAS DE 
//         4 PUNTOS. EL STRING ES MUY LARGO Y EL ARDUINO ENLOQUECE.
//V.1.06: -Se intenta solucionar el problema grave de enviar varios puntos
//V.1.07: -Problema de V.1.06 solucionado. Se emprolija para primer testeo 
//         general el domingo 19/9/21
//V.1.08: -Se agregan las funciones de verificacion I2C y temp interna. 
//V.1.09: -Se emprolijan los mensajes de salida y se agrega la entrada 
//         de configuraciones
//V.1.10  -Se agregan los dos sensores de temp 
///////////////////////////////////////////////////////////////////////////////


/////////// Definitions///////////

#define Samples 10
#define Points 9
#define DAC_Setting_Time 200 //Express in ms
#define ONE_WIRE_MIDDLE 8
#define ONE_WIRE_NADIR 10
#define Resend_times 4
#define Resend_times_data 4
#define Time_out 1000
#define DAC_Max_Voltage 3000

/*     -START Debug selection-      */
///////////////////////// Debug selection ///////////////////////////////
//                                                                     //
//Select the Debug uncommenting the first line.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

//General Debug
//#define Debug
#ifdef Debug
 #define Debug(x)  Serial.print (x)
 #define Debugln(x)  Serial.println (x)
#else
 #define Debug(x)
 #define Debugln(x)
#endif

//General Debug
//#define Debug_temp 
#ifdef Debug_temp 
 #define DebugT(x)  Serial.print (x)
 #define DebugTln(x)  Serial.println (x)
#else
 #define DebugT(x)
 #define DebugTln(x)
#endif

//ADC lecture Debug
//#define Debug_ADC
#ifdef Debug_ADC
  #define Debug_adc(x)  Serial.print (x)
  #define Debugln_adc(x)  Serial.println (x)
#else
  #define Debug_adc(x)
  #define Debugln_adc(x)
#endif


//Statistics Debug
//#define Debug_Statitics
#ifdef Debug_Statitics
  #define Debug_Stats(x)  Serial.print (x)
  #define Debugln_Stats(x)  Serial.println (x)
#else
  #define Debug_Stats(x)
  #define Debugln_Stats(x)
#endif

//Debug Send Protocol
//#define DebugSP
#ifdef DebugSP
 #define DebugSP(x)  Serial.print (x)
 #define DebugSPln(x)  Serial.println (x)
#else
 #define DebugSP(x)
 #define DebugSPln(x)
#endif
//Debug Error detecting
//#define DebugErrorDet
#ifdef DebugErrorDet
 #define DebugErrorDet(x)  Serial.print (x)
 #define DebugErrorDetln(x)  Serial.println (x)
 #define DebugErrorDetHex(x) Serial.print(x,HEX)
#else
 #define DebugErrorDet(x)
 #define DebugErrorDetln(x)
 #define DebugErrorDetHex(x)
#endif

/*     -END Debug selection-      */

/*     -START Setting parameters-      */
///////////////////////// Setting parameters ////////////////////////////
//                                                                     //
//General parameters initialization and setting.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

///////////Libraries///////////

#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h> // MCP4725 library from adafruit
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* strtol */
#include <avr/wdt.h>    /*Watchdog*/
#include "CRC8.h"
#include "CRC.h"
//////////////////////////////

///////////Initializations///////////
//CRC
CRC8 crc;
// Temperature sensor
OneWire oneWire(ONE_WIRE_MIDDLE);
OneWire oneWire_Nadir(ONE_WIRE_NADIR);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DallasTemperature sensors_N(&oneWire_Nadir);
//DAC inicialization
Adafruit_MCP4725 MCP4725; 
//ADC inicialization
Adafruit_ADS1115 ads1;  /* Use this for the 16-bit version */
Adafruit_ADS1115 ads2;  /* Use this for the 16-bit version */
Adafruit_ADS1115 ads3;  /* Use this for the 16-bit version */
Adafruit_ADS1115 ads4;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
///////////////////////////////

///////////Declaracion de estados///////////
enum State_enum {START_DATASET, SEND_DATA, WAITING, CHECK_STATUS, ERROR_HANDLING, TEST};
uint8_t state=WAITING;
////////////////////////////////////////////

///////////Variables globales//////////////
char inputString[100];        // Input buffer
bool stringComplete = false;  // Message received flag
char Message[100];        // Input buffer
uint16_t Buffer[14][Points];
static bool ledState= false;
volatile bool SendOk = false; 
volatile int ErrorCode = 0; 
///////////////////////////////////////////

///////////Declaracion de funciones//////////////
int Read_adc_1(short Counter_Punto,short ADC_name);
void Mosfet_Curve(int Counter_adc);
void DAC(uint32_t MCP4725_value);
int Send(int adc);
bool Send_Command(String Packet);
int Add_CRC(String Packet);
void software_reboot(){asm volatile("jmp 0");}
int control_temp_interno(void);
int I2C_Verification(void);
///////////////////////////////////////////

/*     -END Setting parameters-      */


/*     -START Setup-      */
/////////////////////////////// Setup ///////////////////////////////////
//                                                                     //
//Device initialization                                                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////



void setup() {
  // Serial initialization  
  Serial.begin(9600);
  
  // I/O ports
  pinMode(LED_BUILTIN, OUTPUT);
  
  ///Apertura llaves de habilitacion
  pinMode(4,OUTPUT);//KEY FOR VDD
  digitalWrite(4,HIGH);
  pinMode(7,OUTPUT);//KEY FOR DAC
  digitalWrite(7,HIGH);  
  
  pinMode(5,OUTPUT);//KEY FOR VDD
  digitalWrite(5,HIGH);
  pinMode(3,OUTPUT);//KEY FOR DAC
  digitalWrite(3,HIGH);  
  
  pinMode(2,OUTPUT);//KEY FOR VDD
  digitalWrite(2,HIGH);
  pinMode(6,OUTPUT);//KEY FOR DAC
  digitalWrite(6,HIGH); 
  
  //ADCs
  ads1.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads2.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads3.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads4.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads1.begin(0x49);
  ads2.begin(0x48);
  ads3.begin(0x4B);
  ads4.begin(0x4A);
  
  //DAC
  MCP4725.begin(0x64);
  
  //Temp. Sensor
  sensors.begin();  //Temp Sensor
  sensors_N.begin();  //Temp Sensor

  //CRC Setting
  crc.setPolynome(0x97);
  
  //Delay de seguridad
  delay(1000);
}
/*     -END Setup-      */


/*     -START main-      */
///////////////////////////////// Main //////////////////////////////////
//                                                                     //
//Main. Here is contained the state machine and the call to the        //
//functions.                                                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

void loop() {
int Message_Counter=0;
static int Counter_adc=1;
String DATA="";
///////// START MESSAGES INPUT ////////////

  if (stringComplete) {    
        Serial.println();
        Serial.println(inputString);  
         Message_Counter=0;                

       //START MESSAGE VALIDATION
       if(CRC_Validation()==1){
        
            while((inputString[Message_Counter]!='\r') & (Message_Counter < 98)){  //Cambiar '1' por '\r'
             
              Message[Message_Counter]=inputString[Message_Counter];            
              Serial.println(Message[Message_Counter]);
              Message_Counter++;
            }     
        Serial.println("\n\nMessage validated!!\n\n");         
          
         }else{
          
          Serial.println("\n\nMessage with errors!!\n\n");
         }
      
       
       //for (short i=0;i<6;i++){
       //  Serial.print("\nMessage: ");
       //  Serial.print(Message[i]); 
       //  Serial.print("- i : ");
       // Serial.print(i);   
       // }
        
        
        //////////////////////////
                
        //Restart input string    
        for (short i=0;i<98;i++){
          inputString[i]=0;
        }
        
        stringComplete=false;
        /////////////////////
  }  
///////// END MESSAGES INPUT ////////////
 

//STATE MACHINE
 
  switch(state){
    
    case WAITING:
    state=CHECK_STATUS;
    if(strcmp(Message, "Start")==0){
        state=START_DATASET; 
      }else if(strcmp(Message, "Status")==0){
        state=CHECK_STATUS;
      }
    break;
    
    case CHECK_STATUS:
   // if(Send_Command((String)"Status Report")){
    //  if(Send_Command((String)"-I2C Ver.:")){
          DATA.concat(I2C_Verification());
          Serial.print(Add_CRC(DATA));
          DATA="";
         // if(Send_Command((String)"-Internal Temp.:")){            
            DATA.concat(control_temp_interno());
            Serial.print(Add_CRC(DATA));
            DATA="";
          //  if(Send_Command((String)"-Board Temp.:")){
              sensors.requestTemperatures(); // Send the command to get temperatures.         
              DATA.concat(sensors.getTempCByIndex(0)*1000);
              Serial.print("TEMP:");
              Serial.print(Add_CRC(DATA));
              DATA="";
              sensors_N.requestTemperatures(); // Send the command to get temperatures.         
              DATA.concat(sensors_N.getTempCByIndex(0)*1000);
              Serial.print("TEMP_N:");
              Serial.print(Add_CRC(DATA));
              DATA="";
           /* 
            }else{
              state=ERROR_HANDLING;       
              ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
            }
          }else{
            state=ERROR_HANDLING;       
            ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
          } 
        }else{
          state=ERROR_HANDLING;       
          ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.    
        }
    }else{
      state=ERROR_HANDLING;       
      ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
    }
      */
    Send_Command((String)"END Status");
    break;
    
 
    
    case START_DATASET:
      if(Counter_adc<4){
       if(Send_Command((String)"STARTING")==0){          
          SendOk=false;
          Mosfet_Curve(Counter_adc);       
          Serial.println("END");      
          state=SEND_DATA;         
        }else{
          state=ERROR_HANDLING;       
          ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
        }            
      }else{
        Counter_adc=1;
        state=WAITING;
      }
      
    break;
    
    case SEND_DATA:
    digitalWrite(LED_BUILTIN, HIGH);         
    if(Send_Command((String)"SENDING DATA")==0){
        SendOk=false;
    
        if(Send(Counter_adc)==1){ //Look if there has been an error
          state=ERROR_HANDLING;       
          ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
        }else{
          Counter_adc++; 
        state=START_DATASET;
        }        
    }else{
      state=ERROR_HANDLING;       
      ErrorCode=1; //ERROR IN COMMUNICATION. Too many attemps.
    }    
    digitalWrite(LED_BUILTIN, LOW);     
    break;
    
    case ERROR_HANDLING:
    Send_Command((String)"ERROR! Error code:");
    DATA.concat(ErrorCode);
    Serial.println(Add_CRC(DATA));    
    DATA="";
    Send_Command((String)"ERROR! Restarting ...");
    delay(5000);
    software_reboot();
    break;
    
    case TEST:
    break;
    
  }

  //Clean Message    
  for (short i=0;i<98;i++){
          Message[i]=0;
        }
  //Serial.print(state);

}
/*     -END Main-      */


/*     -START Serial event-      */
///////////////////////////// Serial Event //////////////////////////////
//                                                                     //
//Arduino serial event. This executes every time the main loops ends.  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

void serialEvent() {
  static int CharCounter=0;
 
    while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    Serial.print(inChar);
    // add it to the inputString:
    inputString [CharCounter]= inChar;
    CharCounter++;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if ((inChar == '\n') or (CharCounter>250)){
      CharCounter=0;  
      stringComplete = true;
      //Serial.print(inputString);
    }
  } 
}
/*     -END Serial Event-      */

/*     -START Mosfet curve reading-      */
/////////////////////// Mosfet curve reading ////////////////////////////
//                                                                     //
//Mosfet Vgs Vs. Id curve calculation. Measures of the 3 transistors   //
//are made. Mean and Std. Dev. are calculated in a number of samples   //
//define by "Samples" Constant.                                        //
/////////////////////////////////////////////////////////////////////////
void Mosfet_Curve(int Counter_adc){
  int c=0;
uint16_t MCP4725_value = 0;
uint16_t Tx_Flag = 0;
String DATA="";
  
    for (int i=0;i<Points;i++){
      //Serial.print(i);
      DAC(i*(DAC_Max_Voltage/Points));
      
      DATA.concat(i);
      Serial.print(DATA);      
      DebugSPln("\n///MOSFET CURVE///");
      DebugSP("DATA String: ");
      DebugSPln(DATA);
      DebugSP("DATA CRC: ");
      DebugSPln(Add_CRC(DATA));
      Serial.write(13);
      Serial.println(Add_CRC(DATA));  
      DATA="";
                
      Read_adc_1(i,Counter_adc);
       // Save Temperature Middle
       sensors.requestTemperatures(); // Send the command to get temperatures.
       float tempC = sensors.getTempCByIndex(0);     
       Buffer[12][i] =((tempC*1000)); 
       DebugT("Temp:");
       DebugTln((int16_t) (tempC*1000));

       // Save Temperature Nadir
       sensors_N.requestTemperatures(); // Send the command to get temperatures.
       tempC = sensors_N.getTempCByIndex(0);
       Buffer[13][i] =((tempC*1000)); 
       DebugT("Temp_N:");
       DebugTln((int16_t) (tempC*1000));
       
    }
    DAC(0);
    //Serial.write(255);
     while (Serial.available() <= 1);
      if(Serial.read()=="\n"){
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);    
      }
      
    
  
  
}
/*     -END Mosfet curve reading-      */

/*     -START Data sending-      */
////////////////////// Data sending ///////////////////////////////////
//                                                                   //
//This function sends data via UART. It sends the Buffer data with   //
//some sync parameters incorporated.                                 // 
//                                                                   //
///////////////////////////////////////////////////////////////////////

int Send(int adc){
int Tries_counter=0;
bool Error_Flag=0;
int Counter_A=0,Counter_B=0,Counter_Send=1;
static int Status=0;
String DATA="";
DebugSPln("\n///SEND DATA///");
  Serial.write(13);
  while (Serial.available() <= 0);
  if(Serial.read()==10){
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);    
  }

  Counter_B=0;

  while ((Counter_B<=13) and (Error_Flag==0)){
    while (Counter_Send<=(Points/3)){      
        //Serial.print(Counter_Send);
        DATA.concat(adc);
        DATA.concat(",");
        //Serial.print(Counter_B+1);
        DATA.concat(Counter_B+1); 
        DATA.concat(","); 
        DATA.concat(Counter_Send); 
        //DATA=concat(char(adc);
          for (Counter_A=((Counter_Send-1)*3);Counter_A<((Counter_Send)*3);Counter_A++){
            
             //Serial.print(Buffer[Counter_B][Counter_A]>>8);
              //DATA.concat(Buffer[Counter_B][Counter_A]>>8);
             //Serial.print(Buffer[Counter_B][Counter_A]);
              DATA.concat(",");
              DATA.concat((uint16_t)Buffer[Counter_B][Counter_A]);
              //DATA.concat((uint16_t)Buffer[Counter_B][Counter_A]*0.0001875); //Select this one for decimal format.
              
          }
        DebugSP("DATA String: ");
        DebugSPln(DATA);
        DebugSP("DATA CRC: ");
        DebugSPln(Add_CRC(DATA));
        Serial.print(DATA);        
        Serial.write(13);
        Serial.println(Add_CRC(DATA));        
        DATA=""; 
    
      while (Serial.available() <= 0);
  
      Status=Serial.read();
       DebugSP("Status: ");
      DebugSPln(Status);
      if(Status==10){
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);    
        Counter_Send++;
        Tries_counter=0;
      }else{
        Tries_counter++;
        
        DebugSP("\nResend counter: ");
        DebugSPln(Tries_counter);
      } 
       
      if(Tries_counter>Resend_times_data){
      Error_Flag=1;
      }
      
    }
    Counter_Send=1;
    Counter_B++;    
  }  
  if (Error_Flag==0){
    return 0;    
  }else{
    DebugSP("\n\n-> COMMUNICATION FAILURE! Too many tries.\n\n");
    return 1;
  }

}
/*     -END Data sending-      */





/*     -START DAC Setting-      */
////////////////////// DAC setting /////////////////////////////////
//                                                                //
// This function returns set the DAC value                        //
//                                                                //
//-Subfunction of "Mosfet curve reading"                          //
////////////////////////////////////////////////////////////////////
void DAC(uint32_t MCP4725_value) {
    
    float MCP4725_expected_output;
    
     // MCP4725_expected_output = (5.0/4096.0) * MCP4725_value;
      MCP4725.setVoltage(MCP4725_value, false);
      
      delay(DAC_Setting_Time);
            
      //Serial.print("MCP4725 Value: ");
      //Serial.print(MCP4725_value);      
      //Serial.print("\tExpected Voltage: ");
      //Serial.println(MCP4725_expected_output,3); 
}
/*     -END DAC Setting-      */


/*     -START ADC Reading-      */
//////////////////////ADC reading//////////////////////////////////////
//                                                                   //
// This function returns Mean and Variance of the ADC lecture under  //
//the number of samples define in "#define Samples X".               //
//                                                                   //
//-Subfunction of "Mosfet curve reading"                             //
///////////////////////////////////////////////////////////////////////

int Read_adc_1(short Counter_Punto,short ADC_name){
//Variables initialization//
static int16_t adc0[Samples], adc1[Samples], adc2[Samples], adc3[Samples], adc4[Samples], adc5[Samples];
static float Mean[6]={0,0,0,0,0,0}, Var[6]={0,0,0,0,0,0};
static int8_t Counter_1=0,Counter_2=0;


  Debug_adc("Start data (Mean): ");
  Debugln_adc(Mean[0]);
  Debug_adc("Start data (Var): ");
  Debugln_adc(Var[0]);
  
  //ADC reading with part selection//
  
  for(Counter_1=0;Counter_1<Samples;Counter_1++){
    //ADC selection
    switch (ADC_name){
      //ADC 1
      case 1:
        adc0[Counter_1] = ads1.readADC_SingleEnded(0);
        adc1[Counter_1] = ads1.readADC_SingleEnded(1);
        adc2[Counter_1] = ads2.readADC_SingleEnded(0);
        adc3[Counter_1] = ads2.readADC_SingleEnded(1);
        adc4[Counter_1] = ads3.readADC_SingleEnded(0);
        adc5[Counter_1] = ads3.readADC_SingleEnded(1);
      break;
      //ADC 2
      case 2:
        adc0[Counter_1] = ads1.readADC_SingleEnded(2);
        adc1[Counter_1] = ads1.readADC_SingleEnded(3);
        adc2[Counter_1] = ads3.readADC_SingleEnded(3);
        adc3[Counter_1] = ads4.readADC_SingleEnded(1);
        adc4[Counter_1] = ads4.readADC_SingleEnded(0);
        adc5[Counter_1] = ads1.readADC_SingleEnded(3);// copiado Vs no se tiene lectura
      break;
      //ADC 3
      case 3:
        adc0[Counter_1] = ads2.readADC_SingleEnded(2);
        adc1[Counter_1] = ads2.readADC_SingleEnded(3);
        adc2[Counter_1] = ads3.readADC_SingleEnded(2);
        adc3[Counter_1] = ads2.readADC_SingleEnded(3);// copiado Vs no se tiene lectura
        adc4[Counter_1] = ads4.readADC_SingleEnded(2);
        adc5[Counter_1] = ads4.readADC_SingleEnded(3);
      break;
      // Default
        adc0[Counter_1] = 0;
        adc1[Counter_1] = 0;
        adc2[Counter_1] = 0;
        adc3[Counter_1] = 0;
        adc4[Counter_1] = 0;
        adc5[Counter_1] = 0;
      break;
    }
  
  Mean[0]+=adc0[Counter_1];
  Mean[1]+=adc1[Counter_1];
  Mean[2]+=adc2[Counter_1];
  Mean[3]+=adc3[Counter_1];
  Mean[4]+=adc4[Counter_1];
  Mean[5]+=adc5[Counter_1];
  
  //////Debug adc lecture////////       
  Debug_adc("///// Transistor number:");
  Debug_adc(ADC_name);
  Debugln_adc(" /////");
  Debug_adc("/// Point number:");
  Debugln_adc(Counter_Punto);
  Debug_adc("adc0:");
  Debug_adc(adc0[Counter_1]*0.0001875);
  Debugln_adc("V");
  Debug_adc("adc1:");
  Debug_adc(adc1[Counter_1]*0.0001875);
  Debugln_adc("V");
  Debug_adc("adc2:");
  Debug_adc(adc2[Counter_1]*0.0001875);
  Debugln_adc("V");
  Debug_adc("adc3:");
  Debug_adc(adc3[Counter_1]*0.0001875);
  Debugln_adc("V");
  Debug_adc("adc4:");
  Debug_adc(adc4[Counter_1]*0.0001875);
  Debugln_adc("V");
  Debug_adc("adc5:");
  Debug_adc(adc5[Counter_1]*0.0001875);
  Debugln_adc("V");
  //////////////////////////////
  
  }
  Mean[0]=(float)Mean[0]/Samples;
  Mean[1]=(float)Mean[1]/Samples;
  Mean[2]=(float)Mean[2]/Samples;
  Mean[3]=(float)Mean[3]/Samples;
  Mean[4]=(float)Mean[4]/Samples;
  Mean[5]=(float)Mean[5]/Samples;
  
 //Debug_adc("Med data (Mean): ");
 //Debugln_adc(Mean[0]);
 //Debug_adc("Med data (Var): ");
 //Debugln_adc(Var[0]);
  
  for(Counter_2=0;Counter_2<Samples;Counter_2++){
     
     Var[0] += (adc0[Counter_2] - Mean[0]) * (adc0[Counter_2] - Mean[0]); 
     Var[1] += (adc1[Counter_2] - Mean[1]) * (adc1[Counter_2] - Mean[1]); 
     Var[2] += (adc2[Counter_2] - Mean[2]) * (adc2[Counter_2] - Mean[2]); 
     Var[3] += (adc3[Counter_2] - Mean[3]) * (adc3[Counter_2] - Mean[3]); 
     Var[4] += (adc4[Counter_2] - Mean[4]) * (adc4[Counter_2] - Mean[4]); 
     Var[5] += (adc5[Counter_2] - Mean[5]) * (adc5[Counter_2] - Mean[5]); 
  
  //Debug_adc("Var calculation data (Var): "); 
  //Debugln_adc(Var[0]);   
  }
  
  Var[0]= sqrt((float)Var[0] / Samples);
  Var[1]= sqrt((float)Var[1] / Samples); 
  Var[2]= sqrt((float)Var[2] / Samples); 
  Var[3]= sqrt((float)Var[3] / Samples); 
  Var[4]= sqrt((float)Var[4] / Samples); 
  Var[5]= sqrt((float)Var[5] / Samples); 
  
  ////////Debug stats///////// 
  Debug_Stats("///// Transistor number:");
  Debug_Stats(ADC_name);
  Debugln_Stats(" /////");
  Debug_Stats("/// Point number:");
  Debugln_Stats(Counter_Punto);
  Debug_Stats("Mean[0]:");
  Debugln_Stats((uint16_t)Mean[0]*0.0001875);
  Debug_Stats("Mean[1]:");
  Debugln_Stats((uint16_t)Mean[1]*0.0001875);
  Debug_Stats("Mean[2]:");
  Debugln_Stats((uint16_t)Mean[2]*0.0001875);
  Debug_Stats("Mean[3]:");
  Debugln_Stats((uint16_t)Mean[3]*0.0001875);
  Debug_Stats("Mean[4]:");
  Debugln_Stats((uint16_t)Mean[4]*0.0001875);
  Debug_Stats("Mean[5]:");
  Debugln_Stats((uint16_t)Mean[5]*0.0001875);

  
  Debug_Stats("Var[0]:");
  Debugln_Stats((uint16_t)Var[0]*0.0001875);
  Debug_Stats("Var[1]:");
  Debugln_Stats((uint16_t)Var[1]*0.0001875);
  Debug_Stats("Var[2]:");
  Debugln_Stats((uint16_t)Var[2]*0.0001875);
  Debug_Stats("Var[3]:");
  Debugln_Stats((uint16_t)Var[3]*0.0001875);
  Debug_Stats("Var[4]:");
  Debugln_Stats((uint16_t)Var[4]*0.0001875);
  Debug_Stats("Var[5]:");
  Debugln_Stats((uint16_t)Var[5]*0.0001875);
  
  
  //////////////////////////////////
  ////// Save data to buffer ///////
  Buffer[0][Counter_Punto] = Mean[0];
  Buffer[1][Counter_Punto] = Mean[1];
  Buffer[2][Counter_Punto] = Mean[2];
  Buffer[3][Counter_Punto] = Mean[3];
  Buffer[4][Counter_Punto] = Mean[4];
  Buffer[5][Counter_Punto] = Mean[5];
  Buffer[6][Counter_Punto] = Var[0];
  Buffer[7][Counter_Punto] = Var[1];
  Buffer[8][Counter_Punto] = Var[2];
  Buffer[9][Counter_Punto] = Var[3];
  Buffer[10][Counter_Punto] = Var[4];
  Buffer[11][Counter_Punto] = Var[5];
 
  
  return 1;
}
/*     -END ADC Reading-      */



/*     -START Command Data Sending-      */
//////////////////// Command Data Sending /////////////////////////////
//                                                                   //
//Data sending with ACK verification. This funtion tries to send the //
//message the amount of times defined by the constant "Resend_tries" //
//                                                                   //
///////////////////////////////////////////////////////////////////////
bool Send_Command(String Packet){
int Tries_counter=0,aux_counter=0;
bool ACK_Flag=0;
bool flag=0;
char aux[]="ACK";
int Message_Counter=0;
volatile unsigned long previousMillis = 0;  
volatile unsigned long currentMillis = millis();

  while (SendOk==false){
  currentMillis = millis();  
  
    if(Tries_counter<Resend_times){
    
      if (currentMillis - previousMillis >= Time_out) {
          Serial.print(Packet);
          Serial.write('\r');
          Serial.print(Add_CRC(Packet));
          Serial.write('\n');
          Tries_counter++;
          previousMillis = currentMillis;
      }
      
    }else{
      Serial.println("Impossible to connect. I'm out!");
      return 1; //Error! Impossible to send command.
    }
  
  serialEvent();
  
   if (stringComplete) {
        //Serial.print(currentMillis);
        //Serial.print("Input: ");
        //Serial.print(inputString);
        if (CRC_Validation()==1){
          while((inputString[Message_Counter]!='\r') & (Message_Counter < 100)){
            Message_Counter++;
          }
          flag=0;
          for (int i = 0; i <Message_Counter; i++)
          {
            if(inputString[i]!=aux[i]){
              flag=1;
              break;
            }            
          }
          
          
          if (flag==0){
          SendOk=true;
          }
          //Serial.print("MESSAGE VALIDATED");
        }
        for (short i=0;i<98;i++){
        inputString[i]=0;
        }
        flag=0;
        stringComplete=false;
      }
  }
  Serial.println("Message sended!");
  return 0;  
}

/*     -END Mosfet curve reading-      */

/*     -START ADC Reading-      */
////////////////////////// CRC Validation /////////////////////////////
//                                                                   //
//Validation of input data trough CRC comparation in detination      //
//                                                                   //
//-Subfunction of "Data sending"                                     //
///////////////////////////////////////////////////////////////////////

bool CRC_Validation(){
static uint8_t crc_value=0;
static int Message_Counter_1=0,CRC_Counter_1=0;
static char Rx_data[30]; //Message string
static String Rec_CRC=""; //Message string
static String Rec_Message_String = "";
static char Rec_CRC_array[30]; //Message string
static char CRC_data[3]="nnn";
static int CRC_IN;

//DEBUG PRINT STATUS
 Debug("\n\n---CRC Validation---\n\n");
 Debug("Input: ");
 Debugln(inputString);
 
///Converting message to char array
  
   Debug("Received Message: ");
   for (short i=0;i<98;i++){
      Debug(inputString[i]);
    }
   Debugln("\n");
     

/////Validate message/////

    //Count for chars before CRC value
    Message_Counter_1=0;
    while((inputString[Message_Counter_1]!='\r') & (Message_Counter_1 < 100)){ //Cambiar '1' por '\r'
    Message_Counter_1++;
    }
    //Separe CRC from message
    CRC_Counter_1=0;
    while((inputString[CRC_Counter_1]!='\n') & (CRC_Counter_1 < 100)) {
      CRC_Counter_1++;
        }        
    if(Message_Counter_1<CRC_Counter_1){
      CRC_Counter_1=CRC_Counter_1-Message_Counter_1-1;
    }else{
      CRC_Counter_1=0;
    }
    //Validating CRC counter
    if(CRC_Counter_1>3){
      CRC_Counter_1=3;
    }
    //DEBUG PRINT VARIABLES
    Debug("Message_Counter: ");
    Debugln(Message_Counter_1);
    Debug("CRC_Counter: ");
    Debugln(CRC_Counter_1);
   
    //Saving received CRC
    for(short i=0;i<CRC_Counter_1;i++){
          CRC_data[i]=inputString[Message_Counter_1+i+1];
        }
   //Convert data from Char to int with atoi()    
    Debug("CRC_DATA: ");
    Debugln(CRC_data);    
    CRC_IN=atoi(CRC_data);    
    Debug("CRC_Value: ");
    Debugln(CRC_IN);
    
    //Calculating CRC8
    crc.restart();
    for (int i = 0; i <Message_Counter_1; i++)
    {
      Debug(inputString[i]);
      crc.add(inputString[i]);
    }
    crc.getCRC();
    Debug("Calculated CRC: ");
    Debugln(crc.getCRC());    
    //Comparing calculated CRC to received CRC to validate message
    if(CRC_IN==crc.getCRC()){
      //Serial.println("OKAY! WE DID IT!");
      return 1;
    }else{
      return 0;
    }
}
/*     -END CRC Validation-      */       


/*     -START Add CRC-      */
///////////////////////////// Add CRC /////////////////////////////////
//                                                                   //
//Add CRC to data sent to the function.                              //
//                                                                   //
//-Subfunction of "Data sending"                                     //
///////////////////////////////////////////////////////////////////////
int Add_CRC(String Packet){
char Sending_Message[30];
  
  crc.restart(); //CRC Restart
  //Debug("\n\n---ADD CRC---\n\n");
  //Debug("Input String: ");
  //Debug(Packet);
  
  //String to Char array 
  Packet.toCharArray(Sending_Message,Packet.length()+1);
  //Debug("Sending_Message: ");
  //Debugln(Sending_Message);
  //Debug("Size of: ");
  //Debugln(Packet.length());
  
  //Obtaing CRC
  crc.add((uint8_t*)Sending_Message ,Packet.length());    
  return crc.getCRC();
  //Debug("Message CRC: ");
  //Debugln(crc.getCRC());
}
/*     -END Add CRC-      */

/*     -START Internal temperature-      */
/////////////////// SENSOR DE TEMPETURA INTERNO ///////////////////////
//                                                                   //
//Se toma la primera temperatura incial como referencia              //
//y se evalua siempre                                                //
//-Subfunction of "control_interno"                                  //
///////////////////////////////////////////////////////////////////////

int control_temp_interno(void){
  static bool control_medicion=0;
  char temperatura_incial;
int16_t temp;
  unsigned int wADC;
    float t;

  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);
  delay(20);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  wADC = ADCW;

  // CALIBRACION//
  t = (wADC - 324.31 ) / 1.22;
  if (control_medicion == 0){
    temperatura_incial=t;
    control_medicion=1;
  }
  
  if ((t-temperatura_incial)>4){
   
    // ACA SE PODRIA PONE N° DE FALLA//
    // Se podria hablitar una interrupción al ingresar en esta linea//
    // TENER CUIDADO CON NO CORTAR LA COMUNICACION DEL MCU POR LA INTERRUPCION//
    Serial.println (" Se calento");
  }
  //ACA SE DEBE CONSIDERAR 2BYTE PERO CON SIGNO// 
  // EN VEZ DE UNINT16_T -> INT16_T// 
  //GUARDADO EN EL SHEET //
  
  temp =((int16_t) (t*1000)); 
  Serial.println(temp,1);
  //delay(1000);
  return temp;

}

/*     -START I2C Verification-      */
/////////////////////// I2C Verification //////////////////////////////
//                                                                   //
//I2C directions are asked and compared to the expected ones         //
//                                                                   //
//-Subfunction of "I2C_Verification"                                  //
///////////////////////////////////////////////////////////////////////

int I2C_Verification(void){
  byte error, address;
  int nDevices;
  int Detected_error=0;
  bool ADC_1=0,ADC_2=0,ADC_3=0,ADC_4=0,DAC=0;
 
  DebugErrorDetln("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      DebugErrorDet("I2C device found at address 0x");
      if (address<16)
      DebugErrorDet("0");
      DebugErrorDetHex(address);
      DebugErrorDetln("  !");
 
      nDevices++;
      //Look for ADC_1
      if (address==0x48)ADC_1=1;
      //Look for ADC_2
      if (address==0x49)ADC_2=1;
      //Look for ADC_1
      if (address==0x4A)ADC_3=1;
      //Look for ADC_1
      if (address==0x4B)ADC_4=1;
      //Look for ADC_1
      if (address==0x64)DAC=1;
      
    }
    else if (error==4)
    {
      DebugErrorDet("Unknown error at address 0x");
      if (address<16)
        DebugErrorDet("0");
      DebugErrorDetHex(address);
      DebugErrorDetln("  !");
    }    
  }

  //Detected error generation
  if (ADC_1 == 0){
    Detected_error=2+Detected_error;
    DebugErrorDetln("ADC_1 not found\n");
    }
  if (ADC_2 == 0){
    Detected_error=3+Detected_error;
    DebugErrorDetln("ADC_2 not found\n");
    }  
  if (ADC_3 == 0){
    Detected_error=4+Detected_error;
    DebugErrorDetln("ADC_3 not found\n");
    } 
  if (ADC_4 == 0){
    Detected_error=5+Detected_error;
    DebugErrorDetln("ADC_4 not found\n");
    } 
  if (DAC == 0){
    Detected_error=6+Detected_error;
    DebugErrorDetln("DAC not found\n");    
    } 
  if (nDevices == 0){
    Detected_error=1;
    DebugErrorDetln("No I2C devices found\n");
    }
    
    
   DebugErrorDetln("done\n");
    return Detected_error;    
  
}

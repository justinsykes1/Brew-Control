// This #include statement was automatically added by the Particle IDE.
#include <PID-AutoTune.h>

// This #include statement was automatically added by the Particle IDE.
#include <ITEADLIB_Nextion.h>

// This #include statement was automatically added by the Particle IDE.
// #include <ThingSpeak.h>

// This #include statement was automatically added by the Particle IDE.
#include <pid.h>

// This #include statement was automatically added by the Particle IDE.
#include <spark-dallas-temperature.h>

// This #include statement was automatically added by the Particle IDE.
#include "OneWire/OneWire.h"


//PID
//Define Variables we'll be connecting to
double input, output, setPoint, Kp, Ki, Kd;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, PID::DIRECT);

//timer and timer
int startTime,tmrcurrent,tmrHour,tmrMinute, tmrSecond;
String currentTime,tmr;

// PWM
int PWMPin = A4;
//pump
int pumpPin = D0;
int pumpState = 0;

// DS18B20 Thermometer Stuff
#define ONE_WIRE_BUS D1
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress mashInThermometer = { 0x28, 0xFF, 0x46, 0xC6, 0x03, 0x15, 0x02, 0x46 };  //Change to match actual device
DeviceAddress mashOutThermometer = {0x28, 0xFF, 0x3A, 0xC4, 0x03, 0x15, 0x02, 0x0D };
DeviceAddress mashThermometer = { 0x28, 0xFF, 0x35, 0xCB, 0x03, 0x15, 0x02, 0xF8 };
DeviceAddress HLThermometer = { 0x28, 0xFF, 0x93, 0xFF, 0x04, 0x15, 0x03, 0x35 };
double mashInTempF = -1;
double mashOutTempF = -1;
double mashTempF = -1;
double HLTempF = -1;
// Timer displayupdate(1000, updateDisplay);  //display refresh and temperature read.  //Removed as this apprease to be causeing display issues with async nature of calls.
unsigned long lastUpdate = millis();  //setup for temperature and display update fucntion call
unsigned long displayUpdate = 1000; //update display will be called once per secong.

// Functions declare.
void update18B20Temp(DeviceAddress deviceAddress, double &tempF);
int pumpS (String x);
int gsetT (String y);
int PIDSet(String y);

//void updateDisplay();
//void thingspaekUpdate();

//Thingspeak setup

/*
unsigned long myChannelNumber = 58086;
const char * myWriteAPIKey = "449NU2XMSH31NZSG";
TCPClient client;
*/
Timer thing(30000, thingspeakUpdate);  // once per 30 sec updates data to thingspeak.


// Display setup
/*
 *******************************************************************
 * Nextion component for page:Home
 *
 *******************************************************************
 * Declare a button object [page id:0,component id:1, component name: "b0"].
*/
NexHotspot disPump = NexHotspot(1, 20, "disPump");  //pump on off button.
NexText disMash = NexText(1, 1, "disMash");
NexText disMashIn = NexText(1, 4, "disMashIn");
NexText disMashOut = NexText(1, 6, "disMashOut");
NexText disHL = NexText(1, 8, "disHL");
NexNumber disSetTemp = NexNumber(1, 10, "disSetTemp");
NexText disTime = NexText(1, 12, "disTime");
NexText disTmr = NexText(1, 14, "disTmr");
NexHotspot tmrReset = NexHotspot(1, 16, "timerReset");
NexPicture disPPic = NexPicture(1, 19, "disPPic");

/*
 *******************************************************************
 * Nextion component for page:PIDset
 *
 *******************************************************************
*/
NexNumber disPIDOut = NexNumber(2, 12, "disPIDOut");
NexNumber n0 = NexNumber(2, 11, "n0");
NexButton disPIDTune = NexButton(2, 3, "PIDTune");  //requests the Photon to update the PID tune settings on the disply.
NexButton disSet = NexButton(2,13, "disSet");  //button to send the target temp setting from the disply to the Photon.

/*
 *******************************************************************
 * Nextion component for page:PIDTune
 *
 *******************************************************************
*/
NexButton disPIDSet = NexButton(3, 1, "disPIDSet");  //button that sends the PID tune settings to the Photon
char buffer[100] = {0};
NexText disKp = NexText(3, 7, "disKp");
NexText disKi = NexText(3, 8, "disKi");
NexText disKd = NexText(3, 9, "disKd");

/*
 *******************************************************************
 * Nextion component for page:NumPad
 *
 *******************************************************************
*/
NexButton disNumEnter = NexButton(4, 13, "disNumEnter");  //button that sends the PID tune settings to the Photon

/*
 *******************************************************************
 * Nextion component all page
 *
 *******************************************************************
*/
NexPage HomePage = NexPage(1, 0, "Home");
NexPage PIDSetPage = NexPage(2, 0, "TempSet");
NexPage PIDTunePage = NexPage(3, 0, "PIDTune");



/*
* Register a button object to the touch event list.
*/
NexTouch *nex_listen_list[] =
{
    &disPump, &tmrReset,
    &disPIDTune, &disSet,
    &disNumEnter,
    NULL
};

//Initial Setup code.  Runs once at power on.
void setup()
{

    //Pump setup
    pinMode(pumpPin,OUTPUT);
    pinMode(D7,OUTPUT);
    digitalWrite(pumpPin,LOW);

    //PID
    //initialize the variables we're linked to

    setPoint = 100;

    //turn the PID on
    EEPROM.get(10, Kp);  //loads the P,I,D tune settings from EEPROM.  This maintains them through power cycles.
    EEPROM.get(20, Ki);
    EEPROM.get(30, Kd);
    myPID.SetOutputLimits(0,100); //limits the output between 0 and 100% for the PWM output.
    myPID.SetTunings(Kp, Ki, Kd);  //Loads the PID tune values.
    myPID.SetSampleTime(10000);  //sets the period, in Milliseconds. 10 sec.
    myPID.SetMode(PID::AUTOMATIC);  //starts the PID in automatic mode.

    //PWM setup for the PID output drivign a solid state relay.
    pinMode(PWMPin, OUTPUT);
    // DS18B20 configuration.
    sensors.begin();
    sensors.setResolution(mashInThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(mashOutThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(mashThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(HLThermometer, TEMPERATURE_PRECISION);
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    // displayupdate.start(); //starts the timer that updates display data and requests temperature sensors updated temperature.


    //spark function setup
    // SPARK FUNCTION SET UP
    Particle.function("pumpSwitch", pumpSwitch); // expects a 0 for off and a 1 for on.  Will fail with any other string content.
    Particle.function("setTarget", setTarget); //expects a string of integer that is the set temp float *10 to remove the decimal.
    Particle.function("PIDTune", PIDTune); //sends Kpxx.xxKixx.xxKdxx.xx in a string

    //spark variable setup
    Particle.variable("mashInTemp", &mashInTempF, DOUBLE);
    Particle.variable("mashOutTemp", &mashOutTempF, DOUBLE);
    Particle.variable("mashTemp", &mashTempF, DOUBLE);
    Particle.variable("HLTemp", &HLTempF, DOUBLE);
    Particle.variable("pumpState", &pumpState, INT);     // Spark variable: 0 = Off, 1 = On.
    Particle.variable("Setpoint", &setPoint, DOUBLE);  //  Setpoint for the PID controller.
    Particle.variable("Pidout", &output, DOUBLE);  //Current output settign of the PID controller.
    Particle.variable("Kp", &Kp, DOUBLE); //temproary for debug monitoring and testing phone app and controller.
    Particle.variable("Ki", &Ki, DOUBLE);
    Particle.variable("Kd", &Kd, DOUBLE);

    //Thingspeak init.  Thinkspeak is used to capture temperatues and save the data for graphing.
    //  ThingSpeak.begin(client);
    thing.start(); //starts the timer to call function to update thingspeak data.
    Particle.publish("Clear","Null",60, PRIVATE);  //start a new chart at power up by clearing all previous data from the channel.


    /* Set the baudrate which is for debug and communicate with Nextion screen. */
    nexInit();
    /* Register the pop event callback function of the current button component. */
    disPump.attachPush(pumpPopCallback, &disPump);
    disSet.attachPop(TempSetPopCallback, &disSet);
    disNumEnter.attachPop(updatePIDTune, &disNumEnter);
    tmrReset.attachPush(tmrResetPopCallback, &tmrReset);
    disPIDTune.attachPop(PIDTuneEnterPopCallback, &disPIDTune);
    dbSerialPrintln("setup done");
    setBaudrate(115200);


    //Real Time clock setup.
    Time.zone(-7);
    startTime = Time.now();


    Serial.begin(9600);   // open serial over USB
    // On Windows it will be necessary to implement the following line:
    // Make sure your Serial Terminal app is closed before powering your Core
    // Now open your Serial Terminal, and hit any key to continue!
 //while(!Serial.available()) SPARK_WLAN_Loop();

}

/**********************************************************************************************************
Main Code Loop
************************************************************************************************************
*/

void loop()
{


    /*
         * When a pop or push event occured every time,
         * the corresponding component[right page id and component id] in touch event list will be asked.
         */
    nexLoop(nex_listen_list);


    //PID compute calls the PID sending it the current temp data.
    //  PID will only take action every 10 seconds based on initial setup.
    input = mashTempF;

    if (input <= setPoint-1) //avoids integer wind up as system is very slow to move integer component will be at 100% output and cause an overshoot.
    {
        myPID.SetTunings(Kp, 0, 0);
    }
    else
    {
        myPID.SetTunings(Kp, Ki, Kd);
    }

    myPID.Compute();


    if (mashInTempF<setPoint+20)  // safty check to turn off RIMM if output greater than 20 degrees over setpoint.
    {
        analogWrite(PWMPin,(output*2.55),1);
    }
    else
    {
         output = 0;
         analogWrite(PWMPin,output);
    }


    if ((millis() - lastUpdate) > displayUpdate)
    {
        updateDisplay();
        lastUpdate = millis();
    }

}





//Spark Function to turn pump on or off
// SPARK FUNCTION pumpSwitch
int pumpSwitch (String x)                 // String x will be a 0 for relay off; a 1 for relay on)
{
    int tempValue = x.toInt();      // Extract integer
    if (tempValue == 0)
    {
        pumpState = 0;              // Set for relay off
        digitalWrite(pumpPin, LOW);  // Switch relay off
        digitalWrite(D7, LOW);      // Turn on Spark blue LED
    }
    else if (tempValue == 1)
    {
       pumpState = 1;               // Set for relay on
       digitalWrite(pumpPin, HIGH); // Switch relay on
       digitalWrite(D7, HIGH);      // Turn off Spark blue LED
    }
    return pumpState;
}

//Spark Function to updated PID temperature target setpoint

int setTarget (String y)    // String y will contain the set temperature as an int.
{
    int tempValue1 = y.substring(0).toInt();
    setPoint = tempValue1;
    return -1;
}

//Spark Function to update PID settings sent form phone app or called by display setting.
// SPARK FUNCTION PIDset
int PIDTune (String y)       // String y   format Kpxx.xxKixx.xxKdxx.xx where xx.xx contain float values
{
    Kp = y.substring(y.indexOf("Kp")+2, y.indexOf("Ki")).toFloat(); // converts part of string to float
    Ki = y.substring(y.indexOf("Ki")+2, y.indexOf("Kd")).toFloat();
    Kd = y.substring(y.indexOf("Kd")+2).toFloat();
    EEPROM.put(10, Kp);  //Loades new values into EEPROM
    EEPROM.put(20, Ki);
    EEPROM.put(30, Kd);
    myPID.SetTunings(Kp, Ki, Kd); //Updates PID settings

    return -1;
}

/*
 * Button component pop callback function.
 Used to turn the pump on and off from the diaplay.  Toggles based on pump state variable.
 */
void pumpPopCallback(void *ptr)
{

    dbSerialPrintln("Pump PopCallback");
    memset(buffer, 0, sizeof(buffer));
    delay(50);
    /* Get the text value of button component [the value is string type]. */
    if (pumpState == 0)
    {
        pumpSwitch("1");
        disPPic.setPic(9);
    }
    else
    {
        pumpSwitch("0");
        disPPic.setPic(8);
    }
}

/*
 * Buttonr component pop callback function.
 recieves the updated target temperature from the display thin calls the gSetT functions
 passing it the string containg the target temperatue.
*/
void TempSetPopCallback(void *ptr)
{

    dbSerialPrintln("tempsetPopCallback");
    uint32_t number = 1;
    delay(100);
    n0.getValue(&number);
    setPoint = number;
}

// loads the current PID settings to the display.
void PIDTuneEnterPopCallback(void *ptr)
{
  PIDTunePage.show();
  disKp.setText(String(Kp,2));
  disKi.setText(String(Ki,3));
  disKd.setText(String(Kd,2));
}


//FUNCTION to read temp from Dallas temp sensors.
void update18B20Temp(DeviceAddress deviceAddress, double &tempF)
{

  double tempTemp = sensors.getTempF(deviceAddress);
 if (tempTemp > -100)
  {
  tempF = tempTemp;
  }
}


void updateDisplay()  //updates display and calls for temp update.  must be greater than 750ms
{
    Serial.println("updateDisplay called");
    //refresh temps.
    update18B20Temp(mashInThermometer, mashInTempF);
    update18B20Temp(mashOutThermometer, mashOutTempF);
    update18B20Temp(mashThermometer, mashTempF);
    update18B20Temp(HLThermometer, HLTempF);
    sensors.requestTemperatures();

uint8_t pageCurrent;
   sendCurrentPageId(&pageCurrent);
   switch(pageCurrent) {
      case 1 :
         disSetTemp.setValue(setPoint);
         disMashIn.setText(String(mashInTempF,1)); //Send temp to nextion display.
         disMashOut.setText(String(mashOutTempF,1)); //Send temp to nextion display.
         disMash.setText(String(mashTempF,1)); //Send temp to nextion display.
         disHL.setText(String(HLTempF,1)); //Send temp to nextion display.
         // time of day section
        currentTime = Time.hourFormat12();
        currentTime.concat(":");
        if (Time.minute() <= 9)
        {
            currentTime.concat("0");
        }
        currentTime.concat(Time.minute());
        currentTime.concat(":");
        if (Time.second() <= 9)
        {
            currentTime.concat("0");
        }
        currentTime.concat(Time.second());
        if (Time.isPM())
        {
            currentTime.concat(" PM");
        }
        disTime.setText(currentTime);
        tmrcurrent = Time.now() - startTime;
        tmrSecond = tmrcurrent % 60;
        tmrMinute =((tmrcurrent % 3600) / 60);
        tmrHour =((tmrcurrent % 86400) / 3600);
        tmr = tmrHour;
        tmr.concat(":");
        if (tmrMinute <= 9)
        {
            tmr.concat("0");
        }
        tmr.concat(tmrMinute);
        tmr.concat(":");
        if (tmrSecond <= 9)
        {
            tmr.concat("0");
        }
        tmr.concat(tmrSecond);
        disTmr.setText(tmr);
        break;
      case 2 :
        disPIDOut.setValue(output);
        break;
      case 3 :
        //disKp.setText(String(Kp,2));
        //disKi.setText(String(Ki,2));
        //disKd.setText(String(Kd,2));
        break;

      default :
         Serial.println("Invalid value" );
   }

}



// Function call to reset the brewer timer.

void tmrResetPopCallback(void *ptr)
{
    startTime = Time.now();
}






void updatePIDTune(void *ptr)  //reads PID values from display and generates a string to send to PIDSet funcion to save to EEPROM.
{
   Serial.print("updatepidsety called: ");

   PIDTunePage.show();
   delay(500);
   String temp = "Kp";
   memset(buffer, 0, sizeof(buffer));
   disKp.getText(buffer, sizeof(buffer));
   temp += String(buffer);
   temp += "Ki";
   memset(buffer, 0, sizeof(buffer));
   disKi.getText(buffer, sizeof(buffer));
   temp += String(buffer);
   temp += "Kd";
   memset(buffer, 0, sizeof(buffer));
   disKd.getText(buffer, sizeof(buffer));
   temp += String(buffer);
   PIDTune(temp);
   disKp.setText(String(Kp,2));
   disKi.setText(String(Ki,3));
   disKd.setText(String(Kd,2));
}

/* Function that sends updated temperature data to thingspeak.
*/
void thingspeakUpdate()
{
   Serial.println("thingspeak update called");
    float f1 = (float) mashInTempF;
    float f2 = (float) mashOutTempF;
    float f3 = (float) mashTempF;
     /*
    ThingSpeak.setField(1,f1);
    ThingSpeak.setField(2,f2);
    ThingSpeak.setField(3,f3);
    // Then you write the fields that you've set all at once.
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    */
    Particle.publish("Mash_Temp", String(f3), 60, PRIVATE);
}



/*
*******************************************************************************************
PID Auto tune Function
*******************************************************************************************
*/

void PIDAutoTune()
{



//double kpmodel=1.5, taup=100, theta[50];
//double outputStart=5;
double aTuneStep=50, aTuneNoise=0.15, aTuneStartValue=50;
unsigned int aTuneLookBack=30;

boolean tuning = true;

PID_ATune aTune(&input, &output);


myPID.SetMode(PID::MANUAL);

output=aTuneStartValue;
aTune.SetNoiseBand(aTuneNoise);
aTune.SetOutputStep(aTuneStep);
aTune.SetLookbackSec((int)aTuneLookBack);
tuning = true;

  while(tuning)
  {
    if ((millis() - lastUpdate) > displayUpdate)
    {
        updateDisplay();
        lastUpdate = millis();
    }
    input = mashOutTempF;
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp,Ki,Kd);
      EEPROM.put(10, Kp);  //Loades new values into EEPROM
      EEPROM.put(20, Ki);
      EEPROM.put(30, Kd);
    }
    analogWrite(PWMPin,(output*2.55));
  }

myPID.SetMode(PID::AUTOMATIC);

}

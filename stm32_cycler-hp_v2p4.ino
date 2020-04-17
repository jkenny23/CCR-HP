/*
    Main code for the CCR v2.4
    Details here: http://rev0.net/index.php?title=CCR

    2019 Justin Kenny
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel-ANDnXOR.h>
#include <EEPROM.h>

// Hardware Configurations:
// HW 2.0 - First CCR v2 HW using individual N- and P- gate drivers, max current 3A from 12V, 3.5A from 5V
// HW 2.4 - Second CCR v2 HW using synchronous gate drivers, max current +/-6A from 7.5-12V
// 1S - Using 220k/150k divider for 0-4.84V measurement
// 2S - Using 430k/150k divider for 0-9.46V measurement
// OLED - Using OLED display or not (not currently compatible with LEDs)
//#define OLED_ENABLED
#define HW_1_0
//#define REGEN_ENABLED

#ifdef OLED_ENABLED
  #include <Adafruit_GFX_AS.h>
  #include <Adafruit_SSD1306_STM32.h>
  
  Adafruit_SSD1306 display(-1);

  #if (SSD1306_LCDHEIGHT != 32)
    #error("Height incorrect, please fix Adafruit_SSD1306.h!");
  #endif
#endif

/*#if defined(__arm__)
  extern "C" char* sbrk(int incr);
  static int FreeStack() {
    char top = 't';
    return &top - reinterpret_cast<char*>(sbrk(0));
  }
#endif*/

volatile int interruptCounter;

const char vers[] = "2.0-04032020"; 

#define AFTERDISWAIT 300//300 //300s after charging wait time
#define CHGSETTLEWAIT 1//30 //30s after starting charge settle time
#define AFTERCHGWAIT 1//60 //60s after charging wait time
#define LOOP_PERIOD 500 //In microseconds
#define LTCC_THRES 4 //4s of <constant current threshold before ending charge
#define CHARGE 1
#define DISCHARGE 2
#define DISCONNECT 0
#define REGEN 3
#define LED_OFF 0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3
#define LED_CYAN 4
#define LED_BLUE 5
#define LED_PURPLE 6
#define LED_WHITE 7
//Pin definitions - Analog inputs
#define AC1VL PB0 //Cell 1 voltage input low range
#define AC1AL PB1 //Cell 1 current input low range
#define AC1VH PA6 //Cell 1 voltage input high range
#define AC1AH PA7 //Cell 1 current input high range
#define AC1T PA3 //Cell 1 temperature input
#define AC1R PA1 //Cell 1 reverse voltage input
#define AIREF PA5 //Reference voltage input
//Pin definitions - Digital outputs
#define OC1NF2 PB6 //CC 1 setting output
#define C1DOFF PB15 //CC 1 enable output - high = top right FET enabled
#define C2DOFF PA15 //CC 2 enable output - high = other 3 FETs enabled
#define C1ON PB14 //Cell enable/disable - low = FETs on
#define FANON PB4 //Fan control outputc1
#define WSDI PB13 //LED control pin, inverted logic
#define HSTH1 0
#define HSTH2 1
#define BUFV 2
#define HVV 3
//Pin definitions - Digital inputs
#define S1 PB9 //Switch 1
#define S2 PC13 //Switch 2
#ifdef HW_1_0
  #define MAX_CHG_CUR -9000
  #define MAX_CHG_VOL 84000
  #define MIN_CHG_VOL 1000
  #define MIN_CCC 10
  #define MAX_DIS_CUR 30000
  #define MIN_DIS_VOL 700
  #define MAX_DIS_VOL 44000
  #define MINBUCKDUTY 1
  #define MAXBUCKDUTY 397
  #define MAX_VBUF 85000
  #define DEF_CHG_CUR -1500
  #define DEF_CHG_VOL 4200
  #define DEF_CCC 50
  #define DEF_DIS_CUR 1500
  #define DEF_DIS_VOL 2700
  //CCR v1.0 HW configuration: (ToDo: Update)
  #define AUXSEL2 PB5 //Lo = HS thermistor IRFP, Hi = HS thermistor STB or 12V
  #define AUXSEL1 PB12 //Lo = 12V, Hi = HS thermistor STBs
  #define OC1PF PA9 //Buck low FET control output, positive logic; higher PWM duty = higher output voltage /LIN
  #define OC1ON PA8 //Buck high FET control output, positive logic; higher PWM duty = higher output voltage HIN
  #define AC1R PA1
  #define ABUFV PA4
  #define AHVV PA0
#endif
#define DEF_DIS_MODE 0
#define DEF_PSU_MODE 1
#define DEF_CYCLES 1
#define OVT_THRESH 70
#define OVV_THRESH 85000
#define LED_BRIGHTNESS 80 //1-255 for LED brightness
#define NIMH_DV 10
#define DEF_CELL_TYPE 0
#define STARTUP_CYCLES 5 //number of cycles-1 (0.25ms each) to delay before turning on cell

volatile int16 charge_current_1 = -1500;
volatile uint16 charge_voltage_1 = 4200;
volatile uint16 discharge_current_1 = 1500;
volatile uint16 discharge_voltage_1 = 2700;
volatile uint16 ccc_1 = 50; //Charge CC cutoff in mA (50mA)
volatile uint16 num_cycles_1 = 1;
volatile uint8 discharge_mode_1 = 0;
volatile uint8 psu_dir_1 = 0;
volatile uint16 tm_duty_1 = 0;
volatile int16 charge_current_2 = -1500;
volatile uint16 charge_voltage_2 = 4200;
volatile uint16 discharge_current_2 = 1500;
volatile uint16 discharge_voltage_2 = 2700;
volatile uint16 ccc_2 = 50; //Charge CC cutoff in mA (50mA)
volatile uint16 num_cycles_2 = 1;
volatile uint8 discharge_mode_2 = 0;
volatile uint8 psu_dir_2 = 0;
volatile uint16 tm_duty_2 = 0;
volatile uint8 slot1_startup = 0;
volatile uint8 slot2_startup = 0;
volatile uint16 cell1_vmax = 0;
volatile uint16 cell2_vmax = 0;
volatile uint8 cell1_type = 0; //0 = Li-Ion, 1 = NiMH
volatile uint8 cell2_type = 0;
volatile uint8 displayEnabled = 0;
volatile uint8 fanSpeed = 0; //0 = off, 1 = pwm, 2 = on

//Initialize reference voltage with default until read out of EEPROM
#define REFAVALINIT 2048000.0f
volatile float REFAVAL = REFAVALINIT;
//EEPROM address for reference value: 0
#define REFAVALADDR 0
//Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
//Voltage reads high => increase ADC2V value
//Current reads high => increase ADC2I value
//EEPROM address for low range voltage value: 1
#define ADC2VLADDR 1
//EEPROM address for high range voltage value: 2
#define ADC2VHADDR 2
#define ADC2VLINIT 432.981f
#define ADC2VHINIT 43.2981f
//todo: reset adc2v2init during "l" calibration for storing correction factors
//EEPROM address for low current value: 3
#define ADC2ILADDR 3 //10A/V * 15/23.2 => 80.2508LSB/A
#define ADC2ILINIT 124.121f //0.5mOhm -> 40A = 20mV * 200 = 4V, -10A = -5mV * 200 = -1V (248.242 for 2nd CCR-HP)
//EEPROM address for high current value: 4
#define ADC2IHADDR 4
#define ADC2IHINIT 248.242f
//EEPROM address for 12V current value: 5
#define B12V2VADDR 5
#define B12V2VINIT 3.22266f //x/4096*3.3*1000*4
//EEPROM address for HV current value: 6
#define BHVV2VADDR 6
#define BHVV2VINIT 22.5586f //x/4096*3.3*1000*28
//EEPROM address for low current divider value: 7
#define ADC2IDLADDR 7
#define ADC2IDLINIT 1.5467f
//EEPROM address for low current divider value: 8
#define ADC2IDHADDR 8
#define ADC2IDHINIT 1.5467f
volatile float B12V2V = B12V2VINIT;
volatile float BHVV2V = BHVV2VINIT;
#define VR12V 22.5586f
volatile float ADC2IL = ADC2ILINIT;
volatile float ADC2VL = ADC2VLINIT;
volatile float ADC2IH = ADC2IHINIT;
volatile float ADC2VH = ADC2VHINIT;
volatile float ADC2IDL = ADC2IDLINIT;
volatile float ADC2IDH = ADC2IDHINIT;
volatile unsigned int initbuckduty1 = 16;
volatile unsigned int initbuckduty2 = 16;
volatile float corr_factor = 1.0f;
#define TOFFS1 0.0f
#define TOFFS2 0.0f
#define TOFFSHS1 0.0f
#define TOFFSHS2 0.0f

Adafruit_NeoPixel leds = Adafruit_NeoPixel(1, WSDI, NEO_GRB + NEO_KHZ800);

uint16 adcval1 = 0;
uint16 adcval2 = 0;
uint16 adcval3 = 0;
uint16 adcval4 = 0;
uint16 adcval5 = 0;
uint16 adcval6 = 0;
uint32 adciref = 0;
uint16 vbuf_i = 0;
uint16 vhv_i = 0;
uint8 loopcnt = 0;
//uint32 interruptcnt = 0;
volatile int16 duty1 = 0;
uint16 loop1 = 0;
volatile uint8 state1 = 8;
/* Mode List:
    0 = c = Charge
    1 = d = Discharge
    2 = y = Cycle
    3 = p = Power Supply
    4 = t = Test
    5 = n = None (Parking)
*/
volatile uint8 mode1 = 5;
uint8 irstate1 = 0;
uint16 cycle_count_1 = 0;
volatile uint16 settle1 = 0;
int16 vr1 = 0;
int16 vr2 = 0;
int16 vbati1 = 0;
int16 ibati1 = 0;
uint8 ltcc_count = 0;
int16 vbat_now1 = 0;
int16 ibat_now1 = 0;
float tmpfl = 0;
float vbat1 = 0;
float ibat1 = 0;
volatile int vbat_1_1 = 0;
volatile int ibat_1_1 = 0;
float vload1 = 0;
float iload1 = 0;
float voc11 = 0;
float voc21 = 0;
volatile float mah1 = 0;
volatile float mwh1 = 0;
volatile float ir1 = 0;
volatile float temp1 = 0;
float voc12 = 0;
volatile float temp_t = 0;
volatile uint16 adctemp = 0;
volatile uint16 devicepwr = 0;
volatile uint32 tick = 0;
volatile uint32 last_tick = 0;
volatile uint32 last_tick_fan = 0;
//HardwareTimer timer1(1);
//HardwareTimer timer2(2);

boolean fantoggle = false;

//y 1500 2700 0 1500 4200 1000 1\r\n
//32 chars with /r/n + 8 corrections (e.g. 4 backspace + 4 chars)
#define MAXCHARS 40
char receivedChars[MAXCHARS];
boolean newData = false;

unsigned short getAuxADC(unsigned char adcsel) {
  switch (adcsel)
  {
    /*#define ABUFV PA4
      #define AUXSEL2 PB5 //Lo = HS thermistor IRFP, Hi = HS thermistor STB or 12V
      #define AUXSEL1 PB12 //Lo = 12V, Hi = HS thermistor STBs*/
    case HSTH1:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, HIGH);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
    case HSTH2:
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, LOW);
      return analogRead(ABUFV);
      break;
    case BUFV:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, LOW);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
    case HVV:
      return analogRead(AHVV);
      break;
    default:
      pinMode(AUXSEL1, OUTPUT);
      digitalWrite(AUXSEL1, HIGH);
      pinMode(AUXSEL2, OUTPUT);
      digitalWrite(AUXSEL2, HIGH);
      return analogRead(ABUFV);
      break;
  }
}

unsigned short getCell1RV(){
  int16 rv = 0;

  //rv = (int16)((((float)analogRead(AC1R)) * VR12V) - (((float)analogRead(AC1VH)) * ADC2VH / 2.0));
  rv = (int16)((((float)analogRead(AC1R)) * VR12V) - (((float)analogRead(AC1VL)) * ADC2VL / 2.0));

  if(rv < 0)
    return 0;
  else
    return 0;//rv;
}

unsigned short getChgPwr(){
  unsigned long power = 0;
  
  if ((state1 == 3) || (state1 == 4)) //Charging states
  {
    power = (charge_current_1*charge_voltage_1*-1)/8000;
  }
  
  return (unsigned short)power; //Returns power in mW
}

unsigned short getDisPwr(){
  unsigned long power = 0;
  
  if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
  {
    power = discharge_current_1*charge_voltage_1/1000;
  }

  return (unsigned short)power; //Returns power in mW
}

void setChg1(unsigned char state) {
  switch (state)
  {
    /*#define OC1PF PA9 //Buck low FET control output, positive logic; higher PWM duty = higher output voltage /LIN
      #define OC1ON PA8 //Buck high FET control output, positive logic; higher PWM duty = higher output voltage HIN
      #define OC1NF2 PB6 //CC 1 setting output
      #define C1DOFF PB15 //CC 1 enable output - high = top right FET enabled
      #define C2DOFF PA15 //CC 2 enable output - high = other 3 FETs enabled
      #define C1ON PB14 //Cell enable/disable - low = FETs on*/
    case DISCONNECT:
      #ifdef HW_1_0
        //Disable buck - set high FET control low
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        //Disable buck - set low FET control low (inverted logic)
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH);
        //Disable protection FETs
        pinMode(C1ON, OUTPUT);
        digitalWrite(C1ON, HIGH);
        //Disable 3 discharge FETs
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, LOW);
        //Disable top right FET
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, LOW);
        //Set CC PWM off
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW); //Non-inverting mode
      #endif
      break;
    case CHARGE:
      #ifdef HW_1_0
        presetDuty1();
        duty1 = initbuckduty1; //Assume 3.7V initial cell voltage - optimise later
        //Enable buck - set high FET control PWM
        pinMode(OC1ON, PWM);
        pwmWrite(OC1ON, duty1);
        //Enable buck - set low FET control PWM
        pinMode(OC1PF, PWM);
        pwmWrite(OC1PF, duty1);
        slot1_startup = STARTUP_CYCLES;
        //Disable protection FETs until synchronous buck started
        pinMode(C1ON, OUTPUT);
        digitalWrite(C1ON, HIGH);
        //Disable 3 discharge FETs
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, LOW);
        //Disable top right FET
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, LOW);
        //Set CC PWM off
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW); //Non-inverting mode
      #endif
      break;
    case DISCHARGE:
      #ifdef HW_1_0
        //Disable buck - set high FET control low
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        //Disable buck - set low FET control low (inverted logic)
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH);
        //Set CC PWM
        duty1 = 0;
        pinMode(OC1NF2, PWM);
        pwmWrite(OC1NF2, duty1); //Non-inverting mode
        //Enable top right FET
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, HIGH);
        if(discharge_current_1 > 3000)
        {
          //Enable 3 discharge FETs if setting >3A
          pinMode(C2DOFF, OUTPUT);
          digitalWrite(C2DOFF, HIGH);
        }
        //Enable protection FETs
        pinMode(C1ON, OUTPUT);
        digitalWrite(C1ON, LOW);
      #endif
      break;
    default:
      #ifdef HW_1_0
        //Disable buck - set high FET control low
        pinMode(OC1ON, OUTPUT);
        digitalWrite(OC1ON, LOW);
        //Disable buck - set low FET control low (inverted logic)
        pinMode(OC1PF, OUTPUT);
        digitalWrite(OC1PF, HIGH);
        //Disable protection FETs
        pinMode(C1ON, OUTPUT);
        digitalWrite(C1ON, HIGH);
        //Disable 3 discharge FETs
        pinMode(C2DOFF, OUTPUT);
        digitalWrite(C2DOFF, LOW);
        //Disable top right FET
        pinMode(C1DOFF, OUTPUT);
        digitalWrite(C1DOFF, LOW);
        //Set CC PWM off
        pinMode(OC1NF2, OUTPUT);
        digitalWrite(OC1NF2, LOW); //Non-inverting mode
      #endif
      break;
  }
}

void presetDuty1(void)
{
  unsigned int inputV;
  unsigned int cellV;
  
  inputV = (int)(((float)getAuxADC(AHVV)) * BHVV2V);
  //Convert ADC value to value in Volts
  /*volatile float ADC2IL = ADC2ILINIT;
    volatile float ADC2VL = ADC2VLINIT;
    volatile float ADC2IH = ADC2IHINIT;
    volatile float ADC2VH = ADC2VHINIT;*/
  cellV = (int)((((float)analogRead(AC1VL)) / ADC2VL) * 1000 + 0.5); //Multiply by 1000 for mV, round
  if(cellV > 9200) //switchover at 9.2V (9.46V max)
    cellV = (int)((((float)analogRead(AC1VH)) / ADC2VH) * 1000 + 0.5); //Multiply by 1000 for mV, round
  initbuckduty1 = (int)((float)cellV/(float)inputV*399);
  if(initbuckduty1 < MINBUCKDUTY)
    initbuckduty1 = MINBUCKDUTY;
  else if(initbuckduty1 > MAXBUCKDUTY)
    initbuckduty1 = MAXBUCKDUTY;
}

void pauseInts(void)
{
  Timer1.pause();
  Timer4.pause();
  Timer2.pause();
}

void resumeInts(void)
{
  Timer1.resume();
  Timer4.resume();
  Timer2.resume();
}

void setLED1(unsigned char color) {
/*#define OFF 0
#define RED 1
#define YELLOW 2
#define GREEN 3
#define CYAN 4
#define BLUE 5
#define PURPLE 6*/
  switch (color)
  {
    case LED_OFF:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
    case LED_RED:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,0);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
    case LED_YELLOW:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, LOW);
      break;
    case LED_GREEN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, LOW);
      break;
    case LED_CYAN:
      leds.setPixelColor(0,0,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, HIGH);
      //digitalWrite(LED1B, HIGH);
      break;
    case LED_BLUE:
      leds.setPixelColor(0,0,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    case LED_PURPLE:
      leds.setPixelColor(0,LED_BRIGHTNESS,0,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    case LED_WHITE:
      leds.setPixelColor(0,LED_BRIGHTNESS,LED_BRIGHTNESS,LED_BRIGHTNESS);
      leds.show();
      //digitalWrite(LED1R, HIGH);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, HIGH);
      break;
    default:
      leds.setPixelColor(0,0,0,0);
      leds.show();
      //digitalWrite(LED1R, LOW);
      //digitalWrite(LED1G, LOW);
      //digitalWrite(LED1B, LOW);
      break;
  }
}

void setup() {
  int i = 0;
  unsigned short int wData = 0, estatus = 0;
  
  setChg1(DISCONNECT);
  disableDebugPorts();
  Serial.begin(115200);
  pinMode(C1ON, OUTPUT);
  digitalWrite(C1ON, HIGH); //Default reverse/low voltage detect override off
  pinMode(AC1T, INPUT_ANALOG); //Temp1
  pinMode(AC1AL, INPUT_ANALOG); //Ibat low
  pinMode(AC1AH, INPUT_ANALOG); //Ibat hi
  pinMode(AC1VL, INPUT_ANALOG); //Vbat low
  pinMode(AC1VH, INPUT_ANALOG); //Vbat hi
  pinMode(AC1R, INPUT_ANALOG); //Vrev1
  pinMode(ABUFV, INPUT_ANALOG); //Vbuf
  pinMode(AHVV, INPUT_ANALOG); //Vhv
  pinMode(AIREF, INPUT_ANALOG); //VIref
  pinMode(FANON, OUTPUT);
  digitalWrite(FANON, LOW);
  adciref = analogRead(AIREF); //Iref voltage

  leds.begin();

  setLED1(LED_OFF);

  Timer1.pause();
  //timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer1.setPrescaleFactor(1);
  Timer1.setOverflow(400);
  Timer1.refresh();
  Timer1.resume();

  Timer4.pause();
  //timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer4.setPrescaleFactor(1);
  Timer4.setOverflow(400);
  Timer4.refresh();
  Timer4.resume();

  Timer2.pause(); // Pause the timer while we're configuring it
  Timer2.setPrescaleFactor(1);
  Timer2.setOverflow(36000); //72 MHz/36000 = 2000 Hz
  //Set up an interrupt on channel 1
  Timer2.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer2.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  Timer2.attachCompare1Interrupt(handler_loop);
  Timer2.refresh(); //Refresh the timer's count, prescale, and overflow
  delay(5000);
  //Timer2.resume(); //Start the timer counting

  #ifdef OLED_ENABLED
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  
    // init done
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    //display.display();
  #endif

  #ifdef HW_1_0
  Serial.print("HW Cfg: 1.0");
  #endif
  #ifdef OLED_ENABLED
  Serial.println("OLED Enabled");
  #endif

  estatus = EEPROM.init();
  if(estatus != 0)
  {
    Serial.print("> EEPROM init err: ");
    Serial.println(estatus, HEX);
  }
  
  Serial.println("> Init cal from EEPROM");
  estatus = EEPROM.read(REFAVALADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 856 || wData < 256) //ideal = 1.0556V
    {
      Serial.println("> Ref cal out of range, using default.");
    }
    else
    {
      Serial.print("> Ref cal value: 1.0");
      Serial.print(wData);
      Serial.println("V");
      REFAVAL = (10000.0+(float)wData)*124.1212;
    }
  }
  else
  {
    Serial.println("> Ref cal not found, using default.");
  }
  estatus = EEPROM.read(ADC2VLADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4600 || wData < 3800)
    {
      Serial.println("> L Vcal out of range, using default.");
    }
    else
    {
      Serial.print("> L Vcal value: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2VL = ADC2VL * 4200.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> L Vcal not found, using default.");
  }
  estatus = EEPROM.read(ADC2VHADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 33400 || wData < 25400)
    {
      Serial.println("> H Vcal out of range, using default.");
    }
    else
    {
      Serial.print("> H Vcal value: ");
      Serial.print(wData);
      Serial.println("mV");
      //Voltage reported / Voltage real * ADC2VxINIT orig = ADC2VxINIT new
      ADC2VH = ADC2VH * 29400.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> H Vcal not found, using default.");
  }
  estatus = EEPROM.read(ADC2ILADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 1200 || wData < 800)
    {
      Serial.println("> L Ical out of range, using default.");
    }
    else
    {
      Serial.print("> L Ical value: ");
      Serial.print(wData);
      Serial.println("mA");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2IL = ADC2IL * 1000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> L Ical not found, using default.");
  }
  estatus = EEPROM.read(ADC2IHADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 11000 || wData < 9000)
    {
      Serial.println("> H Ical out of range, using default.");
    }
    else
    {
      Serial.print("> H Ical value: ");
      Serial.print(wData);
      Serial.println("mA");
      //Current reported / Current real * ADC2IxINIT orig = ADC2IxINIT new
      ADC2IH = ADC2IH * 10000.0/((float)wData);
    }
  }
  else
  {
    Serial.println("> H Ical not found, using default.");
  }
  estatus = EEPROM.read(B12V2VADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 14000 || wData < 10000)
    {
      Serial.println("> 12V cal out of range, using default.");
    }
    else
    {
      Serial.print("> 12V cal value: ");
      Serial.print(wData);
      Serial.println("mV");
      B12V2V = (12000.0/(float)wData)*B12V2VINIT;
    }
  }
  else
  {
    Serial.println("> 12V cal not found, using default.");
  }
  estatus = EEPROM.read(BHVV2VADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 14000 || wData < 10000)
    {
      Serial.println("> HV cal out of range, using default.");
    }
    else
    {
      Serial.print("> HV cal value: ");
      Serial.print(wData);
      Serial.println("mV");
      BHVV2V = (12000.0/(float)wData)*BHVV2VINIT;
    }
  }
  else
  {
    Serial.println("> 12V cal not found, using default.");
  }
  estatus = EEPROM.read(ADC2IDLADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4467 || wData < 6467)
    {
      Serial.println("> ADCL div cal out of range, using default.");
    }
    else
    {
      Serial.print("> ADCL div cal value: 2.");
      Serial.println(wData);
      ADC2IDL = 2.0 + ((float)wData/10000.0);
    }
  }
  else
  {
    Serial.println("> ADCL div cal not found, using default.");
  }
  estatus = EEPROM.read(ADC2IDHADDR, &wData);
  if(estatus == 0)
  {
    if(wData > 4467 || wData < 6467)
    {
      Serial.println("> ADCL div cal out of range, using default.");
    }
    else
    {
      Serial.print("> ADCL div cal value: 2.");
      Serial.println(wData);
      ADC2IDH = 2.0 + ((float)wData/10000.0);
    }
  }
  else
  {
    Serial.println("> ADCL div cal not found, using default.");
  }

  Serial.println("> Init ref...");
  adciref = 0;
  for(i=0;i<1000;i++)
  {
    adciref += analogRead(AIREF); //Iref voltage
    //Serial.println(adciref);
  }
  Serial.print("> ADC corr: ");
  corr_factor = (REFAVAL/((float)adciref));
  Serial.println(corr_factor*100);
  ADC2IL = ADC2IL/corr_factor;
  ADC2IH = ADC2IH/corr_factor;
  ADC2VL = ADC2VL/corr_factor;
  ADC2VH = ADC2VH/corr_factor;
  
  Serial.print("> SW ver: ");
  Serial.println(vers);
  adciref = (uint32)((float)adciref/1000.0); //Iref voltage
  
  printMenu(mode1);
  
  Timer2.resume(); //Start the timer counting
}

void updateADCRef(){
  int i = 0;
  adciref = 0;
  for(i=0;i<1000;i++)
  {
    adciref += analogRead(AIREF); //Iref voltage
  }
  corr_factor = (REFAVAL/((float)adciref));
  ADC2IL = ADC2IL/corr_factor;
  ADC2IH = ADC2IH/corr_factor;
  ADC2VL = ADC2VL/corr_factor;
  ADC2VH = ADC2VH/corr_factor;
  adciref = (uint32)((float)adciref/1000.0); //Iref voltage
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarkers[] = {'c', 'd', 'y', 'p', 't', 'n', '?', 'v', 's', 'l', 'r', 'z', 'q', 'a'};
  char endMarkers[] = {'\n', '\r'};
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc == '\b')
      Serial.print("\b \b");
    else
      Serial.print(rc);
        
    if (recvInProgress == true) {
      //Serial.println("Receive in progress");
      if (rc == endMarkers[0] || rc == endMarkers[1]){
        //Serial.println("Received endMarker");
        receivedChars[ndx] = '\0'; // terminate the string
        /*Serial.print("ndx=");
          Serial.println(ndx);
          Serial.print("rc=");
          Serial.print(rc);*/
        recvInProgress = false;
        ndx = 0;
        newData = true;
      } 
      else if (rc == '\b') { //Backspace = do not increment character index, do not write new character
        /*Serial.println("Received BS");
        Serial.print("ndx=");
        Serial.println(ndx);
        Serial.print("rc=");
        Serial.print(rc);*/
        if (ndx == 0) {
          ndx = 0;
        }
        else {
          //receivedChars[ndx] = 0;
          ndx--;
        }
      }
      else {
        /*Serial.println("Received normal");
          Serial.print("ndx=");
          Serial.println(ndx);
          Serial.print("rc=");
          Serial.print(rc);*/
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= MAXCHARS) {
          ndx = MAXCHARS - 1;
        }
      }
    }

    else if (rc == startMarkers[0] || rc == startMarkers[1] || rc == startMarkers[2] || rc == startMarkers[3] || rc == startMarkers[9] || rc == startMarkers[11] || rc == startMarkers[13]
             || rc == startMarkers[4] || rc == startMarkers[5] || rc == startMarkers[6] || rc == startMarkers[7] || rc == startMarkers[8] || rc == startMarkers[10] || rc == startMarkers[12]) {
      /*Serial.println("Received startMarker");
        Serial.print("ndx=");
        Serial.println(ndx);
        Serial.print("rc=");
        Serial.print(rc);*/
      recvInProgress = true;
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAXCHARS) {
        ndx = MAXCHARS - 1;
      }
    }

    else if (rc == endMarkers[0] || rc == endMarkers[1]) {
      Serial.print("\r\n");
      Serial.print("> ");
    }
  }
}

unsigned int fast_atoi_leading_pos( const char * p )
{
    unsigned int x = 0;
    ++p;
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    return x;
}

int fast_atoi_leading_neg( const char * p )
{
    int x = 0;
    bool neg = false;
    ++p;
    if (*p == '-') {
        neg = true;
        ++p;
    }
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    if (neg) {
        x = -x;
    }
    return x;
}

void printMenu(uint8 menu2) 
{
  switch (menu2) {
    case 0:
      Serial.print("\r\n");
      Serial.println("> Charging");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 1:
      Serial.print("\r\n");
      Serial.println("> Discharging");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 2:
      Serial.print("\r\n");
      Serial.println("> Cycle Testing");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 3:
      Serial.print("\r\n");
      Serial.println("> PSU Mode");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 4:
      Serial.print("\r\n");
      Serial.println("> Test Mode");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    case 6:
      Serial.print("\r\n");
      Serial.println("> IR Test");
      Serial.println(">  Press n to end");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
    default:
      //'c', 'd', 'y', 'p', 't', 'n', '?', 'v', 's', 'l', 'r', 'z', 'q', 'a'
      Serial.print("\r\n");
      Serial.println("> Select Mode:");
      Serial.println(">  Charge");
      Serial.println(">   c[1-2] i[charge current, mA] v[charge voltage, mV] o[cutoff current, mA] n[cell type: 0 = Li, 1 = Ni]");
      Serial.println(">            100-1500, def.1500    2400-4500, def.4200   50-250, def.50        def.0");
      Serial.println(">  Discharge");
      Serial.println(">   d[1-2] i[dis current, mA] v[cutoff voltage, mV] m[mode: 0 = constant, 1 = stepped]");
      Serial.println(">            100-1500, def.1500 700-3900, def.2700    def.0");
      Serial.println(">          r[dir: 0 = resistive, 2 = regen]");
      Serial.println(">            def.0");
      Serial.println(">  Cycle");
      Serial.println(">   y[1-2] i[dis current, mA] v[cutoff voltage, mV] m[mode: 0 = constant, 1 = stepped]");
      Serial.println(">            100-1500, def.1500 700-3900, def.2700    def.0");
      Serial.println(">          k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]");
      Serial.println(">            100-1500, def.1500    2400-4500, def.4200   50-250, def.50        def.1");
      Serial.println(">          r[dir: 0 = resistive, 2 = regen] n[cell type: 0 = Li, 1 = Ni]");
      Serial.println(">            def.0                            def.0");
      Serial.println(">  Power Supply");
      Serial.println(">   p[1-2] r[dir: 0 = resistive, 1 = buck, 2 = regen] v[voltage setting, mV] i[current limit, mA]");
      Serial.println(">            def.1                                      0-9500, def.4200       50-1500, def.1500");
      Serial.println(">          o[cutoff current, mA]");
      Serial.println(">           50-250, def.50");
      Serial.println(">  Cal R/W Mode");
      Serial.println(">   t[r/w] a[address: 0-999] d[data, unsigned int (0-65535)]");
      Serial.println(">  IR Test Mode");
      Serial.println(">   r[1-2] i[test current, mA] r[direction: 0 = resistive, 2 = regen]");
      Serial.println(">            100-1500, def.1500  def.0");
      Serial.println(">  Test Mode (raw PWM duty)");
      Serial.println(">   q[1-2] l[duty cycle (0-399)] r[direction: 0 = resistive, 2 = regen]");
      Serial.println(">            0-399                 def.0");
      Serial.println(">  Help (Prints this menu)");
      Serial.println(">   ?");
      Serial.println(">  Stop current mode/test");
      Serial.println(">   n[1-2]");
      Serial.println(">  Version");
      Serial.println(">   v");
      Serial.println(">  Soft Reset");
      Serial.println(">   z");
      Serial.println(">  OLED En/Disable (Toggle)");
      Serial.println(">   a");
      Serial.println(">  Status");
      Serial.println(">   s");
      Serial.print("\r\n");
      Serial.print("> ");
      break;
  }
}

void parseCharge1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;
  cell1_type = DEF_CELL_TYPE;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_1 = strVal;
          Serial.print(charge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell1_type = strVal;
          if(cell1_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
  if(cell1_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell1_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
  }
}

void parseDischarge1(uint8 nArgs, char* args[])
{
  uint8 i;
  uint16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_1 = DEF_DIS_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  discharge_mode_1 = DEF_DIS_MODE;
  psu_dir_1 = DEF_DIS_MODE;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_1 = strVal;
          Serial.print(discharge_voltage_1);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_1 = strVal;
          if(discharge_mode_1 == 0)
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
  }
}

void parseIR1(uint8 nArgs, char* args[])
{
  uint8 i;
  uint16 strVal = 0;
  
  //Set default discharge current
  discharge_current_1 = DEF_DIS_CUR;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else
            Serial.println("Regenerative");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
}

void parsePSU1(uint8 nArgs, char* args[])
{
  //Power Supply
  // p r[direction: 0 = boost, 1 = buck] v[voltage setting, mV] i[current limit, mA]
  //     default = 1                      default = 4200        default = 1500
  uint8 i;
  uint16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  discharge_voltage_1 = DEF_CHG_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  psu_dir_1 = DEF_PSU_MODE;
  ccc_1 = DEF_CCC;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<5)
            strVal = 5;
          discharge_current_1 = strVal;
          charge_current_1 = -1*strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<100)
            strVal = 100;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_1 = strVal;
          discharge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'r':
          Serial.print("> Using dir: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Discharge");
          else if(psu_dir_1 == 1)
            Serial.println("Buck");
          else if(psu_dir_1 == 2)
            Serial.println("Boost");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    Serial.println("Buck");
    if(charge_voltage_1 == DEF_CHG_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(charge_voltage_1);
      Serial.println("mV");
    }
  }
  else
  {
    if(discharge_voltage_1 == DEF_DIS_VOL)
    {
      Serial.print("> Using def voltage: ");
      Serial.print(discharge_voltage_1);
      Serial.println("mV");
    }
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
}

void parseTM1(uint8 nArgs, char* args[])
{
  //Test Mode
  // t r[direction: 0 = boost, 1 = buck] l[duty cycle (0-199)]
  //     default = 1                      default = 0 (boost), 199 (buck)
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  psu_dir_1 = DEF_PSU_MODE;
  tm_duty_1 = 399;

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'l':
          Serial.print("> Using duty cycle: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>399)
            strVal = 399;
          else if(strVal<0)
            strVal = 0;
          tm_duty_1 = strVal;
          Serial.println(tm_duty_1);
          break;
        case 'r':
          Serial.print("> Using dir: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Boost");
          else
            Serial.println("Buck");
          break;
        default:
          break;
      }
    }
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
    {
      Serial.println("Boost");
      if(discharge_voltage_1 == DEF_DIS_VOL)
      {
        Serial.print("> Using def voltage: ");
        Serial.print(discharge_voltage_1);
        Serial.println("mV");
      }
    }
    else
    {
      Serial.println("Buck");
      if(charge_voltage_1 == DEF_CHG_VOL)
      {
        Serial.print("> Using def voltage: ");
        Serial.print(charge_voltage_1);
        Serial.println("mV");
      }
    }
  }
}

void parseCycle1(uint8 nArgs, char* args[])
{
  uint8 i;
  int16 strVal = 0;
  
  //Set default discharge current/voltage/mode
  discharge_voltage_1 = DEF_DIS_VOL;
  discharge_current_1 = DEF_DIS_CUR;
  discharge_mode_1 = DEF_DIS_MODE;
  psu_dir_1 = DEF_PSU_MODE;
  //Set default charge current/voltage/cutoff
  charge_voltage_1 = DEF_CHG_VOL;
  charge_current_1 = DEF_CHG_CUR;
  ccc_1 = DEF_CCC;
  num_cycles_1 = DEF_CYCLES;
  cell1_type = DEF_CELL_TYPE;
  
  //Cycle
  // y i[discharge current, mA] v[cutoff voltage, mV] m[mode: 0 = constant current, 1 = stepped]
  //     default = 1500          default = 2700       default = 0
  //   k[charge current, mA] u[charge voltage, mV] o[cutoff current, mA] l[number of cycles]
  //     default = 1500       default = 4200       default = 50         default = 1

  Serial.print("\r\n");
  if(nArgs>1)
  {
    for(i=1; i<nArgs; i++)
    {
      switch (args[i][0])
      {
        case 'i':
          Serial.print("> Using current: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>MAX_DIS_CUR)
            strVal = MAX_DIS_CUR;
          else if(strVal<100)
            strVal = 100;
          discharge_current_1 = strVal;
          Serial.print(discharge_current_1);
          Serial.println("mA");
          break;
        case 'v':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_DIS_VOL)
            strVal = MIN_DIS_VOL;
          else if(strVal>MAX_DIS_VOL)
            strVal = MAX_DIS_VOL;
          discharge_voltage_1 = strVal;
          Serial.print(discharge_voltage_1);
          Serial.println("mV");
          break;
        case 'm':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal>1)
            strVal = 1;
          else if(strVal<0)
            strVal = 0;
          discharge_mode_1 = strVal;
          if(discharge_mode_1 == 0)
            Serial.println("Constant");
          else
            Serial.println("Stepped");
          break;
        case 'k':
          Serial.print("> Using current: ");
          strVal = -1*fast_atoi_leading_pos(args[i]);
          if(strVal<MAX_CHG_CUR)
            strVal = MAX_CHG_CUR;
          else if(strVal>-100)
            strVal = -100;
          charge_current_1 = strVal;
          Serial.print(charge_current_1);
          Serial.println("mA");
          break;
        case 'u':
          Serial.print("> Using voltage: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CHG_VOL)
            strVal = MIN_CHG_VOL;
          else if(strVal>MAX_CHG_VOL)
            strVal = MAX_CHG_VOL;
          charge_voltage_1 = strVal;
          Serial.print(charge_voltage_1);
          Serial.println("mV");
          break;
        case 'o':
          Serial.print("> Using cutoff: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<MIN_CCC)
            strVal = MIN_CCC;
          else if(strVal>1000)
            strVal = 1000;
          ccc_1 = strVal;
          Serial.print(ccc_1);
          Serial.println("mA");
          break;
        case 'l':
          Serial.print("> Using cycles: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<1)
            strVal = 1;
          else if(strVal>10000)
            strVal = 10000;
          num_cycles_1 = strVal;
          Serial.println(num_cycles_1);
          break;
        case 'r':
          Serial.print("> Using mode: ");
          strVal = fast_atoi_leading_pos(args[i]);
          #ifdef REGEN_ENABLED
            if(strVal>1)
              strVal = 2;
          #else
            if(strVal>1)
              strVal = 1;
          #endif
          else if(strVal<0)
            strVal = 0;
          psu_dir_1 = strVal;
          if(psu_dir_1 == 0)
            Serial.println("Resistive");
          else if(psu_dir_1 == 2)
            Serial.println("Regen");
          break;
        case 'n':
          Serial.print("> Using cell type: ");
          strVal = fast_atoi_leading_pos(args[i]);
          if(strVal<0)
            strVal = 0;
          else if(strVal>1)
            strVal = 1;
          cell1_type = strVal;
          if(cell1_type == 0)
            Serial.println("Li*");
          else
            Serial.println("Ni*");
          break;
        default:
          break;
      }
    }
  }
  
  if(discharge_voltage_1 == DEF_DIS_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(discharge_voltage_1);
    Serial.println("mV");
  }
  if(discharge_current_1 == DEF_DIS_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(discharge_current_1);
    Serial.println("mA");
  }
  if(discharge_mode_1 == DEF_DIS_MODE)
  {
    Serial.print("> Using def mode: ");
    if(discharge_mode_1 == 0)
      Serial.println("Constant");
    else
      Serial.println("Stepped");
  }
  if(psu_dir_1 == DEF_PSU_MODE)
  {
    Serial.print("> Using def mode: ");
    if(psu_dir_1 == 0)
    {
      Serial.println("Resistive");
    }
    else if(psu_dir_1 == 2)
    {
      Serial.println("Regen");
    }
  }
  if(charge_voltage_1 == DEF_CHG_VOL)
  {
    Serial.print("> Using def voltage: ");
    Serial.print(charge_voltage_1);
    Serial.println("mV");
  }
  if(charge_current_1 == DEF_CHG_CUR)
  {
    Serial.print("> Using def current: ");
    Serial.print(charge_current_1);
    Serial.println("mA");
  }
  if(ccc_1 == DEF_CCC)
  {
    Serial.print("> Using def cutoff: ");
    Serial.print(ccc_1);
    Serial.println("mA");
  }
  if(num_cycles_1 == DEF_CYCLES)
  {
    Serial.print("> Using def cycles: ");
    Serial.println(num_cycles_1);
  }
  if(cell1_type == DEF_CELL_TYPE)
  {
    Serial.print("> Using def cell type: ");
    if(cell1_type == 0)
      Serial.println("Li*");
    else
      Serial.println("Ni*");
  }
}

void runStateMachine(void)
{
  loop1++;

  //digitalWrite(PB12, HIGH);
  //2kHz loop:
  //Cell 1 - Ibat, Vbat, CC/CV control loops
  adcval1 = analogRead(AC1AL); //Ibat raw
  //Convert ADC value to value in Amps
  tmpfl = (((float)adcval1*ADC2IDL) - (float)adciref) / ADC2IL; //ADC to I = (ADC-IREF)/(4096b)*(3.3Vmax)/(0.5V/A)
  ibat1 += tmpfl; //Accumulate 2000 values for average (result = mA*2)
  ibat_now1 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mA, round

  adcval2 = analogRead(AC1VL); //adc_read(ADC, 8);
  if(adcval2 > 3980) //Voltage > 9.2V, switch to high range
  {
    adcval2 = analogRead(AC1VH);
    //Convert ADC value to value in Volts
    tmpfl = ((float)adcval2) / ADC2VH; //ADC to V = (ADC)/(4096b)*(3.3Vmax)*220/150
  }
  else
  {
    //Convert ADC value to value in Volts
    tmpfl = ((float)adcval2) / ADC2VL; //ADC to V = (ADC)/(4096b)*(3.3Vmax)*220/150
  }
  vbat1 += tmpfl; //Accumulate 2000 values for average (result = mV*2)
  vbat_now1 = (int)(tmpfl * 1000 + 0.5); //Multiply by 1000 for mV, round

  //Constant current/voltage control loops
  if ((state1 == 3) || (state1 == 4)) //Charging states
  {
    if (ibat_now1 > charge_current_1) //Ibat < 1.5A
    {
      if (vbat_now1 < charge_voltage_1) //Vbat < 4.2V, Ibat < 1.5A
      {
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
      }
      else //Vbat >= 4.2V, Ibat < 1.5A
      {
          duty1--;
          if (duty1 < MINBUCKDUTY)
            duty1 = MINBUCKDUTY;
      }
    }
    else //Ibat >= 1.5A
    {
        duty1--;
        if (duty1 < MINBUCKDUTY)
          duty1 = MINBUCKDUTY;
    }
    pwmWrite(OC1PF, duty1);
    pwmWrite(OC1ON, duty1);
  }
  if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
  {
    if(psu_dir_1 < 2)
    {
      if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
      {
        if (vbat_now1 > (discharge_voltage_1 - 100)) //Vbat > 2.7V, Ibat < 1.5A
        {
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        }
        else //Vbat <= 2.7V, Ibat < 1.5A
        {
          duty1--;
          if (duty1 < MINBUCKDUTY)
            duty1 = MINBUCKDUTY;
        }
      }
      else //Ibat >= 1.5A
      {
        duty1--;
        if (duty1 < MINBUCKDUTY)
          duty1 = MINBUCKDUTY;
      }
      pwmWrite(OC1NF2, duty1); //CC Load
    }
    else
    {
      if (ibat_now1 < discharge_current_1) //Ibat < 1.5A
      {
        if (vbat_now1 > (discharge_voltage_1 - 100)) //Vbat > 2.7V, Ibat < 1.5A
        { 
          duty1--;
          if (duty1 < MINBUCKDUTY)
            duty1 = MINBUCKDUTY;
        }
        else //Vbat <= 2.7V, Ibat < 1.5A
        {
          duty1++;
          if (duty1 > MAXBUCKDUTY)
            duty1 = MAXBUCKDUTY;
        }
      }
      else //Ibat >= 1.5A
      {
        duty1++;
        if (duty1 > MAXBUCKDUTY)
          duty1 = MAXBUCKDUTY;
      }
      pwmWrite(OC1PF, duty1);
      pwmWrite(OC1ON, duty1);
    }
  }
  
  //500Hz loop - Cell 1 IR
  if ((loop1 % 4) == 0)
  {
    //digitalWrite(PB12, HIGH);
    if (irstate1 == 2)
    {
      //Iload in mA/8 (mA*2 * (500/2000) * 0.25)
      iload1 = iload1 + ((float)adcval1 - adciref) / ADC2IL; //Accumulate current samples from 250-500ms
      //Vload in mV/8 (mV*2 * (500/2000) * 0.25)
      vload1 = vload1 + ((float)adcval2) / ADC2VL; //Accumulate voltage samples from 250-500ms
    }
    else if (irstate1 == 3)
    {
      //Voc2 in mV/8 (mV*2 * (500/2000) * 0.25)
      voc21 = voc21 + ((float)adcval2) / ADC2VL;
    }

    if ((irstate1 == 1) && (loop1 == 500)) //Get Voc1 from first 250ms of sampling, then turn on load at 250ms
    {
      voc11 = vbat1; //Vbat in mV/2 (mV*2 * (500/2000))
      iload1 = 0;
      vload1 = 0;
        if(psu_dir_1 < 2)
        {
          setChg1(DISCHARGE);
        }
        #ifdef REGEN_ENABLED
        else
        {
          setChg1(CHARGE);
        }
        #endif
      irstate1 = 2;
    }
    if ((irstate1 == 2) && (loop1 == 1000)) //Get Vload, Iload from 250-500ms of sampling, then turn off load at 500ms
    {
      voc21 = 0;
      setChg1(DISCONNECT);
      irstate1 = 3;
    }
    if ((irstate1 == 3) && (loop1 == 1500)) //Get Voc2 from 500-750ms of sampling
    {
      //     mV/2 + mV/8*4    -  mV/8*4*2    /  mA/8  / 125
      //         mV           -  mV          /  A = mOhms
      ir1 = ((voc11 + voc21 * 4.0) - (vload1 * 8.0)) / (iload1 / 125.0);
      //Msg type 3 (IR): 3,4126.45,mV,4123.15,mOhms,1259.21,mA,4053.12,mV,56.92,mOhms, 7,1
      //(IR Complete,3,Voc1,mV,Voc2,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
      Serial.print("3,");
      Serial.print(voc11 * 2.0);
      Serial.print(",");
      Serial.print(voc21 * 8.0);
      Serial.print(",");
      Serial.print(iload1 * 8.0);
      Serial.print(",");
      Serial.print(vload1 * 8.0);
      Serial.print(",");
      Serial.print(ir1);
      Serial.print(",");
      Serial.println(state1);
      //Serial.print(",");
      //Serial.println(settle1);
      irstate1 = 0;
    }

    //1Hz loop: Periodic status printing, temp measurement
    if (loop1 == 2000)
    {
      //Serial.print("Int cnt: ");
      //Serial.println(interruptcnt);
      //interruptcnt++;
      //digitalWrite(PB12,HIGH);
      loop1 = 1;
      
      //mA*2 /2 /3600 = mAH
      mah1 = mah1 + ibat1 / 7200.0;
      mwh1 = mwh1 + vbat1 / 2000.0 * ibat1 / 7200.0;
      vbat_1_1 = vbat1 / 2.0;
      ibat_1_1 = ibat1 / 2.0;    
      
      adcval3 = analogRead(AC1T);
      //Code for 2.495V sourcing 10k NTC divider
      if (adcval3 > 1269) //0-35C, use linear estimation
      {
        temp1 = ((float)adcval3) / -28.82 + 78.816 + TOFFS1;
      }
      else //35C+, use polynomial estimation
      {
        temp1 = (((float)adcval3) * ((float)adcval3) * ((float)adcval3) / -20005929.2 + ((float)adcval3) * ((float)adcval3) / 6379.75 + ((float)adcval3) / -4.866 + 144.9107) + TOFFS1;
      }
      //Code for 3.3V sourcing 10k NTC divider
      /*if (adcval3 > 1900) //0-28C, use linear estimation
      {
        temp1 = ((float)adcval3) / -38.61 + 78.055 + TOFFS1;
      }
      else //35C+, use polynomial estimation
      {
        temp1 = (((float)adcval3)*((float)adcval3)*((float)adcval3)/-181172413.8+((float)adcval3)*((float)adcval3)/31672.3+((float)adcval3)/-11.3881+119.565) + TOFFS1;
      }*/
      if(((int)temp1 > OVT_THRESH) && (state1 != 8))
      {
        setChg1(DISCONNECT);
        setLED1(LED_RED);
        state1 = 8;
        settle1 = 0;
        mode1 = 5;
        //Timer1.pause();
        Serial.println("> Cell 1 OVT, stopping");
        Serial.print("> Cell 1 Temp: ");
        Serial.println(temp1);
      }
      if(((int)vbat_1_1 > OVV_THRESH) && (state1 != 8))
      {
        setChg1(DISCONNECT);
        setLED1(LED_RED);
        state1 = 8;
        settle1 = 0;
        mode1 = 5;
        //Timer1.pause();
        Serial.println("> Cell 1 OVV, stopping");
        Serial.print("> Cell 1 Voltage: ");
        Serial.println(vbat_1_1);
      }
      
      //78.125us/char at 115200
      if (state1 != 8)
      {
        //Msg type 0 (Periodic Status): 0,3754.12,mV,-1453.98,mA,-750.19,mAH,-2810.34,mWH,23.5,C,4,358
        //(Periodic Status,0,Vbat,mV,Ibat,mA,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
        Serial.print("0,");
        Serial.print(vbat_1_1);
        Serial.print(",");
        Serial.print(ibat_1_1);
        Serial.print(",");
        Serial.print(mah1);
        Serial.print(",");
        //vbuf_i = (int)(((float)getAuxADC(BUFV)) * BUF2V);
        //Serial.print(vbuf_i); //2.417);
        Serial.print(mwh1);
        Serial.print(",");
        Serial.print(temp1);
        Serial.print(",");
        Serial.println(state1);
        //Serial.print(",");
        //Serial.println(settle1);
      }
      /* State machine:
          0. Battery Disconnected (Unused)
          1. Battery Discharge
          Voltage < 2.75V?
          2. Battery Disconnected
          Wait 5 minutes
          3. Battery Charge
          Current < 50mA?
          4. Battery Disconnect
          5. Wait 1 minute
          6. Measure IR
          7. Measure IR
          Wait 10 seconds
          8. Parking
          1. Battery Discharge (goto top)
      */
      /* Mode List:
          0 = c = Charge
          1 = d = Discharge
          2 = y = Cycle
          3 = p = Power Supply
          4 = t = Test
      */
      switch (state1)
      {
        case 0: //Default state; determine to charge or discharge based on current SoC
          //Serial.println("Case 0");
          setChg1(DISCONNECT);
          vbati1 = (int)(vbat_1_1);
          if (vbati1 >= 3700) //Above mid-charge state
          {
            state1 = 1;
            setChg1(DISCHARGE);
          }
          else
            state1 = 3;
          break;
        case 1: //Discharge state; check for LVC, goto disconnect state if triggered
          //Serial.println("Case 1");
          vbati1 = (int)(vbat_1_1);
          if ((vbati1 <= discharge_voltage_1) && (mode1 != 3)) //Discharged at 2.75V
          {
            state1 = 2;
            settle1 = 0;
            setLED1(LED_PURPLE);
          }
          else if ((ibat_1_1 <= ccc_1) && (mode1 == 3) && (settle1 > 10)) //Full battery = <50mA CC
          {
            state1 = 2;
            settle1 = 0;
            setLED1(LED_PURPLE);
          }
          break;
        case 2: //Disconnect/settling state after discharge, wait 5 minutes before charging
          //Serial.println("Case 2");
          setChg1(DISCONNECT);
          if (settle1 > AFTERDISWAIT)
          {
            if(mode1 == 1)
            {
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              setLED1(LED_OFF);
            }
            else
            {
              state1 = 3;
              settle1 = 0;
              setChg1(CHARGE);
              setLED1(LED_CYAN);
            }
            //Msg type 1 (Discharged): 0,2754.12,mV,32.57,mOhms,893.21,mAH,3295.12,mWH,25.1,C,3,1
            //(Periodic Status,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("1,");
            Serial.print(vbat_1_1);
            Serial.print(",");
            Serial.print(ir1);
            Serial.print(",");
            Serial.print(mah1);
            Serial.print(",");
            Serial.print(mwh1);
            Serial.print(",");
            Serial.print(temp1);
            Serial.print(",");
            Serial.println(state1);
            //Serial.print(",");
            //Serial.println(settle1);
            mah1 = 0;
            mwh1 = 0;
          }
          break;
        case 3: //Charging state (CC)
          //Serial.println("Case 3");
          if (settle1 > CHGSETTLEWAIT) //Wait 30s for current to settle
          {
            state1 = 4;
            settle1 = 0;
            cell1_vmax = 0;
          }
          break;
        case 4: //Charging state (CC/CV)
          //Serial.println("Case 4");
          ibati1 = (int)(ibat_1_1 * -1.0);
          vbati1 = (int)(vbat_1_1);
          if(vbati1 > cell1_vmax)
            cell1_vmax = vbati1;
          if(cell1_type == 0)
          {
            if ((ibati1 <= ccc_1) && (mode1 != 3)) //Full battery = <50mA CC
            {
              ltcc_count++;
              if(ltcc_count >= LTCC_THRES) //NEW - check for 2nd consecutive CC threshold hit
              {
                setChg1(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
                state1 = 5;
                settle1 = 0;
                setLED1(LED_PURPLE);
                ltcc_count = 0;
              }
            }
            else
              ltcc_count = 0;
          }
          else
          {
            if (((vbati1 <= (cell1_vmax - NIMH_DV)) && settle1 > 600) || (vbati1 >= 1800)) //Full battery = -10mV dV or >1.8V, 10 minute delay until detection
            {
              setChg1(DISCONNECT); //Disconnect/settling state after charge, wait 10 minutes before discharging
              state1 = 5;
              settle1 = 0;
              setLED1(LED_PURPLE);
            }
          }
          break;
        case 5:
          //Serial.println("Case 5");
          if (settle1 > AFTERCHGWAIT) //1 Minutes to settle after charging
          {
            state1 = 6;
            settle1 = 0;
          }
          break;
        case 6: //IR measure state
          //Serial.println("Case 6");
          irstate1 = 1;
          state1 = 7;
          break;
        case 7: //IR measure state
          //Serial.println("Case 7");
          if (settle1 > 2)
          {
            if(mode1 == 0 || mode1 == 6) //Finished charge cycle, reset+park state
            {
              setChg1(DISCONNECT);
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              setLED1(LED_OFF);
            }
            else
            {
              cycle_count_1++;
              if(cycle_count_1 == (num_cycles_1 + 1))//1 more cycle for initial charge
              {
                setChg1(DISCONNECT);
                state1 = 8;
                settle1 = 0;
                mode1 = 5;
                setLED1(LED_OFF);
              }
              else
              {
                #ifdef HW_1_0
                  if(psu_dir_1 == 2)
                  {
                    state1 = 1;
                    settle1 = 0;
                    #ifdef REGEN_ENABLED
                      setChg1(CHARGE);
                    #endif
                    setLED1(LED_YELLOW);
                  }
                  else
                  {
                    state1 = 1;
                    settle1 = 0;
                    setLED1(LED_YELLOW);
                    setChg1(DISCHARGE);
                  }
                #endif
              }
            }
            //Msg type 2 (Charged): 0,4126.45,mV,32.57,mOhms,-750.19,mAH,-2810.34,mWH,23.5,C,6,1
            //(Charged,0,Vbat,mV,IR,mOhms,Capacity,mAH,Capacity,mWH,Temp,C,State,Time)
            Serial.print("2,");
            Serial.print(vbat_1_1);
            Serial.print(",");
            Serial.print(ir1);
            Serial.print(",");
            Serial.print(mah1);
            Serial.print(",");
            Serial.print(mwh1);
            Serial.print(",");
            Serial.print(temp1);
            Serial.print(",");
            Serial.println(state1);
            //Serial.print(",");
            //Serial.println(settle1);
            mah1 = 0;
            mwh1 = 0;
          }
          break;
        case 8: //Parking state
          setChg1(DISCONNECT);
          break;
        case 9: //Test state
          //setChg1(DISCONNECT);
          break;
        default:
          setChg1(DISCONNECT);
          state1 = 8;
          settle1 = 0;
          break;
      }
      settle1++;
      if (settle1 > 32000)
        settle1 = 32000;
      ibat1 = 0;
      vbat1 = 0;
      loopcnt = 0;
      if((state1 != 8))
      {
        adciref = analogRead(AIREF); //Iref voltage
        vhv_i = (int)(((float)getAuxADC(HVV)) * BHVV2V);
        vbuf_i = (int)(((float)getAuxADC(BUFV)) * B12V2V);
        if(vbuf_i > MAX_VBUF)
        {
          if ((state1 == 1) || (state1 == 6) || (state1 == 7)) //Discharging states
          {
            if(psu_dir_1 >= 2)
            {
              setChg1(DISCONNECT);
              setLED1(LED_RED);
              state1 = 8;
              settle1 = 0;
              mode1 = 5;
              Serial.println("> Vbuf OV, stop C1 regen");
            }
          }
        }
        //Msg type 4 (Debug):
        //(Debug,4,Vbuf,mV,Vrev,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
        Serial.print("4,");
        Serial.print(vhv_i); //2.417);
        Serial.print(",");
        Serial.print(vbuf_i); //2.417);
        adctemp = getAuxADC(HSTH1);
        if (adctemp > 1269) //0-35C, use linear estimation
        {
          temp_t = ((float)adctemp) / -28.82 + 78.816 + TOFFS2;
        }
        else //35C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -20005929.2 + ((float)adctemp) * ((float)adctemp) / 6379.75 + ((float)adctemp) / -4.866 + 144.9107) + TOFFS2;
        }
        Serial.print(",");
        Serial.println(temp_t);//2.417);
      }
    }
  }
    
}

void loop() {

  /* Mode List:
      0 = c = Charge
      1 = d = Discharge
      2 = y = Cycle
      3 = p = Power Supply
      4 = t = Test
          v = Version
          ? = Help
          s = Status
  */
  uint8 i = 0;
  uint16 estatus = 0, wAddress = 0, wData = 0;
  uint16 cellV = 4200;
  char *args[8];
  recvWithStartEndMarkers();
  if (newData == true)
  {
    args[i] = strtok(receivedChars, " ");
    while (args[i] != NULL) //i = number of arguments in received string
    {
      args[++i] = strtok(NULL, " ");
    }
    switch (args[0][0]) {
      case 'c':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1VL)) / ADC2VL) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling charge");
              break;
            }
            mode1 = 0;
            parseCharge1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(LED_CYAN);
            printMenu(mode1);
          }
        }
        break;
      case 'd':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 1;
            parseDischarge1(i, args);
            state1 = 1;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
              if(psu_dir_1 == 0) //Resistive Mode
              {
                setChg1(DISCHARGE);
              }
              #ifdef REGEN_ENABLED
                else if(psu_dir_1 == 2) //Regenerative Mode
                {
                  setChg1(CHARGE);
                }
              #endif
            setLED1(LED_YELLOW);
            printMenu(mode1);
          }
        }
        break;
      case 'r':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            mode1 = 6;
            parseIR1(i, args);
            state1 = 6;
            settle1 = 0;
            ir1 = 0;
            //Timer2.resume(); //Start the timer counting
            setLED1(LED_PURPLE);
            printMenu(mode1);
          }
        }
        break;
      case 'y':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1VL)) / ADC2VL) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling charge");
              break;
            }
            mode1 = 2;
            parseCycle1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < charge set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            cycle_count_1 = 0;
            state1 = 3;
            settle1 = 0;
            mah1 = 0;
            mwh1 = 0;
            //Timer2.resume(); //Start the timer counting
            setChg1(CHARGE);
            setLED1(LED_CYAN);
            printMenu(mode1);
          }
        }
        break;
      case 'p':
        if(args[0][1] == '1')
        {
          if(getCell1RV() > 100)
          {
            setLED1(LED_RED);
            Serial.println("> Reverse polarity cell, cancelling");
            Serial.print("> Cell 1 voltage: ");
            Serial.println(getCell1RV());
          }
          else
          {
            cellV = (int)((((float)analogRead(AC1VL)) / ADC2VL) * 1000 + 0.5); //Multiply by 1000 for mV, round
            if(cellV > MAX_CHG_VOL)
            {
              Serial.print("> Max input exceeded on slot 1, cancelling PSU");
              break;
            }
            mode1 = 3;
            parsePSU1(i, args);
            if(charge_voltage_1 < cellV)
            {
              Serial.print("> Cell V < PSU set., increasing to ");
              charge_voltage_1 = cellV + 100;
              if(charge_voltage_1 > MAX_CHG_VOL)
                charge_voltage_1 = MAX_CHG_VOL;
              Serial.print(charge_voltage_1);
              Serial.println("mV");
            }
            if(psu_dir_1 == 1) //Buck aka charge mode
            {
              state1 = 3;
              settle1 = 0;
              mah1 = 0;
              mwh1 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg1(CHARGE);
              setLED1(LED_CYAN);
            }
            else if(psu_dir_1 == 0)
            {
              state1 = 1;
              settle1 = 0;
              mah1 = 0;
              mwh1 = 0;
              //Timer2.resume(); //Start the timer counting
              setChg1(DISCHARGE);
              setLED1(LED_YELLOW);
            }
            #ifdef REGEN_ENABLED
              else if(psu_dir_1 == 2)
              {
                mode1 = 1;
                state1 = 1;
                settle1 = 0;
                mah1 = 0;
                mwh1 = 0;
                //Timer2.resume(); //Start the timer counting
                setChg1(CHARGE);
                setLED1(LED_YELLOW);
              }
            #endif
            printMenu(mode1);
          }
        }
        break;
      case 't':
        if(args[0][1] == 'w')
        {
          mode1 = 6;
          Serial.println("\r\n> Writing EEPROM: ");
          //aXXX = address in dec.
          wAddress = fast_atoi_leading_pos(args[1]);
          //dXXX = data in dec.
          wData = fast_atoi_leading_pos(args[2]);
          Serial.print("> Address 0x");
          Serial.print(wAddress, HEX);
          Serial.print(", Data = dec. ");
          Serial.println(wData);
          estatus = EEPROM.write(wAddress, wData);
          if(estatus == 0)
          {
            Serial.print("> Write finished, Status: ");
            Serial.println(estatus, HEX);
          }
          else
          {
            Serial.print("> EEPROM error, code: ");
            Serial.println(estatus, HEX);
          }
        }
        else if(args[0][1] == 'r')
        {
          mode1 = 6;
          Serial.println("\r\n> Reading EEPROM: ");
          wAddress = fast_atoi_leading_pos(args[1]);
          Serial.print("> Address 0x");
          Serial.print(wAddress, HEX);
          estatus = EEPROM.read(wAddress, &wData);
          Serial.print(", Data = dec. ");
          Serial.println(wData);
          if(estatus == 0)
          {
            Serial.print("> Read finished, Status: ");
            Serial.println(estatus, HEX);
          }
          else
          {
            Serial.print("> EEPROM error, code: ");
            Serial.println(estatus, HEX);
          }
        }
        break;
      case 'l': //Calibration mode/update
      //Currently broken
        /*
        updateADCRef();
        ADC2V1 = ADC2V1INIT;
        ADC2V2 = ADC2V2INIT;
        ADC2I1 = ADC2I1INIT;
        ADC2I2 = ADC2I2INIT;
        mode1 = 3;
        charge_voltage_1 = DEF_CHG_VOL;
        charge_current_1 = -1000;
        psu_dir_1 = DEF_PSU_MODE;
        state1 = 3;
        settle1 = 0;
        mah1 = 0;
        mwh1 = 0;
        //Timer2.resume(); //Start the timer counting
        setChg1(CHARGE);
        setLED1(LED_CYAN);
        mode2 = 3;
        charge_voltage_2 = DEF_CHG_VOL;
        charge_current_2 = -1000;
        psu_dir_2 = DEF_PSU_MODE;
        state2 = 3;
        settle2 = 0;
        mah2 = 0;
        mwh2 = 0;
        //Timer2.resume(); //Start the timer counting
        setChg2(CHARGE);
        setLED2(LED_CYAN);
        Serial.print("\r\n");
        Serial.println("> Cal Mode");
        Serial.println("> Slot 1/2 corr. reset to default");
        Serial.print(">  ADC corr. factor: ");
        Serial.print(corr_factor*100);
        Serial.println(" Slot 1+2 set to 4.20V, 1.0A");
        Serial.println(">  Press n1, n2 to end");
        Serial.print("\r\n");
        Serial.print("> ");
        */
        break;
      case 'n':
        if(args[0][1] == '1')
        {
          mode1 = 6;
          state1 = 8;
          setChg1(DISCONNECT);
          //Timer2.pause(); //Start the timer counting
          setLED1(LED_OFF);
          Serial.println("\r\n");
          //printMenu(mode1);
        }
        break;
      case 'v':        
        Serial.print("\r\n> Software version: ");
        Serial.println(vers);
        Serial.print("\r\n> ");
        break;
      case 'q':
        if(args[0][1] == '1')
        {
          mode1 = 3;
          state1 = 9;
          settle1 = 0;
          parseTM1(i, args);
          setChg1(CHARGE);
          pwmWrite(OC1PF, tm_duty_1);
          pwmWrite(OC1ON, tm_duty_1);
          //printMenu(mode1);
        }
        break;
      case 's':
        Serial.println("\r\n");
        if((state1 == 8))
        {
          //adciref = analogRead(AIREF); //Iref voltage
          vbuf_i = (int)(((float)getAuxADC(AHVV)) * BHVV2V);
        }
        //Msg type 4 (Debug):
        //(Debug,4,Vbuf,mV,Vrev,mV,Iload,mA,Vload,mV,IR,mOhms,State,Time)
        Serial.print("4,");
        Serial.print(vbuf_i); //2.417);
        Serial.print(",");
        Serial.print(getCell1RV());
        Serial.print(",");
        Serial.print(adciref);//2.417);
        adctemp = getAuxADC(HSTH1);
        if (adctemp > 939) //0-40C, use linear estimation
        {
          temp_t = ((float)adctemp) / -32.56 + 72.980 + TOFFSHS1;
        }
        else //40C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -7047334.3 + ((float)adctemp) * ((float)adctemp) / 3173.10 + ((float)adctemp) / -3.6194 + 143.561) + TOFFSHS1;
        }
        Serial.print(",");
        Serial.print(temp_t);//2.417);
        adctemp = getAuxADC(HSTH2);
        if (adctemp > 939) //0-40C, use linear estimation
        {
          temp_t = ((float)adctemp) / -32.56 + 72.980 + TOFFSHS2;
        }
        else //40C+, use polynomial estimation
        {
          temp_t = (((float)adctemp) * ((float)adctemp) * ((float)adctemp) / -7047334.3 + ((float)adctemp) * ((float)adctemp) / 3173.10 + ((float)adctemp) / -3.6194 + 143.561) + TOFFSHS2;
        }
        Serial.print(",");
        Serial.println(temp_t);//2.417);
        Serial.print("9,");
        Serial.print(vbat_1_1);
        Serial.print(",");
        Serial.print(ibat_1_1);
        Serial.print(",");
        Serial.print(mah1);
        Serial.print(",");
        Serial.print(mwh1);
        Serial.print(",");
        Serial.print(temp1);
        Serial.print(",");
        Serial.println(ir1);
        Serial.print("\r\n> ");
        break;
      #ifdef OLED_ENABLED
        case 'a':
          if(displayEnabled == 0)
          {
            displayEnabled = 1;
            Serial.println("\r\n> Disp Enable\r\n>");
          }
          else
          {
            displayEnabled = 0;
            Serial.println("\r\n> Disp Disable\r\n>");
          }
          break;
      #endif
      case '?':
        mode1 = 99;
        printMenu(mode1);
        //Timer2.pause(); //Start the timer counting
        break;
      case 'z':
        //Soft reset
        nvic_sys_reset();
        break;
      default:
        mode1 = 99;
        printMenu(mode1);
        //Timer2.pause(); //Start the timer counting
        break;
    }

    newData = false;
  }
  //Display thread; update 1 Hz
  if((tick - last_tick) >= 2000)
  {
    last_tick = tick;

    if(displayEnabled)
    {
      #ifdef OLED_ENABLED
      display.clearLine1();
      display.setCursor(0,0);
      display.print((int)vbat_1_1);
      display.setCursor(30,0);
      display.print((int)vbat_1_2);
      display.displayLine1();
      #endif
    }
    
    if((getChgPwr()+getDisPwr()) < 4500)
    {
      fanSpeed = 1;
    }
    else
    {
      fanSpeed = 2;
    }
  }
  if((state1 == 1) || (state1 == 3) || (state1 == 4)) //Fan on
  {
    if(fanSpeed == 1) //Fan PWM
    {
      if((tick - last_tick_fan) >= 16)
      {
        last_tick_fan = tick;
        if(fantoggle) //50% duty, 62.5Hz
        {
          fantoggle = !fantoggle;
          digitalWrite(FANON, fantoggle); //Non-inverting mode
        }
        else
        {
          fantoggle = !fantoggle;
          digitalWrite(FANON, fantoggle); //Non-inverting mode
        }
      }
    }
    else //Fan full on
    {
      digitalWrite(FANON, HIGH); //Non-inverting mode
    }
  }
  else //Fan off
  {
    digitalWrite(FANON, LOW); //Non-inverting mode
  }
  //delay(100);
  /*loopcnt++;
  if(loopcnt > 9)
    loopcnt = 0;*/
  if (interruptCounter > 8) {
    setChg1(DISCONNECT);
    mode1 = 5;
    state1 = 8;
    setLED1(LED_RED);
    Serial.println("> Int Ovf Error, Stopping!");
    Serial.print("> Ints Pending: ");
    Serial.println(interruptCounter);
    interruptCounter--;
    runStateMachine();
  }
  else if (interruptCounter > 0) {
    interruptCounter--;
    runStateMachine();
  }
  if (slot1_startup > 0)
  {
    slot1_startup--;
    if(slot1_startup == 0)
      digitalWrite(C1ON, LOW); //synchronous buck started, reconnect cell 
  }
}

//2kHz interrupt
void handler_loop(void) {
  interruptCounter++;
  tick++;
}


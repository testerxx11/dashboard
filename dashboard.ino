#include "Wire.h"
#define DS3232_I2C_ADDRESS 0x68

#include <SPI.h>
#include <Adafruit_GFX.h>
//https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_SSD1306.h>

#include <BME280I2C.h>
#include <EEPROM.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);  //!!! WTF??

/* ==== Global BME280 Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
/* ==== END BME280 Global Variables ==== */

//#define LOGO16_GLCD_HEIGHT 16 
//#define LOGO16_GLCD_WIDTH  16 
//static const unsigned char PROGMEM logo16_glcd_bmp[] =
//{ B00000000, B11000000,
//  B00000001, B11000000,
//  B00000001, B11000000,
//  B00000011, B11100000,
//  B11110011, B11100000,
//  B11111110, B11111000,
//  B01111110, B11111111,
//  B00110011, B10011111,
//  B00011111, B11111100,
//  B00001101, B01110000,
//  B00011011, B10100000,
//  B00111111, B11100000,
//  B00111111, B11110000,
//  B01111100, B11110000,
//  B01110000, B01110000,
//  B00000000, B00110000 };

#define SERIAL_BAUD 115200 // 9600 //115200

const byte PWM_PIN=3; // D3 nano
const byte SPEED_SENSOR_PIN=2; // D2 nano
const byte TAXOMETR_PIN=3; // D3 nano
const byte BUTTON_DWN_PIN=4; // D4 nano
const byte BUTTON_UP_PIN=5; // D5 nano
const byte BUTTON_ENT_PIN=6; // D6 nano
const byte BUTTON_ESC_PIN=7; // D7 nano
//const int ICP_PIN=8; // D8 nano
//T1 = D5 nano

volatile unsigned int int_per_loop;
unsigned int int_per_loop_display;
volatile unsigned int int_taxo_per_loop;
unsigned int int_taxo_per_loop_display;

unsigned long start, finished, elapsed;

unsigned long start_loop, finished_loop, elapsed_loop;

uint16_t ticks_per_meter = 30;
unsigned long speed_pulses = 0;
unsigned long odometr_tics = 256009 * 1000 / 8 * ticks_per_meter;


//unsigned long odometr = 256009;
unsigned long odometr_0 = 256009 - 259;
unsigned long odometr_1 = 256009 - 1050;
//uint16_t odometr_0_show = 259;

float speed = 0.0;
float tmp_float;

//uint16_t temp = 66;
uint16_t memory_free;

//byte display_mode = 0; // 0 - default(odometers); 1 - speedo/taxo; 2 - meteo; 6 - DBG
#define SCREEN_MAIN 0
#define SCREEN_SETTINGS 4
#define SCREEN_DBG 5

int8_t path [4] = {SCREEN_MAIN, -1, -1, -1};
int8_t level_deep = 0;


unsigned long convert_odometer_ticks_to_km(unsigned long odo_ticks)
{
    tmp_float = odo_ticks; 
    tmp_float = tmp_float * 8  / (ticks_per_meter * 1000);
    odo_ticks = tmp_float;
    return odo_ticks;
}



unsigned long read_eeprom(uint16_t addr)
{
    unsigned long val = 0L;
    for(int step=0; step < 4; step++)
    {
        byte tmp_byte = EEPROM.read(addr);
        Serial.println(F("read eeprom:"));
        Serial.println(tmp_byte, HEX);
        val = (val << 8) | tmp_byte;
        addr++;
    }
    return val;
}

void write_eeprom(uint16_t addr, unsigned long val)
{
    for(int step=3; step >= 0; step--)
    {
        byte tmp_byte = val & 0xFF;
        //Serial.println(F("write eeprom:"));
        //Serial.println(tmp_byte, HEX);
        EEPROM.write(addr+step, tmp_byte);
        val = val >> 8;
    }
  
    /***
    The function EEPROM.update(address, val) is equivalent to the following:
  
    if( EEPROM.read(address) != val ){
      EEPROM.write(address, val);
    }
  ***/
}

// eeprom memory maping
// 00 01 02 03 odometr
// 04 05 06 07 odometr check
// 08 09 0A 0B odometr save copy
// 0C 0D 0E 0F odometr save copy check
// 10 11 12 13 odometr 0 ( start point )
// 14 15 16 17 odometr 1 ( start point )
// 18 19 1A 1B reserved for odometr 2
// 1C 1D 1E 1F reserved for odometr 3
// 20 21 22 23 reserved for odometr 4
// 24 25       ticks_per_meter (24 reserved for hi byte)
// 26

unsigned long odometr_last_save = 0L;

void write_odometr(unsigned long val)
{
    if(odometr_last_save != val)
    {
        Serial.print(F("Write odometr:"));
        Serial.println(val, HEX);
        uint16_t addr = 0;
        for(int step=0; step < 4; step++)
        {
            write_eeprom(addr, val);
            addr+=4;
        }
        odometr_last_save = val;
    }    
}

void reset_odometr(int8_t odo_numb)
{
    // odo_numb: 0/1
    if( odo_numb == 0 )
    {
        write_eeprom(0x10, odometr_tics);
        odometr_0 = odometr_tics;
    }
    else if( odo_numb == 1 )
    {
        write_eeprom(0x14, odometr_tics);
        odometr_1 = odometr_tics;
    }
}


byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
float temp = 66.6;
float hum(NAN), pres(NAN);
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */


void setup() {

  Serial.begin(SERIAL_BAUD);
  //while(!Serial) {} // wait for serial port to connect. Needed for Leonardo only

  unsigned long tmp_long;
//  delay(15000);
//  unsigned long test_odo = 960030720; //0xAABBCCDF;
//  tmp_long = read_eeprom(0);
//  if( tmp_long != test_odo )
//  {
//      Serial.print(F("Read odometr:"));
//      Serial.println(tmp_long, HEX);
//      //write_eeprom(0, test_odo);
//      write_odometr(test_odo);
//  }
//  delay(60000);

  tmp_long = read_eeprom(0);
  odometr_tics = tmp_long;
  odometr_0 = odometr_tics; // TODO
  odometr_1 = odometr_tics; // TODO
  
  Wire.begin();
  
  // set the initial time here:
  // DS3232 seconds, minutes, hours, day, date, month, year
  //setDS3232time(0, 57, 9, 3, 28, 3, 17); // 1 - sun 2 mon

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //0x3C initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,0);
  display.print(F("SUPER BLACKBIRD"));
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  
  
  while(!bme.begin())
  {
    Serial.println(F("Could not find BME280 sensor!"));
    delay(1000);
  }

  pinMode(SPEED_SENSOR_PIN, INPUT);
  attachInterrupt(0, speed_sensor_interrupt, FALLING);

  //pinMode(PWM_PIN, OUTPUT);
  pinMode(TAXOMETR_PIN, INPUT);
  attachInterrupt(1, taxometr_interrupt, FALLING);

  TCCR1A=0; //reset timer/counter control register A

  pinMode(BUTTON_DWN_PIN, INPUT);
  digitalWrite(BUTTON_DWN_PIN, 1);
  pinMode(BUTTON_UP_PIN, INPUT);
  digitalWrite(BUTTON_UP_PIN, 1);
  pinMode(BUTTON_ENT_PIN, INPUT);
  digitalWrite(BUTTON_ENT_PIN, 1);
  pinMode(BUTTON_ESC_PIN, INPUT);
  digitalWrite(BUTTON_ESC_PIN, 1);
  

}

void speed_sensor_interrupt()
{
  int_per_loop++;
}

void taxometr_interrupt()
{
  int_taxo_per_loop++;
}



void show_display(byte hour, byte minute, byte second)
{
  display.clearDisplay();

  if(path[0] == SCREEN_DBG)
  {// DBG mode
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print(memory_free, DEC);
    display.setTextSize(2);
    display.setCursor(0,20);
    display.print(int_per_loop_display, DEC); // print speed sensor, Hz
    speed = int_per_loop_display * 0.108333;
    display.setCursor(60,20);
    display.print(speed, 1);
    display.setCursor(0,40);
    display.print(int_taxo_per_loop_display, DEC);
  }
  else if(path[0] == SCREEN_SETTINGS)
  {
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(F("SETTINGS"));
    
    if (( level_deep == 1) && ( path[1] == 0))
    {// inside screen, select tiks per meter
        display.setCursor(10,20);
        display.print(F("TICKS PER METER"));
    }
    
    
  }
  else if(path[0] == 3)
  {
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(F("METEO"));
  }
  else if(path[0] == 2)
  {
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(F("GEAR"));
  }
  else if(path[0] == 1)
  {
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print(F("SPEED"));
  }
  else // default path[0] == 0 SCREEN_MAIN
  {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    //tmp_float = odometr_tics; 
    //tmp_float = tmp_float * 8  / (ticks_per_meter * 1000);
    //odometr = tmp_float;
    unsigned long tmp_ulong;
    tmp_ulong = convert_odometer_ticks_to_km(odometr_tics);
    display.print(tmp_ulong, DEC); //print overall odometer
    display.setCursor(45,0);
    display.print(hour, DEC); //print time
    display.print(":");
    if (minute<10)
    {
      display.print("0");
    }
    display.print(minute, DEC);
    display.print(".");
    if (second<10)
    {
      display.print("0");
    }
    display.print(second, DEC);
  
    display.setCursor(102,17);
    display.print(hum,0); // humidity with round // display.print("75%");
    display.print(" %");
  
    display.setTextSize(2);
    
    display.setCursor(80,40);
    display.print(F("13.5")); // Voltage
    
    display.setCursor(100,0);
    display.print(temp, 0); // print temperature
    display.drawPixel(127, 0, WHITE);
    display.drawPixel(126, 0, WHITE);
    display.drawPixel(127, 1, WHITE);
    display.drawPixel(126, 1, WHITE);
    
    //--------- odometers ------------------------------------
    uint16_t tmp16;
    uint8_t tmp8;
    if (( level_deep == 1) && ( path[1] == 0))
    {// inside screen, selected odo 0
        display.setTextColor(BLACK, WHITE); // 'inverted' text
    }
    
    display.setTextSize(3);
    display.setCursor(10,20);
    //display.print(odometr_0_show); // print odometr 0
    if(( level_deep == 2 ) && ( path[1] == 0 ))
        display.print(F("RESET?"));
    else
    {
        tmp16 = (uint16_t) ((odometr_0 - odometr_tics) * 8  / (ticks_per_meter)); // get meters
        tmp8 = tmp16 % 1000; // get remaind meters
        tmp8 = tmp8 / 100; // 800m -> 8
        tmp16 = tmp16 / 1000; // m -> km
        display.print(tmp16); // print odometr 0
        display.setTextSize(2);
        display.print(".");
        display.print(tmp8);
    }
  
    display.fillCircle(3, 26, 3, WHITE);
    
    if (( level_deep == 1) && ( path[1] == 1))
    {// inside screen , selected odo 1
        display.setTextColor(BLACK, WHITE); // 'inverted' text
    }
    else
        display.setTextColor(WHITE);
    
    display.setTextSize(2);
    display.setCursor(10,50);
    if(( level_deep == 2 ) && ( path[1] == 1 ))
        display.print(F("RESET?"));
    else
        display.print("TODO"); // print odometr 1
    
    //--------- end odometers ------------------------------------
  
    //display.drawChar(80, 20, 'B', 0, 1, 2);
    
  }  // END default path[0] == 0
 
  //start = millis();
  display.display();
  //finished = millis();
  //elapsed = finished - start;
  //Serial.print(elapsed, DEC);
  //Serial.print("/");
}

//---------------- DS3232 time -------------------------------------------------------

void setDS3232time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3232
  Wire.beginTransmission(DS3232_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}

void readDS3232time(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,
byte *month,byte *year)
{
  Wire.beginTransmission(DS3232_I2C_ADDRESS);
  Wire.write(0); // set DS3232 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3232_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3232 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

byte hour, minute, second;

void read_time()
{
  byte dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3232
  readDS3232time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  if(false)
  {// used for debug
      // send it to the serial monitor
      Serial.print(hour, DEC);
      // convert the byte variable to a decimal number when displayed
      Serial.print(":");
      if (minute<10)
      {
        Serial.print("0");
      }
      Serial.print(minute, DEC);
      Serial.print(":");
      if (second<10)
      {
        Serial.print("0");
      }
      Serial.print(second, DEC);
      Serial.print(" ");
      
      Serial.print(year, DEC);
      Serial.print("/");
      Serial.print(month, DEC);
      Serial.print("/");
      Serial.print(dayOfMonth, DEC);
          
      Serial.print(F(" Day of week: "));
      switch(dayOfWeek){
        case 1:
          Serial.println(F("Sunday"));
          break;
        case 2:
          Serial.println(F("Monday"));
          break;
        case 3:
          Serial.println(F("Tuesday"));
          break;
        case 4:
          Serial.println(F("Wednesday"));
          break;
        case 5:
          Serial.println(F("Thursday"));
          break;
        case 6:
          Serial.println(F("Friday"));
          break;
        case 7:
          Serial.println(F("Saturday"));
          break;
      }
  }
}

//---------------- END DS3232 time -------------------------------------------------------



extern int __bss_end;
extern void *__brkval;
int memoryFree()
{
  int freeValue;
  if((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)&__brkval);
  return freeValue;  
}



unsigned long previous_mrs = 0;
unsigned long current_mrs;
uint16_t interval = 3000; // us
bool ledState = LOW;

int read_bme_counter = 0;
int write_odometr_counter = 0;

uint16_t final_delay;

//int tmp;
int tmp_int;
uint16_t pin_push_counter;

void loop() 
{
    int_per_loop = 0;
    int_taxo_per_loop = 0;
    start_loop = micros();

    // Set Counter Clock is external pin
    bitSet(TCCR1B, CS12);
    bitSet(TCCR1B, CS11);

    speed_pulses += int_per_loop_display;
    //tmp_int = speed_pulses >> 11; // /2048
    tmp_int = speed_pulses >> 3; // /8
    if( tmp_int > 0 )
    {
        //speed_pulses &= 0x7FF;
        speed_pulses &= 0x7;
        odometr_tics += tmp_int;
    }  

    write_odometr_counter++;
    if(( int_per_loop_display == 0 ) || (write_odometr_counter > 1800))
    {// if there are no impulses -> stop happens. OR 30min moving.
        write_odometr(odometr_tics);
        write_odometr_counter = 0;
    }
      
//    Serial.println("");
      memory_free = memoryFree();
//    Serial.println(memory_free);
      
    read_time(); // display the real-time clock data on the Serial Monitor

    if(read_bme_counter <= 0)
    {
        read_bme();
        read_bme_counter = 20;
    }
    else
        read_bme_counter--;

    start = millis();
    show_display(hour, minute, second);
    finished = millis();
    elapsed = finished - start;
    Serial.print(elapsed, DEC);
    Serial.print(F(" ms. "));

    //printBME280Data(&Serial);
    //printBME280CalculatedData(&Serial);

    Serial.print(elapsed_loop, DEC);
    Serial.println(F(" us. "));


    byte btn_list [4]  = {BUTTON_DWN_PIN, BUTTON_UP_PIN, BUTTON_ENT_PIN, BUTTON_ESC_PIN };
  
    pin_push_counter = 0;
    int last_pin_press = 0;
    while(true)
    { //btn read cycle
        finished_loop = micros();
        elapsed_loop = finished_loop - start_loop; // us
        final_delay = 1000 - elapsed_loop/1000; // ms from every second
        if(final_delay <= 0)
          break;

        if(final_delay < 800)
        {// litle pause for prevent secondary push
            bool break_btn_read_cycle = false;
            //current_mrs = micros();
            for( int step = 0; step < 4; step++ )
            {// read all buttons
                tmp_int = digitalRead(btn_list[step]); // read the input pin 8 uS

                if ( tmp_int == 0 )
                {
                    Serial.print(" captured.");
                    if( last_pin_press != step)
                    {    
                        last_pin_press = step;
                        pin_push_counter = 0;
                    }
                    else
                        pin_push_counter++;
                    
                    if( pin_push_counter > 10  )
                    {
                        button_processing(btn_list[step]);
                        break_btn_read_cycle = true;
                        break;
                    }
                }
            }          
            //current_mrs = micros() - current_mrs;
            //Serial.print(" digRead,us: ");
            //Serial.print(current_mrs, DEC); // 8 uS
            if ( break_btn_read_cycle == true )
              break;
        }
    
    }//end btn read cycle
  
  
    //Serial.print(int_per_loop, DEC);  
    int_per_loop_display = int_per_loop;
    //int_taxo_per_loop_display = int_taxo_per_loop;
    TCCR1B=0; // Stop counter
    int_taxo_per_loop_display = TCNT1;
    TCNT1=0;
      
} // end loop

void button_processing(byte btn_numb)
{
    Serial.print(" pin:");
    Serial.println(btn_numb);
    if ( level_deep == 0 )
    {// level 0 - root screens
        if ( btn_numb == BUTTON_DWN_PIN)
        {
            path[0]++;
            if ( path[0] > 5 ) path[0] = 0;
        }
        else if ( btn_numb == BUTTON_UP_PIN)
        {
            path[0]--;
            if ( path[0] < 0 ) path[0] = 5;
        }
        else if ( btn_numb == BUTTON_ENT_PIN)
        {// enter to screen
            level_deep = 1;
            path[1] = -1; // by default - nothing
            if( path[0] == SCREEN_MAIN )
            {// if overal screen
                path[1] = 0; // select 0 odo
            }
            else if ( path[0] == SCREEN_SETTINGS ) 
            {
                path[1] = 0; // select tiks per meter
            }
        }
        else // = BUTTON_ESC_PIN
            path[0] = 0;
        
    }//end level 0 - root screens
    else if ( level_deep == 1 )
    {// level 1 - inside screen
        if ( btn_numb == BUTTON_ESC_PIN)
        {
            level_deep = 0;
        }
        else if ( path[0] == SCREEN_MAIN )
        {
            if (( btn_numb == BUTTON_DWN_PIN) || ( btn_numb == BUTTON_UP_PIN))
                path[1] = (~path[1]) & 0x1; // 0->1 1->0 change pos selected odometr
            else if ( btn_numb == BUTTON_ENT_PIN)
            {// select odo to reset
                level_deep = 2;
            }
        }
        
    }//end level 1 - inside screen
    else if ( level_deep == 2 )
    {
        if ( btn_numb == BUTTON_ESC_PIN)
        {
            level_deep = 1;
        }
        else if (( path[0] == SCREEN_MAIN ) && ( btn_numb == BUTTON_ENT_PIN ))
        {
            reset_odometr(path[1]);
            level_deep = 1;
        }
    }

    Serial.print(" level_deep:");
    Serial.print(level_deep);

    Serial.print(" path[0]:");
    Serial.print(path[0]);

    Serial.print(" path[1]:");
    Serial.println(path[1]);
    
 
}

void loopDBG() {
  // put your main code here, to run repeatedly:
  //displayTime(); // display the real-time clock data on the Serial Monitor,
  //delay(1000); // every second

//  for(int i=0; i<256; i++)
//  {
//    analogWrite(PWM_PIN, i);
//    delay(10);
//  }
//  for(int i=255; i>=0; i--)
//  {
//    analogWrite(PWM_PIN, i);
//    delay(10);
//  }

  current_mrs = micros();

  if(current_mrs - previous_mrs > interval)
  {
    previous_mrs = current_mrs;
    if(ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(PWM_PIN, ledState);
  }

//  digitalWrite(PWM_PIN, HIGH);
//  delay(1);
//  digitalWrite(PWM_PIN, LOW);
//  delay(1);
  
}


uint8_t pressureUnit(3);

//---------- bme ------------------

void read_bme()
{
   // read time = 2-3 mS
   // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
   bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  /* Alternatives to ReadData():
    float temp(bool celsius = false);
    float pres(uint8_t unit = 0);
    float hum();

    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
   */
//  Serial.print(F("Temp:"));
//  Serial.print(temp);
//  Serial.print("C");
//  Serial.print("\tHum:");
//  Serial.print(hum);
//  Serial.print("%RH");
//  Serial.print(F("\tPressure:"));
//  Serial.print(pres);
//  Serial.print(" atm");
  
}

void printBME280Data(Stream* client){
  
//  start = millis();
                            // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
   bme.read(pres, temp, hum, metric, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  /* Alternatives to ReadData():
    float temp(bool celsius = false);
    float pres(uint8_t unit = 0);
    float hum();

    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
   */

//  finished = millis();
//  elapsed = finished - start;
//  Serial.print(elapsed, DEC);
//  Serial.print("/");
   
  client->print(F("Temp:"));
  client->print(temp);
  client->print("C"); //client->print("°"+ String(metric ? 'C' :'F'));
  client->print("\tHum:");
  client->print(hum);
  client->print("%RH");
  client->print(F("\tPressure:"));
  client->print(pres);
  client->print(" atm");
}

float altitude, dewPoint;

void printBME280CalculatedData(Stream* client){
  altitude = bme.alt(metric);
  dewPoint = bme.dew(metric);
  client->print(F("\tAlt:"));
  client->print(altitude, 2);
  client->print("m"); //client->print((metric ? "m" : "ft"));
  client->print(F("\tDew point: "));
  client->print(dewPoint, 2);
  client->println("C"); //client->println("°"+ String(metric ? 'C' :'F'));

}
//---------- END bme -----------------



/*
   code to publish sensor data from ESP32 to Raspberry Pi via ROS
   Hardware to pull data from:
   TCA9548A I2C Multiplexer
   3x MLX90640 IR Array
   BME680 Humidity Sensor
   VL53L0X IR Range Finder
   BNO055 Inertial Measurement Unit
*/

#include <limits>
#include <stdlib.h>

// ros and message types
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt8MultiArray.h>

#define THERMAL_SIZE 384

// publisher declarations
ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher range_pub("range", &range_msg);
sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("intern_temp", &temp_msg);
sensor_msgs::RelativeHumidity humid_msg;
ros::Publisher humid_pub("intern_humidity", &humid_msg);
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
std_msgs::UInt8MultiArray thermal_msg;
std_msgs::MultiArrayLayout thermal_msg_layout;
std_msgs::MultiArrayDimension thermal_msg_dim[1];
std_msgs::MultiArrayDimension thermal_msg_row;
uint8_t thermal_data[THERMAL_SIZE];
ros::Publisher thermal_pub0("thermal_0u", &thermal_msg);
ros::Publisher thermal_pub1("thermal_1u", &thermal_msg);
ros::Publisher thermal_pub2("thermal_2u", &thermal_msg);
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// I2C lib
#include <Wire.h>

// sensor libraries and class initializations
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
// https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example/issues/2
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
static float mlx90640To[768];
paramsMLX90640 mlx90640[4];

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055();

// multiplexer address
#define TCAADDR 0x70

/*                      *\
 *  BMS declarations
\*                      */
// Author of original BMS code: https://github.com/stuartpittaway/diyBMS
// Modified to work with esp32 and ROS

//#include <power_supply.h>
#include "bms_values.h"
#include "i2c_cmds.h"
#include "settings.h"
/*#include "SoftAP.h"
#include "WebServiceSubmit.h"*/
#include <Wire.h>
#include <time.h>
#include <esp_timer.h>
#include <Ticker.h>

// gpio 13 connected to red led for bms cell
#define LED_ON digitalWrite(13, LOW)
#define LED_OFF digitalWrite(13, HIGH)

// bms globals
const int sensorIn = A0;
int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module
int ACS712shift = 22;
bool InverterMon = false;

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;


//Allow up to 24 modules
cell_module cell_array[24];
int cell_array_index = -1;
int cell_array_max = 0;
unsigned long next_submit;
bool runProvisioning;

bool manual_balance = false;
bool auto_balance = true;
bool max_enabled = false;
// 0=No balancing 1=Manual balancing started 2=Auto Balancing enabled 3=Auto Balancing enabled and bypass happening 4=A module is over max voltage
int balance_status = 2;

  
//Configuration for thermistor conversion
//use the datasheet to get this data.
//https://www.instructables.com/id/NTC-Temperature-Sensor-With-Arduino/
float Vin=3.3;     // [V]        
float Rt=10000;    // Resistor t [ohm]
float R0=20000;    // value of rct in Temp0 [ohm]
float Temp1=273.15;   // [K] in datasheet 0º C
float Temp2=373.15;   // [K] in datasheet 100° C
float RT1=35563;   // [ohms]  resistence in Temp1
float RT2=549.4;     // [ohms]   resistence in Temp2
float beta=0.0;    // initial parameters [K]
float Rinf=0.0;    // initial parameters [ohm]   
float Temp0=298.15;   // use Temp0 in Kelvin [K]
float Vout=0.0;    // Vout in A0 
float Rout=0.0;    // Rout in A0
float TempK=0.0;   // variable output
float TempC=0.0;   // variable output


// muliplexer port assignments
#define THERMAL_0 0
#define THERMAL_1 1
#define THERMAL_2 3
#define BME 2
#define BNO 7
#define BMS 4   // need to check

// whether each sensor is online
bool bmeOnline = false;
bool bnoOnline = false;
bool thermalOnline = false;

Ticker myTicker;

// initialize ROS publishers and each sensor
void setup() {
  Serial.begin(57600);
  Serial.println("\n\n");
  Serial.println("ESP32 software started");
  // ros initializations
  nh.initNode();
  Serial.println("Ros Node initialized");
  //nh.advertise(range_pub);
  nh.advertise(temp_pub);
  nh.advertise(humid_pub);
  nh.advertise(imu_pub);
  nh.advertise(thermal_pub0);
  nh.advertise(thermal_pub1);
  nh.advertise(thermal_pub2);
  nh.advertise(battery_state_pub);
  nh.spinOnce(); //lets the ros node handler send data to pi
  Serial.println("ROS Publishers Initialized");
  Serial.println("\n\n");

  // I2C init
  Wire.begin();
  //Serial.begin(57600);
  // BME init
  tcaselect(BME);
  if (!bme.begin()) {
    // could not find BME
    Serial.println("Failed to connect to BME");
  } else {
    bmeOnline = true;
  }

  // Set up oversampling and filter initialization
  if(bmeOnline){
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

//  // range init
//  tcaselect(RANGE);
//  if (!lox.begin()) {
//    // could not find IR range
//    Serial.println("Failed to connect to IR Range");
//    while (1);
//  }

  // BNO init
  tcaselect(BNO);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  } else {
    bnoOnline = true;
  }
  
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;

  // thermal inits
  Serial.println("initializing thermal arrays");
  thermalOnline = true;
  MLX_init(THERMAL_0);
  MLX_init(THERMAL_1);
  MLX_init(THERMAL_2);
  Serial.println("thermal arrays initialized");

  // init some constant data for ROS messages
  temp_msg.variance = 1.0;
  humid_msg.variance = 0.03;
  
  const char row[4] = "row";
  thermal_msg_row.label = row;
  thermal_msg_row.size = THERMAL_SIZE;
  thermal_msg_row.stride = THERMAL_SIZE;

  thermal_msg_dim[0] = thermal_msg_row;

  thermal_msg_layout.dim = thermal_msg_dim;
  thermal_msg_layout.dim_length = 1;

  thermal_msg.layout = thermal_msg_layout;
  thermal_msg.data_length = THERMAL_SIZE;

  // BMS Setup

  myConfig.autobalance_enabled = auto_balance;
  Serial.begin(19200);           // start serial for output
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  //13 is LED
  pinMode(13, OUTPUT);
  LED_OFF;

  //Thermistor setup
  beta=(log(RT1/RT2))/((1/Temp1)-(1/Temp2));
  Rinf=R0*exp(-beta/Temp0);
  
  Serial.println(F("DIY BMS Controller Startup"));

  initWire();

  if (LoadConfigFromEEPROM()) {
    Serial.println(F("Settings loaded from EEPROM"));
  } else {
    Serial.println(F("We are in initial power on mode (factory reset)"));
    FactoryResetSettings();
  }

  /*if (LoadWIFIConfigFromEEPROM()) {
    Serial.println(F("Connect to WIFI AP"));
    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
      would try to act as both a client and an access-point and could cause
      network-issues with your other WiFi-devices on your WiFi-network. */
 /*   WiFi.mode(WIFI_STA);
    WiFi.begin(myConfig_WIFI.wifi_ssid, myConfig_WIFI.wifi_passphrase);
  } else {
    //We are in initial power on mode (factory reset)
    setupAccessPoint();
  }*/

  scani2cBus();

  //Ensure we service the cell modules every 0.5 seconds
  //os_timer_setfn(&myTimer, timerCallback, NULL);
  //os_timer_arm(&myTimer, 1000, true);
  myTicker.attach_ms(1000, timerCallback);
  

  //Check WIFI is working and connected
  //Serial.print(F("WIFI Connecting"));

  //TODO: We need a timeout here in case the AP is dead!
  /*while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print( WiFi.status() );
  }
  Serial.print(F(". Connected IP:"));
  Serial.println(WiFi.localIP());*/

  //SetupManagementRedirect();
  
  // init some constants for battery state, Battery: 14.4V 5Ah LiNiMnCo 26650 Battery
  battery_state_msg.present = true;  // battery is detected
  battery_state_msg.power_supply_status = 0; //POWER_SUPPLY_STATUS_UNKNOWN
  battery_state_msg.current = 0;
  battery_state_msg.charge = 0;
  battery_state_msg.design_capacity = 5.0;
  battery_state_msg.power_supply_technology = 2; //POWER_SUPPLY_TECHNOLOGY_LION
  
  
  Serial.println("all sensors connected");
  Serial.println("setup complete");
}

// repeatedly read from each sensor and publish to ROS
void loop() {

  // read from BME sensor
  if(bmeOnline){
    tcaselect(BME);
    temp_msg.header.stamp = nh.now();
    humid_msg.header.stamp = nh.now();
    if (bme.performReading()) {
      temp_msg.temperature = bme.temperature;
      humid_msg.relative_humidity = bme.humidity / 100.0;
    } else {
      temp_msg.temperature = 0.0;
      humid_msg.relative_humidity = -1.0;
    }
    temp_pub.publish(&temp_msg);
    humid_pub.publish(&humid_msg);
  }

  // read from TOF sensor
  //VL53L0X_RangingMeasurementData_t measure;

  //tcaselect(RANGE);
  //lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  //range_msg.header.stamp = nh.now();
  //range_msg.radiation_type = sensor_msgs::Range::INFRARED;
//  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//    int range_msr = measure.RangeMilliMeter;
//    range_msg.range = (float)range_msr;
//    range_msg.max_range = (float)measure.RangeDMaxMilliMeter;
//  } else {
//
//    range_msg.range = std::numeric_limits<float>::infinity();
//    range_msg.max_range = 0.0;
//  }
//  range_pub.publish(&range_msg);

  // BNO Readings
  if(bnoOnline){
    imu_msg.header.stamp = nh.now();
    
    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.x = static_cast<double>(quat.x());
    imu_msg.orientation.y = static_cast<double>(quat.y());
    imu_msg.orientation.z = static_cast<double>(quat.z());
    imu_msg.orientation.w = static_cast<double>(quat.w());
  
    imu::Vector<3> angular = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = static_cast<double>(angular.x());
    imu_msg.angular_velocity.y = static_cast<double>(angular.y());
    imu_msg.angular_velocity.z = static_cast<double>(angular.z());
    
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = static_cast<double>(accel.x());
    imu_msg.linear_acceleration.y = static_cast<double>(accel.y());
    imu_msg.linear_acceleration.z = static_cast<double>(accel.z());
  
    imu_pub.publish(&imu_msg);
  }

  // IR Array readings
  if(thermalOnline){
    MLX_read(THERMAL_0);
    for(int j = 0; j < 768; j += THERMAL_SIZE) {
      thermal_msg_layout.data_offset = j;
      thermal_msg.layout = thermal_msg_layout;
      for(int i = j; i < THERMAL_SIZE+j; i++) {
        float temp = mlx90640To[i];
        if (temp <= 25.0) {
          temp = 25;
        }
        else if (temp >= 280.0) {
          temp = 280;
        }
        temp -= 25;
        uint8_t temp2 = static_cast<uint8_t>(temp);
        thermal_data[i%THERMAL_SIZE] = temp2;
      }
    thermal_msg.data = thermal_data;
    thermal_pub0.publish(&thermal_msg);
    }
    MLX_read(THERMAL_1);
    for(int j = 0; j < 768; j += THERMAL_SIZE) {
      thermal_msg_layout.data_offset = j;
      thermal_msg.layout = thermal_msg_layout;
      for(int i = j; i < THERMAL_SIZE+j; i++) {
        float temp = mlx90640To[i];
        if (temp <= 25.0) {
          temp = 25;
        }
        else if (temp >= 280.0) {
          temp = 280;
        }
        temp -= 25;
        uint8_t temp2 = static_cast<uint8_t>(temp);
        thermal_data[i%THERMAL_SIZE] = temp2;
      }
    thermal_msg.data = thermal_data;
    thermal_pub1.publish(&thermal_msg);
    }
    MLX_read(THERMAL_2);
    for(int j = 0; j < 768; j += THERMAL_SIZE) {
      thermal_msg_layout.data_offset = j;
      thermal_msg.layout = thermal_msg_layout;
      for(int i = j; i < THERMAL_SIZE+j; i++) {
        float temp = mlx90640To[i];
        if (temp <= 25.0) {
          temp = 25;
        }
        else if (temp >= 280.0) {
          temp = 280;
        }
        temp -= 25;
        uint8_t temp2 = static_cast<uint8_t>(temp);
        thermal_data[i%THERMAL_SIZE] = temp2;
      }
    thermal_msg.data = thermal_data;
    thermal_pub2.publish(&thermal_msg);
    }
  }

  // processing readings from BMS
    if (cell_array_max > 0) {  
       /* for ( int a = 0; a < cell_array_max; a++) {
          Serial.print(cell_array[a].address);
          Serial.print(':');
          Serial.print(cell_array[a].voltage);
          Serial.print(':');
          Serial.print(cell_array[a].temperature);
          Serial.print(':');
          Serial.print(cell_array[a].bypass_status);
          Serial.print(' ');
        }
        Serial.println();  */
    
    if ((millis() > next_submit) /*&& (WiFi.status() == WL_CONNECTED)*/) {
        //if (myConfig.invertermon_enabled == true ) {
          Voltage = getVPP();
          VRMS = (Voltage/2.0) *0.707; 
          AmpsRMS = (VRMS * 1000)/mVperAmp;
        //}
      
      //emoncms.postData(myConfig, cell_array, cell_array_max);
      //influxdb.postData(myConfig, cell_array, cell_array_max);
 
      for (int a = 0; a < cell_array_max; a++) {
        //Serial.println(" Cell Voltage = " + String(cell_array[a].voltage));
        //Serial.println(" Max Cell Voltage = " + String(myConfig.max_voltage*1000));
        if (cell_array[a].voltage >= myConfig.max_voltage*1000) {
          cell_array[a].balance_target = myConfig.max_voltage*1000; 
           max_enabled = true;
           balance_status = 4;
        } else if (manual_balance!= true) cell_array[a].balance_target = 0;
      }
      if (max_enabled!=true) avg_balance(); 
      max_enabled = false;

      //Update Influxdb/emoncms every 20 seconds
      next_submit = millis() + 20000;
    }
  }
  

  // Read BMS data
  tcaselect(BMS);
  battery_state_msg.header.stamp = nh.now();
  //battery_state_msg.voltage = ;      // voltage level
  //battery_state_msg.current = ;       // current draw/supply from/to battery
  //battery_state_msg.capacity = ;      // remaining capacity used for time alg.
  //battery_state_msg.health = ;        // fix actual value, redundant indicator --> use temp msg
  battery_state_pub.publish(&battery_state_msg);

  nh.spinOnce();
}

// initialize an MLX sensor
void MLX_init(int port) {
  tcaselect(port);
  if (isConnected() == false || thermalOnline==false)
  {
    // sensor not detected
    Serial.printf("Failed to connect to MLX %d\n", port);
    thermalOnline = false;
    return;
  }
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) {
    Serial.printf("Failed to load params MLX %d\n", port);
    while (1); // failed to load system parameters
  }
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640[port]);
  if (status != 0) {
    Serial.printf("Failed to extract params MLX %d\n", port);
    while (1); // failed to extract parameters
  }
  //Serial.printf("gainEE: %d\n", mlx90640[port].gainEE);
  //Serial.println("");
  //Serial.printf("%d, %d, %d\n", mlx90640[port].offset[64], mlx90640[port].offset[357], mlx90640[port].offset[746]);
}

void MLX_read(int port) {
  tcaselect(port);
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status >= 0) {

      float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640[port]);
      float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640[port]);

      float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
      float emissivity = 0.95;
      
      MLX90640_CalculateTo(mlx90640Frame, &mlx90640[port], emissivity, tr, mlx90640To);
    }
  //Serial.printf("%d mode: %f\n", port, (mlx90640Frame[832] & 0x1000) >> 5);
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

// select multiplexer port
void tcaselect(uint8_t i) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

/*                      *\
 *  BMS helper functions
\*                      */
float getVPP() {
  float result;
  
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here

   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn-ACS712shift);

       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }
   //Serial.println(readValue);
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
 }

void avg_balance() {
  uint16_t avgint = 0;
  float avgintf = 0.0;
       
  if ((myConfig.autobalance_enabled == true) && (manual_balance == false)) {
    
    if (cell_array_max > 0) {
    //Work out the average 
    float avg = 0;
    for (int a = 0; a < cell_array_max; a++) {
      avgintf = cell_array[a].voltage/1000.0;
      avg += 1.0 * avgintf;
    }
    avg = avg / cell_array_max;

    avgint = avg;

    balance_status = 2;
    
    //Serial.println("Average cell voltage is currently : " + String(avg*1000));
    //Serial.println("Configured balance voltage : " + String(myConfig.balance_voltage*1000));

    if ( avg >= myConfig.balance_voltage )  {
      for ( int a = 0; a < cell_array_max; a++) {
        if (cell_array[a].voltage > avg*1000) {
          cell_array[a].balance_target = avg*1000;
          balance_status = 3;
        }
      } 
    }  else {
      for (int a = 0; a < cell_array_max; a++) {
        command_set_bypass_voltage(cell_array[a].address,0); }}
        } 
  } else  balance_status = 0;    
}

void print_module_details(struct  cell_module *module) {
  Serial.print("Mod: ");
  Serial.print(module->address);
  Serial.print(" V:");
  Serial.print(module->voltage);
  Serial.print(" VC:");
  Serial.print(module->voltage_calib);
  Serial.print(" T:");
  Serial.print(module->temperature);
  Serial.print(" TC:");
  Serial.print(module->temperature_calib);
  Serial.print(" R:");
  Serial.print(module->loadResistance);
  Serial.println("");
}

void check_module_quick(struct  cell_module *module) {
  module->voltage = cell_read_voltage(module->address);
  module->temperature = tempconvert(cell_read_board_temp(module->address));   
  module->bypass_status = cell_read_bypass_enabled_state(module->address);
 if (module->voltage >= 0 && module->voltage <= 5000) {
    if ( module->voltage > module->max_voltage || module->valid_values == false) {
      module->max_voltage = module->voltage;
    }
    if ( module->voltage < module->min_voltage || module->valid_values == false) {
      module->min_voltage = module->voltage;
    }
    module->valid_values = true;
  } else {
    module->valid_values = false;
  }
}

void check_module_full(struct  cell_module *module) {
  check_module_quick(module);
  module->voltage_calib = cell_read_voltage_calibration(module->address);
  module->temperature_calib = cell_read_temperature_calibration(module->address);
  module->loadResistance = cell_read_load_resistance(module->address);
}

//void timerCallback(void *pArg) {
void timerCallback() {
  LED_ON;

  if (runProvisioning) {
    Serial.println("runProvisioning");
    uint8_t newCellI2CAddress = provision();

    if (newCellI2CAddress > 0) {
      Serial.print("Found ");
      Serial.println(newCellI2CAddress);

      cell_module m2;
      m2.address = newCellI2CAddress;
      cell_array[cell_array_max] = m2;
      cell_array[cell_array_max].min_voltage = 0xFFFF;
      cell_array[cell_array_max].max_voltage = 0;
      cell_array[cell_array_max].balance_target = 0;
      cell_array[cell_array_max].valid_values = false;

      //Dont attempt to read here as the module will be rebooting
      //check_module_quick( &cell_array[cell_array_max] );
      cell_array_max++;
    }

    runProvisioning = false;
    return;
  }


  //Ensure we have some cell modules to check
  if (cell_array_max > 0 && cell_array_index >= 0) {


    if (cell_array[cell_array_index].update_calibration) {

      if (cell_array[cell_array_index].factoryReset) {
        command_factory_reset(cell_array[cell_array_index].address);
      } else {

        //Check to see if we need to configure the calibration data for this module
        command_set_voltage_calibration(cell_array[cell_array_index].address, cell_array[cell_array_index].voltage_calib);
        command_set_temperature_calibration(cell_array[cell_array_index].address, cell_array[cell_array_index].temperature_calib);

        command_set_load_resistance(cell_array[cell_array_index].address, cell_array[cell_array_index].loadResistance);
      }
      cell_array[cell_array_index].update_calibration = false;
    }

    check_module_quick( &cell_array[cell_array_index] );

    if (cell_array[cell_array_index].balance_target > 0) {
      command_set_bypass_voltage(cell_array[cell_array_index].address, cell_array[cell_array_index].balance_target);
      cell_array[cell_array_index].balance_target = 0;
    }

    cell_array_index++;
    if (cell_array_index >= cell_array_max) {
      cell_array_index = 0;
    }
  }

  LED_OFF;
} // End of timerCallback

void scani2cBus() {

  cell_array_index = 0;

  //We have 1 module
  cell_array_max = 0;

  //Scan the i2c bus looking for modules on start up
  for (uint8_t address = DEFAULT_SLAVE_ADDR_START_RANGE; address <= DEFAULT_SLAVE_ADDR_END_RANGE; address++ )
  {
    if (testModuleExists(address) == true) {
      //We have found a module
      cell_module m1;
      m1.address = address;
      //Default values
      m1.valid_values = false;
      m1.min_voltage = 0xFFFF;
      m1.max_voltage = 0;
      cell_array[cell_array_max] = m1;

      check_module_full( &cell_array[cell_array_max] );

      //Switch off bypass if its on
      command_set_bypass_voltage(address, 0);

      print_module_details( &cell_array[cell_array_max] );

      cell_array_max++;
    }
  }
}

float tempconvert(float rawtemp) {
  Vout=Vin*((float)(rawtemp)/1024.0); // calc for ntc
  Rout=(Rt*Vout*(Vin-Vout));

  TempK=(beta/log(Rout/Rinf)); // calc for temperature
  TempC=TempK-273.15;

  return TempC;
}

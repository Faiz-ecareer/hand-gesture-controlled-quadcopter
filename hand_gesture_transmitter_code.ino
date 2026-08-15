#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
//these variables are for mpu 6050. angle calculation
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
booleanset_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; // same for the receiver

RF24 radio(9, 10);  //Set CE and CSN pins

// The sizeof this struct should not exceed 32 bytes
struct Data_to_be_sent {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
  byte ch6;
  byte ch7;
};
//Create a variable with the structure above and name it sent_data
Data_to_be_sentsent_data;
void setup()
{
digitalWrite(13,HIGH); // led will on when calibration is started

Wire.begin();  // begin i2c connection with mpu 6050
radio.begin();
radio.setAutoAck(false);
radio.setDataRate(RF24_250KBPS);
radio.openWritingPipe(my_radio_pipe);

  //Reset each channel value
  sent_data.ch1 = 127;
  sent_data.ch2 = 127;
  sent_data.ch3 = 127;
  sent_data.ch4 = 127;
  sent_data.ch5 = 0;
  sent_data.ch6 = 0;
  sent_data.ch7 = 0;
setup_mpu_6050_registers();      
  for (int cal_int = 0; cal_int<2000 ;cal_int ++){             
    read_mpu_6050_data();                                               
gyro_x_cal += gyro_x;                                           
gyro_y_cal += gyro_y;                                              
gyro_z_cal += gyro_z;                                             
delay(3);                                                       
  }
gyro_x_cal /= 2000;                                                  
gyro_y_cal /= 2000;                                              
gyro_z_cal /= 2000;                                                

delay(2000);
digitalWrite(13,LOW); 

Serial.begin(9600);
}
void loop()
{
read_mpu_6050_data();                                               
gyro_x -= gyro_x_cal;                                                
gyro_y -= gyro_y_cal;                                                
gyro_z -= gyro_z_cal;                                                

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
angle_pitch += gyro_x * 0.0000611;                                   
angle_roll += gyro_y * 0.0000611;                                    

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               

  //Accelerometer angle calculations
acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
angle_pitch_acc -= +1.1;                                             
angle_roll_acc -= -0.0;                                              
  if(set_gyro_angles){
angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
else{
angle_pitch = angle_pitch_acc;                                     
angle_roll = angle_roll_acc;                                        
set_gyro_angles = true;                                            
  }

  //To dampen the pitch and roll angles a complementary filter is used
angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   
angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      
Serial.print(angle_pitch_output);
Serial.print(" ");
Serial.println(angle_roll_output);                    
  /*If your channel is reversed, just swap 0 to 255 by 255 to 0 below
  EXAMPLE:
  Normal:    data.ch1 = map( analogRead(A0), 0, 1024, 0, 255);
  Reversed:  data.ch1 = map( analogRead(A0), 0, 1024, 255, 0);  */

  sent_data.ch1 = map(angle_pitch_output , -90, 90, 0, 255);
  sent_data.ch2 = map( angle_roll_output, -90, 90, 0, 255);
  sent_data.ch3 = map( analogRead(A0), 0, 1024, 0, 255);            
  sent_data.ch4 = map( analogRead(A1), 0, 1024, 0, 255);
  sent_data.ch5 = digitalRead(2);
  sent_data.ch6 = digitalRead(3);
  sent_data.ch7 = map( analogRead(A2), 0, 1024, 0, 255);

radio.write(&sent_data, sizeof(Data_to_be_sent));
}
void read_mpu_6050_data(){                                     
Wire.beginTransmission(0x68);                                        
Wire.write(0x3B);                                                  
Wire.endTransmission();                                            
Wire.requestFrom(0x68,14);                                          
while(Wire.available() < 14);                                   
acc_x = Wire.read()<<8|Wire.read();                              
acc_y = Wire.read()<<8|Wire.read();                            
acc_z = Wire.read()<<8|Wire.read();                              
  temperature = Wire.read()<<8|Wire.read();                         
gyro_x = Wire.read()<<8|Wire.read();                          
gyro_y = Wire.read()<<8|Wire.read();                               
gyro_z = Wire.read()<<8|Wire.read();                               

}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
Wire.beginTransmission(0x68);                                
Wire.write(0x6B);                                          
Wire.write(0x00);                                             
Wire.endTransmission();                                         
  //Configure the accelerometer (+/-8g)
Wire.beginTransmission(0x68);                          
Wire.write(0x1C);                                           
Wire.write(0x10);                                             
Wire.endTransmission();                                         
  //Configure the gyro (500dps full scale)
Wire.beginTransmission(0x68);                                   
Wire.write(0x1B);                                                  
Wire.write(0x08);                                                  
Wire.endTransmission();                                             
}

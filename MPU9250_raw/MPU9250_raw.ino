#include "Wire.h"

#include "I2Cdev.h"
#include "MPU9250.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

void get_one_sample_date_mxyz();
void getAccel_Data(void);
void getGyro_Data(void);
void getCompass_Data(void);
void getCompassDate_calibrated ();

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
double Gxyz[3];
//float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

double acc_pitch;
double gyro_pitch;

double get_gyro;
double compl_pitch;

unsigned long pre_time;
unsigned long cur_time;

double loop_time;

int gyro_deadzone=1;

double x_gyro = 0;
double y_gyro = 0;
double z_gyro = 0;

int16_t gx_offset, gy_offset, gz_offset;

int num_reading_gyro = 100;

static inline int8_t sign(double val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}


void setup()
{
    Wire.begin();
    Serial.begin(38400);                        // 통신속도 38400 bps

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    delay(1000);
    Serial.println("     ");

    //accelgyro.setFullScaleGyroRange(3);
    /*
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    */
    
    accelgyro.setXGyroOffsetUser(-60);
    accelgyro.setYGyroOffsetUser(-60);
    accelgyro.setZGyroOffsetUser(-1);
    /*
    Serial.print("X, Y, Z offset : ");
    Serial.print(accelgyro.getXGyroOffset());
    Serial.print(", ");
    Serial.print(accelgyro.getYGyroOffset());
    Serial.print(", ");
    Serial.print(accelgyro.getZGyroOffset());
    Serial.println("");
    Serial.print("X, Y, Z offset User: ");
    Serial.print(accelgyro.getXGyroOffsetUser());
    Serial.print(", ");
    Serial.print(accelgyro.getYGyroOffsetUser());
    Serial.print(", ");
    Serial.print(accelgyro.getZGyroOffsetUser());
    Serial.println("");
    */
    
    
    //mean_gyro();

    acc_pitch = accPitch();
    gyro_pitch = acc_pitch;
    compl_pitch = acc_pitch;

    pre_time = micros();
    //  Mxyz_init_calibrated ();
}


void loop()
{
    acc_pitch = accPitch();

    cur_time = micros();

    loop_time = (double)(cur_time - pre_time) / 1000000;
    
    /*
    Serial.print("time : ");
    Serial.print(loop_time, 9);
    Serial.println("");
    */
   
    //gyro_pitch = gyroPitch(loop_time, gyro_pitch);
    
    get_gyro = gyroPitch(loop_time, compl_pitch);
    compl_pitch = sign(get_gyro) * ((0.97 * abs(get_gyro)) + (0.03 * abs(acc_pitch)));
    /*
    Serial.print("| A: ");
    Serial.print(acc_pitch, 9);
    Serial.println("");
    Serial.print("=> C: ");
    Serial.print(compl_pitch, 9);
    Serial.println("\n");
    */
   
    Serial.print("C: ");
    Serial.print(compl_pitch, 9);
    Serial.print(" => A: ");
    Serial.print(acc_pitch, 9);
    //Serial.println(", G: ");
    //Serial.print(gyro_pitch, 9);
    Serial.println("\n");
    
    
    /*
    Serial.print("Sign : ");
    Serial.print(sign(get_gyro));
    Serial.print(" | 0.97 -> G: ");
    Serial.print(0.97 * abs(get_gyro), 9);
    Serial.print(" // 0.03 -> A: ");
    Serial.print(0.03 * abs(acc_pitch), 9);
    Serial.println("");

    
    
    
    Serial.print("Pitch Value -> C: ");
    Serial.print(compl_pitch, 9);
    Serial.print(" | A: ");
    Serial.print(acc_pitch, 9);
    Serial.print(" , G: ");
    Serial.print(gyro_pitch, 9);
    Serial.println("");
    */
    
    /*
    Serial.print("X, Y, Z raw-gyro : ");
    Serial.print((double) gx);
    Serial.print(", ");
    Serial.print((double) gy);
    Serial.print(", ");
    Serial.print((double) gz);
    Serial.println(" ");
    */

    /*
    Serial.print("X, Y, Z rotation : ");
    Serial.print(accelgyro.getRotationX());
    Serial.print(", ");
    Serial.print(accelgyro.getRotationY());
    Serial.print(", ");
    Serial.print(accelgyro.getRotationZ());
    Serial.println(" ");
    */
    
    /*
    Serial.println("Gyro(degress/s) of X,Y,Z:");
    Serial.print(Gxyz[0]);
    Serial.print(",");
    Serial.print(Gxyz[1]);
    Serial.print(",");
    Serial.println(Gxyz[2]);
    */
    pre_time = cur_time;
    
    /*
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    Serial.println("calibration parameter: ");
    Serial.print(mx_centre);
    Serial.print("         ");
    Serial.print(my_centre);
    Serial.print("         ");
    Serial.println(mz_centre);
    Serial.println("     ");

    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);
    Serial.println("Gyro(degress/s) of X,Y,Z:");
    Serial.print(Gxyz[0]);
    Serial.print(",");
    Serial.print(Gxyz[1]);
    Serial.print(",");
    Serial.println(Gxyz[2]);
    Serial.println("Compass Value of X,Y,Z:");
    Serial.print(Mxyz[0]);
    Serial.print(",");
    Serial.print(Mxyz[1]);
    Serial.print(",");
    Serial.println(Mxyz[2]);
    Serial.println("The clockwise angle between the magnetic north and X-Axis:");
    Serial.print(heading);
    Serial.println(" ");
    Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
    Serial.println(tiltheading);
    Serial.println("   ");
    Serial.println();*/
}



/*
void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}
void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}
void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}

void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
    }
    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];
    
    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}
void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}
*/
void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 2048;   // 2G : 16384
    Axyz[1] = (double) ay / 2048;   // 2G : 16384
    Axyz[2] = (double) az / 2048;   // 2G : 16384
}
void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx / 16.4;   //* 250 / 32768;
    Gxyz[1] = (double) gy / 16.4;   //* 250 / 32768;
    Gxyz[2] = (double) gz / 16.4;   //* 250 / 32768;
}
/*
void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}
void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}
*/

double accPitch(void) {
  getAccel_Data();
  
  double pitch;
  pitch = (double)-atan2(Axyz[0], Axyz[2]) * 180 / PI;

  return pitch;
}

double gyroPitch(double looptime, double pre_pitch) {
  getGyro_Data();
  
  double pitch;
  pitch = pre_pitch + Gxyz[1] * looptime ;

  /*
  Serial.print("pre_pitch: ");
  Serial.println(pre_pitch, 9);
  Serial.print("\t\tGxyz[1] * looptime: ");
  Serial.print(Gxyz[1]*looptime, 9);
  Serial.print("(<= Gxyz[1]: ");
  Serial.print(Gxyz[1], 9);
  Serial.print(", gy: ");
  Serial.print((double)gy, 9);
  Serial.print(", looptime: ");
  Serial.print(looptime, 9);
  Serial.println(")");
  Serial.print("\t\tpitch_G: ");
  Serial.print(pitch, 9);
  Serial.print("(");
  Serial.print(0.97 * abs(pitch), 9);
  Serial.print(")");
  */
  
  /* <boundary value change> Degree -180 ~ +180 */
    if (pitch > -180 && pitch < 180) {
      return pitch;
    }
    else if (pitch <= -180) {
      pitch = 360 + pitch; // x = 180 - ( abs(x) - 180 )  
      Serial.println("!!!!");
    }
    else {  // (pitch_gyro >= 180)
      pitch = -360 + pitch;
      Serial.println("!!!!");
    }
  

    return pitch;
}

void mean_gyro() {

  getGyro_Data();

  for(int i=0; i<num_reading_gyro; i++) {
    getGyro_Data();

    Serial.print("X, Y, Z raw-gyro : ");
    Serial.print((double) gx);
    Serial.print(", ");
    Serial.print((double) gy);
    Serial.print(", ");
    Serial.print((double) gz);
    Serial.println(" ");
    
    x_gyro += (double) gx;
    y_gyro += (double) gy;
    z_gyro += (double) gz;
    delay(100);
  }

  x_gyro /= num_reading_gyro;
  y_gyro /= num_reading_gyro;
  z_gyro /= num_reading_gyro;

  Serial.print("X, Y, Z mean-raw-gyro : ");
  Serial.print(x_gyro);
  Serial.print(", ");
  Serial.print(y_gyro);
  Serial.print(", ");
  Serial.print(z_gyro);
  Serial.println(" ");

  gx_offset=(int16_t) -x_gyro;
  gy_offset=(int16_t) -y_gyro;
  gz_offset=(int16_t) -z_gyro;
  
  
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  Serial.print("X, Y, Z offset : ");
  Serial.print(accelgyro.getXGyroOffsetUser());
  Serial.print(", ");
  Serial.print(accelgyro.getYGyroOffsetUser());
  Serial.print(", ");
  Serial.print(accelgyro.getZGyroOffsetUser());
  Serial.println(" ");
 
}
/*
void calibrate_gyro() {
  int gx_offset, gy_offset, gz_offset;

  gx_offset=-x_gyro/4;
  gy_offset=-y_gyro/4;
  gz_offset=-z_gyro/4;
  while (1){
    int ready=0;

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    Serial.print("X, Y, Z offset : ");
    Serial.print(accelgyro.getXGyroOffset());
    Serial.print(", ");
    Serial.print(accelgyro.getYGyroOffset());
    Serial.print(", ");
    Serial.print(accelgyro.getZGyroOffset());
    Serial.println(" ");
    Serial.println("...");

    mean_gyro();

    if (abs(x_gyro)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-x_gyro/(gyro_deadzone+1);

    if (abs(y_gyro)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-y_gyro/(gyro_deadzone+1);

    if (abs(z_gyro)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-z_gyro/(gyro_deadzone+1);

    if (ready==6) break;
  }
}
*/

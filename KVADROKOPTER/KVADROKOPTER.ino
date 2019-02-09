// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>
#include <PID_v1.h>
// множитель фильтра
#define BETA 0.22
 
// создаём объект для фильтра Madgwick
Madgwick filter;
 
// создаём объект для работы с акселерометром
Accelerometer accel;
// создаём объект для работы с гироскопом
Gyroscope gyro;
// создаём объект для работы с компасом
Compass compass;

Barometer bar;
 
// переменные для данных с гироскопа, акселерометра и компаса
float gx, gy, gz, ax, ay, az, mx, my, mz;
 
// получаемые углы ориентации (Эйлера)
double yaw, pitch, roll;
 
// переменная для хранения частоты выборок фильтра
float fps = 100;

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

double pdif, rdif;
double setpitch, setroll;
//double pitch, roll;
#define UP 230
int spd = UP; 

PID pitchPID(&pitch, &pdif, &setpitch,2,5,1, DIRECT);
PID rollPID(&roll, &rdif, &setroll,2,5,1, DIRECT);
// калибровочные значения компаса
// полученные в калибровочной матрице из примера «compassCalibrateMatrixx»
const double compassCalibrationBias[3] = {
  524.21,
  3352.214,
  -1402.236
};
 
const double compassCalibrationMatrix[3][3] = {
  {1.757, 0.04, -0.028},
  {0.008, 1.767, -0.016},
  {-0.018, 0.077, 1.782}
};

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


 
void setup()
{
  // открываем последовательный порт
  Serial.begin(115200);
  Serial.println("Begin init...");
  // инициализация акселерометра
  accel.begin();
  // инициализация гироскопа
  gyro.begin();
  // инициализация компаса
  compass.begin();
  bar.begin();
  // калибровка компаса
  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
  // выводим сообщение об удачной инициализации

// configure stabilization code
    //=============================
    pdif = 0.0;
    rdif = 0.0;
    setpitch = 0.0;
    setroll = 0.0;
    pitch = 0.0;
    roll = 0.0;
    
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);   
    
    pitchPID.SetOutputLimits(-30, 30);
    rollPID.SetOutputLimits(-30, 30);

  
  Serial.println("Initialization completed");
}

const int prop1 = 6;
const int prop2 = 9;
const int prop3 = 11;
const int prop4 = 10;


 
void loop()
{
  // запоминаем текущее время

//while (!mpuInterrupt) {
          
          
//          Serial.print("p1:\t");
//          Serial.print(p1);
//          Serial.print("p2:\t");
//          Serial.print(p2);
//          Serial.print("p4:\t");
//          Serial.print(p3);
//          Serial.print("p4:\t");
//          Serial.println(p4);

         
      
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    //}
int p1 = spd + (rdif / 2) + (pdif / 2);
          int p2 = spd - (rdif / 2) + (pdif / 2);
          int p3 = spd + (rdif / 2) - (pdif / 2);
          int p4 = spd - (rdif / 2) - (pdif / 2);
          
          if(p1 >= 255){
            //reduce speed so can stabalize
            spd -= (p1 - 255);
            //continue;
          }//else
          if(p2 >= 255){
            spd -= (p2 - 255);
            //continue;
          }//else
          if(p3 >= 255){
            spd -= (p3 - 255);
            //continue;
          }//else
          if(p4 >= 255){
            spd -= (p4 - 255);
            //continue;
          }
  Serial.print("\t\t");
  Serial.print("p1: ");
  Serial.print(p1);
  Serial.print("\t\t");
  Serial.print("p2: ");
  Serial.print(p2);
  Serial.print("\t\t");
  Serial.print("p3: ");
  Serial.print(p3);
  Serial.print("\t\t");
  Serial.print("p4: ");
  Serial.println(p4);
          analogWrite(prop1, p1);
          analogWrite(prop2, p2);
          analogWrite(prop3, p3);
          analogWrite(prop4, p4);
    
  mpuInterrupt = false;
  unsigned long startMillis = millis();
 
  // считываем данные с акселерометра в единицах G
  accel.readGXYZ(&ax, &ay, &az);
  // считываем данные с гироскопа в радианах в секунду
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);
  // считываем данные с компаса в Гауссах
  compass.readCalibrateGaussXYZ(&mx, &my, &mz);
 
  // устанавливаем коэффициенты фильтра
  filter.setKoeff(fps, BETA);
  // обновляем входные данные в фильтр
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
 
  // получение углов yaw, pitch и roll из фильтра
  yaw =  filter.getYawRad();
  pitch = filter.getPitchRad();
  roll = filter.getRollRad();
 
  // выводим полученные углы в serial-порт
  Serial.print("yaw: ");
  Serial.print(yaw);
  Serial.print("\t\t");
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print("\t\t");
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print("\t\t");
  Serial.print("bar: ");
  Serial.print(bar.readPressureMillimetersHg());
 
  // вычисляем затраченное время на обработку данных
  unsigned long deltaMillis = millis() - startMillis;
  // вычисляем частоту обработки фильтра
  fps = 1000 / deltaMillis;
  pitchPID.Compute();
  rollPID.Compute();

  
  
  if(spd < UP){
        spd++;
  }
}

#include <Arduino.h>

#define M1 4
#define M2 3
#define M3 8
#define M4 7
#define S1 5
#define S2 6
#define BUTTON_PIN 2
#define US_ARDUINO_PIN 13
#define IRD_PIN_1 12
#define IRD_PIN_2 11

///////////////////////////////////   КОНФИГУРАЦИЯ  /////////////////////////

#define ACTIVE_BOTTOM
#define CONF_LINE

//Минимальные и максимальные значения поля 
int minArr[8] = {21, 24, 24, 22, 24, 22, 23, 27};
int maxArr[8] = {960, 964, 964, 954, 954, 948, 934, 935};

//Данные датчиков
int data[8] = {0};
//Коэфициенты для пид
int weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

float new_irr_part = 1 / 20;

byte line[8] = {A7, A6, A5, A4, A3, A2, A1, A0};


///////////////////////////////////      МОТОР    /////////////////////////////

void motor(bool directoin_left, byte speed_left, bool direction_right, byte speed_right)
{ // функция управления двигателями(направление левого, скорость левого, направление правого, скрость правого) 1 это вперед, 0 это назад
    if (directoin_left == 1)
    {
        digitalWrite(M1, 0); // задаем направление движения
        digitalWrite(M2, 1); // задаем скорость вращения
        analogWrite(S1, speed_left);
    }
    if (directoin_left == 0)
    {
        digitalWrite(M1, 1); // задаем направление движения
        digitalWrite(M2, 0); // задаем скорость вращения
        analogWrite(S1, speed_left);
    }
    if (direction_right == 1)
    {
        digitalWrite(M3, 0); // задаем направление движения
        digitalWrite(M4, 1); // задаем скорость вращения
        analogWrite(S2, speed_right);
    }
    if (direction_right == 0)
    {
        digitalWrite(M3, 1); // задаем направление движения
        digitalWrite(M4, 0); // задаем скорость вращения
        analogWrite(S2, speed_right);
    }
}

///////////////////////////////////   АВТОПИЛОТ  //////////////////////////////

#define mx 255B
#define mn 0

int speed = 160;
float  kp = 1.24;
float  kd = 3.0;
float  ki = 1.0;

float old_error = 0, hist_error = 0;
float low_speed = speed;

float get_error()
{
    long long ch = 0, zn = 0;
    for (int i = 0; i < 8; i++)
    {
        ch += 1ll * data[i] * weight[i];
        zn += data[i];
    }
    float e = 0.25 * ch / zn;
    return e;
}


float pid(float error)
{
    float up, ud, ui;
    up = kp * error;
    ud = kd * (error - old_error);
    ui = ki * hist_error;

    hist_error = hist_error * (1 - new_irr_part) + error * new_irr_part;
    old_error = error;

    return up + ud + ui;
}

void move(float change){
    motor(1, low_speed - change, 1, low_speed + change);
}

///////////////////////////////   ДАТЧИКИ   ///////////////////////////////////////

void upd_data()
{
    for (int i = 0; i < 8; i++)
        data[i] = analogRead(line[i]);

    #ifdef CONF_LINE
    for(int i=0; i<8; i++){
        int chisl = min(max(data[i] - minArr[i],   0), 1000);
        int znam  = min(max(maxArr[i] - minArr[i], 0), 1000);
        data[i] = 1000.0 * chisl / znam;
    }
    #endif
}

///////////////////////////////////   ОБЩАЯ   ///////////////////////////////////

void setup()
{
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    Serial.begin(9600);
    
    #ifdef ACTIVE_BOTTOM
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    while (digitalRead(BUTTON_PIN));
    #endif
}

float braking_cof = 0.0;

void loop()
{
    #ifdef ACTIVE_BOTTOM
    if (!digitalRead(BUTTON_PIN)){
        braking_cof += 0.13;
        low_speed = min(speed, speed * braking_cof*braking_cof);
    }
    else{
        low_speed = 0;
        braking_cof = 0.0;
    }
    #endif

    upd_data();
    move(pid(get_error()));
}

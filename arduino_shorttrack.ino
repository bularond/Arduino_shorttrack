#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

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

//#define ACTIVE_BOTTOM
#define CONF_LINE
#define ACTIVE_BLUETOUTH
#define ENABLE_EEPROM

bool DATA_DEBUG = false;
bool MOVE_DEBUG = false;

//Минимальные и максимальные значения поля 
int minArr[8] = {21, 24, 24, 22, 24, 22, 23, 27};
int maxArr[8] = {960, 964, 964, 954, 954, 948, 934, 935};

//Данные датчиков
int data[8] = {0};
//Коэфициенты для пид
int weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

float new_irr_part = 1 / 20;

byte line[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

#ifdef ACTIVE_BLUETOUTH
#define Serial bt
SoftwareSerial bt(11, 12);
#endif

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

    if(DATA_DEBUG){
        for(int i = 0; i < 8; i++){
            Serial.print(data[i]);
            Serial.print(' ');
        }
        Serial.print('\n');
    }
}

///////////////////////////////////   АВТОПИЛОТ  //////////////////////////////

#define mx 255B
#define mn 0

int speed = 80;
float  kp = 1.25;
float  kd = 3.0;
float  ki = 4.0;

float old_error = 0, hist_error = 0;
float now_speed = speed;

void search(byte direction)
{
    motor(direction ^ 1, 120, direction, 120);
    int cnt = 0;
    while(cnt < 2){
        cnt = 0;
        upd_data();
        for(int i = 0; i < 8; i++)
            if(data[i] > 300)
                cnt++;
    }
}

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

int last_dir = 0;

void move(float regulator)
{
    int sum = 0, cnt = 0;
    for(int i = 0; i < 8; i++)
        if(data[i] > 300)
        {
            sum += weight[i];
            cnt++;
        }
    if(cnt == 0 && now_speed != 0){
        search(last_dir);
    }
    else if(abs(sum) == 4 && cnt == 1 && now_speed != 0){
        if(sum == -4){
            search(0);
            last_dir = 0;
        }
        else{
            last_dir = 1;
            search(1);
        }
    }
    else{
        float speed_left  =  min(max(now_speed * (1 - regulator), 0), 255);
        float speed_right =  min(max(now_speed * (1 + regulator), 0), 255);

        if(speed_left > speed_right)
            last_dir = 0;
        else
            last_dir = 1;
        motor(1, speed_left, 1, speed_right);

        if(MOVE_DEBUG){
            Serial.print(speed_left);
            Serial.print(' ');
            Serial.print(speed_right);
            Serial.print('\n');
        }
    }
}

///////////////////////////////////   BLUETOUTH /////////////////////////////////

int menu = 0;
String input;

float string_to_float(String st, int beg = 0){
    st.remove(0, beg);
    return st.toFloat();
}

void interface(){
    if(Serial.available()){
        motor(0, 0, 0, 0);
        input = Serial.readString();
        switch (input[0])
        { 
        case 'p':
            if(input[1] == '+')
                kp += 0.1;
            else if(input[1] == '-')
                kp -= 0.1;
            else
                kp = string_to_float(input, 1);
            Serial.println(kp);
            break;
        case 'd':
            if(input[1] == '+')
                kd += 0.1;
            else if(input[1] == '-')
                kd -= 0.1;
            else
                kd = string_to_float(input, 1);
            Serial.println(kd);
            break;
        case 'i':
            if(input[1] == '+')
                ki += 0.1;
            else if(input[1] == '-')
                ki -= 0.1;
            else
                ki = string_to_float(input, 1);
            Serial.println(ki);
            break;
        case 's':
            if(input[1] == '+')
                speed += 5;
            else if(input[1] == '-')
                speed -= 5;
            else
                speed = string_to_float(input, 1);
            if(now_speed != 0)
                now_speed = speed;
            Serial.println(speed);
            break;
        case 'c':
            if(now_speed == 0)
                now_speed = speed;
            else
                now_speed = 0;
            break;
        case '1':
            motor(1, 100, 1, 100);
            delay(int(string_to_float(input, 1)));
            break;
        case '2':
            motor(0, 100, 0, 100);
            delay(int(string_to_float(input, 1)));
            break;
        case '3':
            motor(0, 100, 1, 100);
            delay(int(string_to_float(input, 1)));
            break;
        case '4':
            motor(1, 100, 0, 100);
            delay(int(string_to_float(input, 1)));
            break;
        case 'g':
            Serial.print(kp);
            Serial.print(' ');
            Serial.print(kd);
            Serial.print(' ');
            Serial.print(ki);
            Serial.print(' ');
            Serial.print(speed);
            Serial.print('\n');
            break;
        case 'f':
            search(last_dir);
            break;
        #ifdef ENABLE_EEPROM
        case 'l':
            int addr = 0;
            EEPROM.put(addr, kp);
            addr += sizeof(kp);
            EEPROM.put(addr, kd);
            addr += sizeof(kd);
            EEPROM.put(addr, ki);
            addr += sizeof(ki);
            EEPROM.put(addr, speed);
            break;
        #endif
        default:
            break;
        }
    }
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

    #ifdef ENABLE_EEPROM
    int addr = 0;
    EEPROM.get(addr, kp);
    addr += sizeof(kp);
    EEPROM.get(addr, kd);
    addr += sizeof(kd);
    EEPROM.get(addr, ki);
    addr += sizeof(ki);
    EEPROM.get(addr, speed);
    #endif

    #ifdef ACTIVE_BOTTOM
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    while (digitalRead(BUTTON_PIN));
    #endif
    now_speed = 0;
}

float braking_cof = 0.0;

void loop()
{
    interface();
    upd_data();
    move(pid(get_error()));
}

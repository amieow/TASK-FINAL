#include "main.h"

const int mac_index_ku = FAUZAN_FIRDAUS; //....

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    mulai_esp_now(mac_index_ku);
}

void loop()
{
    if (Serial.available())
    {
        baca_serial(callback_data_serial);
    }
}
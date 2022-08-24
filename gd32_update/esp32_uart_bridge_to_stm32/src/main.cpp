#include <Arduino.h>


void setup() {
    Serial.begin(9600);
    while(!Serial);
    Serial2.begin(9600, SERIAL_8N1 , 16, 17, false);
    while(!Serial2);

    pinMode(26, OUTPUT); // OUT_MCP
    digitalWrite(26, 1);
    pinMode(18, OUTPUT); // gd32- BOOT0
    digitalWrite(18, 1);
    pinMode(23, OUTPUT); // ENABLE
    delay(250);
    digitalWrite(23, 1);

    pinMode(1, OUTPUT); // TXD0
    pinMode(3, INPUT); // RXD0
    pinMode(17, OUTPUT); // TX1
    pinMode(16, INPUT); // RX1
}


void loop() {
    digitalWrite(17, digitalRead(3));
    digitalWrite(1, digitalRead(16));
}

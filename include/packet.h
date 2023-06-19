#include <Arduino.h>

#define TYPEAPPLICATION 0xFFB1 // identificativo uguale per tutti i dispositivi sensore

// 20 bytes
struct LoRaPacket{
    uint16_t type; // identificativo del device all'interno della propria rete - [0,1]
    uint16_t id; // identificativo del nodo - [2,3]
    float temperature; // [4,7]
    float pressure; // [8,11]
    float humidity; // [12,15]
    float gas; // [16,19]
};
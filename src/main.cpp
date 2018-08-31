/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Henri van de Munt
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * LoRaWAN bin sensor: This pice of software measures the distance to waste and 
 * sends a valid LoRaWAN packet with the measured distance and the battery 
 * voltage. 
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L0X.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0xC0, 0x0C, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x73, 0x99, 0x8E, 0xFD, 0x71, 0x9C, 0xBB, 0xFE, 0x74, 0xBB, 0xB3, 0x21, 0x0A, 0x22, 0x97, 0x57};

void os_getArtEui(u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);

    Serial.print("AppEui: ");
    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.print(buf[i], HEX);
    }
    Serial.println();
}

void os_getDevEui(u1_t *buf)
{
    bool deveui_generated = true;
    u1_t deveui[8];

    // Check if DevEui can be found in EEPROM, this is true if the first 16 bits correspond to the 16 bits of the appkey
    for (uint16_t i = 0; i < 16; i++)
    {
        if (EEPROM.read(i) != pgm_read_byte_near(APPKEY + i))
        {
            deveui_generated = false;
        }
    }

    if (deveui_generated)
    {
        // Found DevEui in EEPROM
        Serial.println("Found DevEui in EEPROM");

        // read the deveui form the eeprom
        for (uint8_t i = 16; i < 24; i++)
        {
            deveui[i - 16] = EEPROM.read(i);
        }
    }
    else
    {
        // No DevEui found in EEPROM, generate one and write

        Serial.println("Generating DevEui and writing to EEPROM");

        for (uint8_t i = 0; i < 8; i++)
        {
            deveui[i] = random(0, 255);
        }

        // write APPKEY to eeprom followed by the generated deveui

        for (uint8_t i = 0; i < 16; i++)
        {
            EEPROM.write(i, pgm_read_byte_near(APPKEY + i));
        }

        for (uint8_t i = 16; i < 24; i++)
        {
            EEPROM.write(i, deveui[i - 16]);
        }
    }

    memcpy(buf, deveui, 8);

    Serial.print("DevEui: ");
    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.print(buf[i], HEX);
    }
    Serial.println();
}

void os_getDevKey(u1_t *buf)
{
    memcpy_P(buf, APPKEY, 16);

    Serial.print("DevKey: ");
    for (uint8_t i = 0; i < 16; i++)
    {
        Serial.print(buf[i], HEX);
    }
    Serial.println();
}

VL53L0X sensor;

static uint8_t payload[20];
static osjob_t sendjob;

int times, rest;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60 * 15;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

long readVcc();
void do_send(osjob_t *j);
void onEvent(ev_t ev);

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        uint32_t i = 0;

        // read battery vcc
        long batteryLevel_mV = readVcc();

        // add battery vcc to payload
        payload[i++] = (batteryLevel_mV >> 8) & 0xFF; //level of battery in mV
        payload[i++] = batteryLevel_mV & 0xFF;

        // read sensor
        uint16_t distance = sensor.readRangeSingleMillimeters();

        Serial.println(distance);

        // add sensor value to payload
        if (!sensor.timeoutOccurred())
        {
            payload[i++] = (distance >> 8) & 0xFF;
            payload[i++] = distance & 0xFF;
        }

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, i, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }

        // enter sleep

        times = (TX_INTERVAL - 5) / 8;
        rest = (TX_INTERVAL - 5) % 8;

        Serial.print("Sleeping ");
        Serial.print(times);
        Serial.print(" times of 8 seconds. Plus: ");
        Serial.print(rest);
        Serial.println(" seconds.");

        Serial.flush();
        for (int i = 0; i < times; i++)
        {
            // Enter power down state for 8 s with Analog Digital Converter and Brown-Out Detection module disabled
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }

        if (rest >= 4)
        {
            LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
            rest = rest - 4;
        }
        if (rest >= 2)
        {
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            rest = rest - 2;
        }
        if (rest >= 1)
        {
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
            rest = rest - 1;
        }

        // Schedule next transmission
        do_send(&sendjob);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Starting"));

    delay(100);

    // set random seed for generating deveui if neccesery
    randomSeed(analogRead(0));

    // setup sensor
    sensor.init();
    sensor.setTimeout(500);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Let LMIC compensate for +/- 1% clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}

long readVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result;              // Vcc in millivolts
}
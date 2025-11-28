#include "NMEA_MUX.h"

#include <Arduino.h>
#include <FIFO.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <string.h>
#if defined(__AVR_ATmega328P__)
#include <ArduinoUniqueID.h>
#endif
#include <ArduinoMqttClient.h>

#include "utils.h"

#ifdef WEB_GUI
EthernetServer server1(80);
#endif

fifo_t msgout_buf;
volatile bool static_send;

struct nmea {
    char id[6];
    char sentence[79];
};

#ifdef TRANSLATE
#define NMEA_DPT "DPT"
#endif

extern "C" {
void start();
void stop();
void led_on();
void led_off();
void led_toggle();
#ifdef DEBUG
void UART0_Init(uint16_t baud);
void UART0_Transmit(unsigned char data);
void UART0_Print(const char* str);
void UART0_Print_P(const char* str);
void UART0_Println(const char* str);
void UART0_Println_P(const char* str);
void UART0_Flush();
void UART0_Print_num(int16_t num, uint8_t base);
#endif
}

EthernetClient c;
MqttClient mqttClient(c);
void connectMQTT(char* broker, uint16_t port);
struct nmea* extractNMEA(char* nmea_line);
void loadParamsFromEEPROM(void);
void saveParamsToEEPROM(void);

char mqtt_broker[] = "192.168.000.253";  // fully set all digits for memory
int mqtt_port = 1883;

char update_host[32];

uint8_t admin_timeout;

/**
 * Init the soft- and hardware.
 * Define variables, and load settings from EEPROM, if they have been stored, otherwise load defaults.
 */
#ifdef DEBUG
char c_buf[128];  // global var for sprintf stuff

#define printsf(txt, num)     \
    sprintf(c_buf, txt, num); \
    UART0_Print(c_buf)
#endif
void setup() {
#if defined(__AVR_ATmega328P__)
    const uint32_t uid = ((uint32_t)UniqueID8[4] << 24) + ((uint32_t)UniqueID8[5] << 16) + ((uint32_t)UniqueID8[6] << 8) + UniqueID8[7];
#endif
    wdt_enable(WDTO_8S);

#ifdef DEBUG

    uint16_t ubrr = (F_CPU / (DEBUG_BAUD * 8)) - 1;
    UART0_Init(ubrr);

    printsf("%u - DEBUG enabled\r\n", BUILD);
    UART0_Flush();
#endif

    randomSeed(analogRead(0));  // init random engine
    build = BUILD;

    // Some Hardware stuff
    start();
#ifdef RESET
    for (uint32_t i = 0; i < 4096; i++) {
        eeprom_write_byte((uint8_t*)(i * 4), 0xffffffff);
    }
#endif

    p_config[P1].ofilter.pm_all = 1;
    p_config[P1].ifilter.pm_all = 1;
    p_config[P1].baud = baud[BAUD_4800];
    p_config[P1].parameter = params[0];
    p_config[P1].direction = BOTH;

    p_config[P2].ofilter.pm_all = 1;
    p_config[P2].ifilter.pm_all = 1;
    p_config[P2].baud = baud[BAUD_9600];
    p_config[P2].parameter = params[0];
    p_config[P2].direction = BOTH;

    p_config[P3].ofilter.pm_all = 1;
    p_config[P3].ifilter.pm_all = 1;
    p_config[P3].baud = baud[BAUD_4800];
    p_config[P3].parameter = params[0];
    p_config[P3].direction = BOTH;

    p_config[P4].ofilter.pm_all = 1;
    p_config[P4].ifilter.pm_all = 1;
    p_config[P4].baud = baud[BAUD_38400];
    p_config[P4].parameter = params[0];
    p_config[P4].direction = BOTH;

    p_config[PETH1].ofilter.pm_all = 1;
    p_config[PETH1].ifilter.pm_all = 1;
    p_config[PETH1].direction = BOTH;
    port1 = SERVER_PORT;

    debug = 0;
    // Network settings
    ip_address[0] = ip_address[1] = ip_address[2] = ip_address[3] = 0;
    dns_a[0] = dns_a[1] = dns_a[2] = dns_a[3] = 0;
    gateway[0] = gateway[1] = gateway[2] = gateway[3] = 0;
    subnet[0] = subnet[1] = subnet[2] = subnet[3] = 0;

    strcpy(hostname, HOST_NAME);

    admin_timeout = ADMIN_TIMEOUT;

    uint16_t crc = eeprom_read_word(0);  // Get the CRC from EEPROM, if.
#ifdef DEBUG
    printsf("%x\r\n", crc);
    uint8_t l = 0;  // EEPROM data loaded flag
#endif

    /**
     * Check, if the factory reset button has been pressed during startup. If so, invalidate the CRC value in EEPROM.
     * All settings will be set to defaults.
     */
    // Button pressed
    if (!(PINL & _BV(RESET_BTN_PIN))) {
        crc -= 1;                   // void CRC
        eeprom_write_word(0, crc);  // and store to EEPROM
        // Blink for confirmation
        for (uint8_t i = 0; i < 10; i++) {
            led_toggle();
            delay(150);
        }
    }

    bool loaded_from_sd = false;
#ifdef SDC
    SPI.begin();
    SD.begin(SD_CS);

    has_sd = card.init(SPI_FULL_SPEED, SD_CS);  // This flag is needed to enable or disable GUI settings.

    if (has_sd) {
#ifdef DEBUG

        switch (card.type()) {
            case SD_CARD_TYPE_SD1:
                UART0_Println("SD1");
                break;
            case SD_CARD_TYPE_SD2:
                UART0_Println("SD2");
                break;
            case SD_CARD_TYPE_SDHC:
                UART0_Println("SDHC");
                break;
            default:
                UART0_Println("Unknown card type");
        }
        if (!volume.init(card)) {
            const static char txt[] PROGMEM = "Could not find FAT16/FAT32 partition.\r\nMake sure you've formatted the card";
            UART0_Println_P(txt);
        } else {
            printsf("Clusters:          %lu\r\n", volume.clusterCount());
            printsf("Blocks x Cluster:  %u\r\n", volume.blocksPerCluster());
            printsf("Total Blocks:      %lu\r\n\r\n", volume.blocksPerCluster() * volume.clusterCount());
            uint32_t volumesize;
            printsf("Volume type is:    FAT%u\r\n", volume.fatType());
            volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
            volumesize *= volume.clusterCount();     // we'll have a lot of clusters
            volumesize /= 2;                         // SD card blocks are always 512 bytes (2 blocks are 1KB)
            printsf("Volume size (Kb):  %lu\r\n", volumesize);
            volumesize /= 1024;
            printsf("Volume size (Mb):  %lu\r\n", volumesize);
            char d_buf[5];
            dtostrf((float)volumesize / 1024.0, 4, 1, d_buf);
            printsf("Volume size (Gb):  %s\r\n", d_buf);
            const static char txt[] PROGMEM = "\nFiles found on the card (name, date and size in bytes): ";
            UART0_Println_P(txt);
            UART0_Flush();
            root.openRoot(volume);
            // list all files in the card with date and size
            root.ls(LS_R | LS_DATE | LS_SIZE);
            root.close();
        }
#endif
        // SD config load
        // If we have a config file, configs listet there superseed the EEPROM data
        has_config = SD.exists(CONFIG_FILE);

        if (has_sd && has_config) {
            config_file = SD.open(CONFIG_FILE);
            uint8_t store = 0;

            if (config_file.available()) {
                const size_t bufferLen = 80;
                char buffer[bufferLen];
                // char* filename = (char*)malloc(sizeof(CONFIG_FILE) + 1);
                // strcpy(filename, CONFIG_FILE);
                IniFile ini(CONFIG_FILE);
                ini.open();
                if (ini.validate(buffer, bufferLen)) {
#ifdef DEBUG
                    const static char txt[] PROGMEM = "[general]";
                    UART0_Println_P(txt);
#endif
                    /* if (ini.getValue("general", "debug", buffer, bufferLen)) {
#ifdef DEBUG
                            // Serial.print(F("debug = "));
                            // Serial.println(buffer);
#endif
                            debug = !strcmp(buffer, "yes");
                    } */
                    if (ini.getValue("mqtt", "host", buffer, bufferLen)) {
                        strcpy(mqtt_broker, buffer);
#ifdef DEBUG
                        const static char txt1[] PROGMEM = "host = ";
                        UART0_Print_P(txt1);
                        UART0_Println(buffer);
#endif
                    }
                    if (ini.getValue("mqtt", "port", buffer, bufferLen)) {
                        mqtt_port = atoi(buffer);
#ifdef DEBUG
                        const static char txt1[] PROGMEM = "port = ";
                        UART0_Print_P(txt1);
                        UART0_Println(buffer);
#endif
                    }
                    if (ini.getValue("network", "hostname", buffer, bufferLen)) {
                        strcpy(hostname, buffer);
#ifdef DEBUG
                        const static char txt1[] PROGMEM = "hostname = ";
                        UART0_Print_P(txt1);
                        UART0_Println(buffer);
#endif
                    } else {
#if defined(__AVR_ATmega328P__)
                        char new_hostname[20];
                        sprintf(new_hostname, HOST_NAME, uid);
                        strcpy(hostname, buffer);
                        Ethernet.setHostname(new_hostname);
#endif
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_6);
                        printErrorMessage(ini.getError());
                        const static char txt1[] PROGMEM = "hostname = ";
                        UART0_Print_P(txt1);
                        UART0_Println(hostname);
#endif
                    }
                    if (ini.getValue("network", "mac", buffer, bufferLen)) {
                        char* p = strtok(buffer, ":");
                        uint8_t j = 0;
                        while (p != NULL) {
                            mac[j++] = hexStr2Int(p);
                            p = strtok(NULL, ":");
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_1);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("network", "ip address", buffer, bufferLen)) {
                        char* p = strtok(buffer, ".");
                        uint8_t j = 0;
                        while (p != NULL) {
                            ip_address[j++] = atoi(p);
                            p = strtok(NULL, ".");
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_2);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("network", "gateway", buffer, bufferLen)) {
                        char* p = strtok(buffer, ".");
                        uint8_t j = 0;
                        while (p != NULL) {
                            gateway[j++] = atoi(p);
                            p = strtok(NULL, ".");
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_3);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("network", "dns", buffer, bufferLen)) {
                        char* p = strtok(buffer, ".");
                        uint8_t j = 0;
                        while (p != NULL) {
                            dns_a[j++] = atoi(p);
                            p = strtok(NULL, ".");
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_4);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("network", "subnet", buffer, bufferLen)) {
                        char* p = strtok(buffer, ".");
                        uint8_t j = 0;
                        while (p != NULL) {
                            subnet[j++] = atoi(p);
                            p = strtok(NULL, ".");
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_5);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("general", "admin_timeout", buffer, bufferLen)) {
#ifdef DEBUG
                        const static char txt1[] PROGMEM = "admin_timeout = ";
                        UART0_Print_P(txt1);
                        UART0_Println(buffer);
#endif
                        admin_timeout = atoi(buffer);
                    }
                    if (ini.getValue("general", "store", buffer, bufferLen)) {
#ifdef DEBUG
                        const static char txt1[] PROGMEM = "store = ";
                        UART0_Print_P(txt1);
                        UART0_Println(buffer);
#endif
                        store = !strcmp(buffer, "yes");
                        if (!strcmp(buffer, "invalidate")) {
                            eeprom_write_word(0, crc - 1);
                        }
                    } else {
#ifdef DEBUG
                        UART0_Print_P(SD_ERROR_6);
                        printErrorMessage(ini.getError());
#endif
                    }
                    if (ini.getValue("general", "update_host", buffer, bufferLen)) {
                        strcpy(update_host, buffer);
                    } else {
                        strcpy(update_host, DEFAULT_UPDATE_HOST);
                    }
                    for (uint8_t i = 0; i < PORTS; i++) {
                        char port[6] = {'p', 'o', 'r', 't', (char)(i + 49), 0};
#ifdef DEBUG
                        UART0_Transmit('[');
                        UART0_Print(port);
                        UART0_Transmit(']');
                        UART0_Println("");
#endif
                        if (ini.getValue(port, "baud", buffer, bufferLen)) {
                            p_config[i].baud = atol(buffer);
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "baud = ";
                            UART0_Print_P(txt1);
                            UART0_Println(buffer);
#endif
                        } else {
                            p_config[i].baud = baud[0];
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Could not read 'baud' from section '";
                            UART0_Print_P(txt1);
                            UART0_Print(port);
                            const static char txt2[] PROGMEM = "', error was ";
                            UART0_Print_P(txt2);
#endif
                            printErrorMessage(ini.getError());
                        }
                        if (ini.getValue(port, "ifilter", buffer, bufferLen)) {
                            char* p = (char*)malloc(bufferLen);
                            p = strtok(buffer, ":");
                            uint8_t j = 0;
                            while (p != NULL) {
                                if (!strcmp(p, "-all")) {
                                    p_config[i].ifilter.pm_all = 0;
                                } else if (!strcmp(p, "+all")) {
                                    p_config[i].ifilter.pm_all = 1;
                                } else {
                                    strcpy(p_config[i].ifilter.pattern[j++], p);
                                }
                                p = strtok(NULL, ":");
                            }
                            free(p);
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "ifilter = ";
                            UART0_Print_P(txt1);
                            UART0_Println(buffer);
#endif
                        } else {
                            p_config[i].ifilter.pm_all = 1;
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Could not read 'ifilter' from section '";
                            UART0_Print_P(txt1);
                            UART0_Print(port);
                            const static char txt2[] PROGMEM = "', error was ";
                            UART0_Print_P(txt2);
                            printErrorMessage(ini.getError());
#endif
                        }
                        if (ini.getValue(port, "ofilter", buffer, bufferLen)) {
                            char* p = (char*)malloc(bufferLen);
                            p = strtok(buffer, ":");
                            uint8_t j = 0;
                            while (p != NULL) {
                                if (!strcmp(p, "-all")) {
                                    p_config[i].ofilter.pm_all = 0;
                                } else if (!strcmp(p, "+all")) {
                                    p_config[i].ofilter.pm_all = 1;
                                } else {
                                    strcpy(p_config[i].ofilter.pattern[j++], p);
                                }
                                p = strtok(NULL, ":");
                            }
                            free(p);
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "ofilter = ";
                            UART0_Print_P(txt1);
                            UART0_Println(buffer);
#endif
                        } else {
                            p_config[i].ofilter.pm_all = 1;
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Could not read 'ofilter' from section '";
                            UART0_Print_P(txt1);
                            UART0_Print(port);
                            const static char txt2[] PROGMEM = "', error was ";
                            UART0_Print_P(txt2);
                            printErrorMessage(ini.getError());
#endif
                        }
                        if (ini.getValue(port, "direction", buffer, bufferLen)) {
                            if (!strcmp(buffer, "in")) {
                                p_config[i].direction = IN;
                            } else if (!strcmp(buffer, "out")) {
                                p_config[i].direction = OUT;
                            } else if (!strcmp(buffer, "both")) {
                                p_config[i].direction = BOTH;
                            } else if (!strcmp(buffer, "off")) {
                                p_config[i].direction = OFF;
                            }
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "direction = ";
                            UART0_Print_P(txt1);
                            UART0_Println(buffer);
#endif
                        } else {
                            p_config[i].direction = BOTH;
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Could not read 'direction' from section '";
                            UART0_Print_P(txt1);
                            UART0_Print(port);
                            const static char txt2[] PROGMEM = "', error was ";
                            UART0_Print_P(txt2);
                            printErrorMessage(ini.getError());
#endif
                        }
                        if (ini.getValue(port, "port", buffer, bufferLen)) {
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "port = ";
                            UART0_Print_P(txt1);
                            UART0_Println(buffer);
#endif
                            port1 = atoi(buffer);
                        } else {
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Could not read 'port' from section '";
                            UART0_Print_P(txt1);
                            UART0_Print(port);
                            const static char txt2[] PROGMEM = "', error was ";
                            UART0_Print_P(txt2);
#endif
                            printErrorMessage(ini.getError());
                        }
                    }
                }
                ini.close();
                // free(filename);
            }
            loaded_from_sd = true;
#ifdef DEBUG
            const static char txt1[] PROGMEM = "Loaded parameters from SD Card";
            UART0_Println_P(txt1);
#endif
            wdt_reset();
#endif
            if (store) {
                saveParamsToEEPROM();
#ifdef DEBUG
                const static char txt1[] PROGMEM = "Parameters saved to EEPROM";
                UART0_Println_P(txt1);
#endif
            }
        }
        config_file.close();
        SD.end();

#ifdef DEBUG
        l = 0;
#endif
    }
#ifdef DEBUG
    else {
        const static char txt1[] PROGMEM = "No SD card found";
        UART0_Println_P(txt1);
    }
#endif

    if (!loaded_from_sd) {
        // EEPROM config load
        // We want to get stored parameter from the EEPROM, but we have to check the validity of the stored data first
        // Now compare the value with the content
        const uint16_t e_crc = eeprom_crc();
#ifdef DEBUG
        printsf("%x\r\n", e_crc);
#endif
        if (e_crc == crc) {
            // Data in EEPROM seems to be valid. Load it.
            loadParamsFromEEPROM();
#ifdef DEBUG
            l = 1;
#endif
        }

#ifdef DEBUG
        if (l) {
            const static char txt[] PROGMEM = "Loaded configs from EEPROM";
            UART0_Println_P(txt);
        } else {
            const static char txt[] PROGMEM = "Set default config";
            UART0_Println_P(txt);
        }
#endif
    }
    wdt_reset();
    msgout_buf = fifo_create(NMEA_LINES, NMEA_LINE_LENGTH * sizeof(char));

    // Hardware init

#ifdef DEBUG
    UART0_Flush();
#endif
#ifndef DEBUG
    Serial.begin(p_config[P1].baud, p_config[P1].parameter);
#endif
    Serial1.begin(p_config[P2].baud, p_config[P2].parameter);
    Serial2.begin(p_config[P3].baud, p_config[P3].parameter);
    Serial3.begin(p_config[P4].baud, p_config[P4].parameter);

    wdt_reset();

#ifdef ETHERNET
    server = (EthernetServer*)malloc(sizeof(EthernetServer));
    server = new EthernetServer(port1);
#ifdef DEBUG
    const static char txt1[] PROGMEM = "Serials and EthernetServers defined";
    UART0_Println_P(txt1);
#endif
    // start the ethernet server
    uint8_t a = 0;

    digitalWrite(SS, HIGH);

    Ethernet.setHostname(hostname);
    wdt_reset();
    if ((ip_address[0] + ip_address[1] + ip_address[2] + ip_address[3]) != 0) {
        if ((dns_a[0] + dns_a[1] + dns_a[2] + dns_a[3]) != 0) {
            if ((gateway[0] + gateway[1] + gateway[2] + gateway[3]) != 0) {
                if ((subnet[0] + subnet[1] + subnet[2] + subnet[3]) != 0) {
                    Ethernet.begin(mac, ip_address, dns_a, gateway, subnet);
                    a = 1;
                }
                if (!a) {
                    Ethernet.begin(mac, ip_address, dns_a, gateway);
                    a = 1;
                }
            }
            if (!a) {
                Ethernet.begin(mac, ip_address, dns_a);
                a = 1;
            }
        }
        if (!a) {
            Ethernet.begin(mac, ip_address);
            a = 1;
        }
    } else {
        Ethernet.begin(mac);
    }

    server->begin();
#ifdef WEB_GUI
    server1.begin();
#endif
    wdt_reset();
#ifdef DEBUG
    const static char txt2[] PROGMEM = "EthernetServers started";
    UART0_Println_P(txt2);
#endif

#ifdef ETHERNET
#ifdef WEB_GUI

    // EthernetClient c;
    if (has_sd) {
        if (c.connect(update_host, 80)) {
#ifdef DEBUG
            const static char txt2[] PROGMEM = "Looking for updates";
            UART0_Println_P(txt2);
#endif
            c.println(F("GET /controller.php?device=MUX4s-1e HTTP/1.1"));
            c.print(F("User-Agent: MUX4s-1e/"));
            c.println(build, DEC);
            c.print(F("Host: "));  // devices.njk-it.de"));
            c.println(update_host);
            c.println(F("Connection: close"));
            c.println();  // end HTTP request header
            c.flush();

            char h_buf[128];
            uint8_t skipped = 0;
            while (c.connected() && !c.available()) delay(1);  // wait for datas
            // while (c.connected()) {
            while (c.connected() || c.available()) {
                uint16_t b_read = c.readBytesUntil('\n', h_buf, 128);
                h_buf[b_read] = 0;
#ifdef DEBUG
                UART0_Println(h_buf);
#endif

                if (!skipped && b_read <= 1) {
                    skipped = 1;
                    continue;
                } else if (skipped && b_read <= 1) {
                    continue;
                }

                if (h_buf[b_read - 1] == '\r') {
                    h_buf[b_read - 1] = 0;
                } else
                    h_buf[b_read] = 0;

                if (skipped) {
                    parseParameter(h_buf);
                }
            }
            // if (skipped) break;
            // }
            c.stop();

            if (build == build_available) {
                if (SD.exists(FIRMWARE_FILE)) {
                    SD.remove(FIRMWARE_FILE);
#ifdef DEBUG
                    const static char txt1[] PROGMEM = "Remove old firmware file from SD card.";
                    UART0_Println_P(txt1);
#endif
                }
            }
        }
    }
#endif
    if (Ethernet.link() == 1) {
#ifdef DEBUG
        const static char txt1[] PROGMEM = "Connecting to MQTT";
        UART0_Println_P(txt1);
#endif
        connectMQTT(mqtt_broker, mqtt_port);
    }
#endif

#endif

    char t[40];
    sprintf(t, "%s - MUX4s-1e started", hostname);
    sendFreeText(t);
    send2All();
#ifdef DEBUG
    printsf("%s - MUX4s-1e started\r\n", hostname);
    UART0_Flush();
#endif
    // Timer setup
    timerSetup();

    key = random(0xffff);

    // enable serial interrupts on Serial 1 for LED blinking
    UCSR0B |= _BV(TXCIE0);
}

/**
 * Send the buffer data to all ports.
 */
void send2All() {
    if (fifo_is_empty(msgout_buf)) {
        return;
    }

    char nmea_line[NMEA_LINE_LENGTH];
    fifo_get(msgout_buf, nmea_line);

    if (matchFilter(nmea_line, &(p_config[P1].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        if (nmea_line[0] & 0b11000010) {
            uint8_t t = nmea_line[0];
            nmea_line[0] &= 0b00111101;
            led_on();
#ifndef DEBUG            
            sendStream(Serial, P1, nmea_line);
#else
            UART0_Println(nmea_line);
#endif
            // led_off();
            nmea_line[0] = t;
        }
    }
    if (matchFilter(nmea_line, &(p_config[P2].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        if ((nmea_line[0] & 0b11000010) != 0b01000000) {
            uint8_t t = nmea_line[0];
            nmea_line[0] &= 0b00111101;
            sendStream(Serial1, P2, nmea_line);
            nmea_line[0] = t;
        }
    }
    if (matchFilter(nmea_line, &(p_config[P3].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        if ((nmea_line[0] & 0b11000010) != 0b10000000) {
            uint8_t t = nmea_line[0];
            nmea_line[0] &= 0b00111101;
            sendStream(Serial2, P3, nmea_line);
            nmea_line[0] = t;
        }
    }
    if (matchFilter(nmea_line, &(p_config[P4].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        if ((nmea_line[0] & 0b11000010) != 0b11000000) {
            uint8_t t = nmea_line[0];
            nmea_line[0] &= 0b00111101;
            sendStream(Serial3, P4, nmea_line);
            nmea_line[0] = t;
        }
    }
#ifdef ETHERNET
    // Is OUT active?
    if (Ethernet.link() == 1) {
        if ((p_config[PETH1].direction & OUT)) {
            if (matchFilter(nmea_line, &(p_config[PETH1].ofilter))) {
                // Check if it has come from the same source. Avoid to echo if so.
                if ((nmea_line[0] & 0b11000010) != 0b000000010) {
                    nmea_line[0] &= 0b00111101;
                    const char* ptr = strchr(nmea_line, '\0');
                    if (ptr) {
                        int16_t j = ptr - nmea_line;
                        server->write(nmea_line, j);
                    } else {
                        server->write(nmea_line, NMEA_LINE_LENGTH - 2);
                    }
                    // server.println("$GTEST*");
                    server->write(crlf, 2);  // line break
                    mesg_sent++;
                }
            }
        }

        if (mqttClient.connected()) {
            char buf[48];
            nmea_line[0] = '$';
            struct nmea* result = extractNMEA(nmea_line);
            if (strcmp(result->id, "NULL")) {
                sprintf(buf, "vessels/self/nmea0183/%s/%s", hostname, result->id);
                mqttClient.beginMessage(buf);
                mqttClient.print(result->sentence);
                mqttClient.endMessage();
            }
            free(result);
            // mqttClient.stop();
        }
    }

#endif
    /**
#ifdef SDC
    if (debug && has_sd) {
            nmea_out = SD.open(NMEA_OUT_FILE, FILE_WRITE);
            nmea_buffer[i][0] &= 0b00111101;
            sendStream(nmea_out, PETH1, i);
            nmea_out.close();
    }
#endif
*/
}

ISR(USART0_TX_vect) {
    led_off();
}

/**
 * Main LOOP
 */
void loop() {
    wdt_reset();
    /**
     * READING & WRITING
     */

    // Get the incoming data

    if (Serial.available()) {
        led_on();
        readStream(Serial, P1);
        led_off();
    }
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }
    readStream(Serial1, P2);
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }
    readStream(Serial2, P3);
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }
    readStream(Serial3, P4);
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }

#ifdef ETHERNET
    if (Ethernet.link() == 1) {
        EthernetClient client = server->available();  // returns first client which has data to read or a 'false' client
        if (client) {                                 // client is true only if it is connected and has data to read
            readStream(client, PETH1);
            send2All();
        }
#ifdef WEB_GUI
        /**
         * Only available the first 5 Minutes
         */

        client = server1.available();  // returns first client which has data to read or a 'false' client
        if (client) {                  // client is true only if it is connected and has data to read
                                       /* #ifndef DEBUG
                                                   UCSR0B &= ~_BV(TXCIE0);
                                                   Serial.end();
                                       #endif
                                                   Serial1.end();
                                                   Serial2.end();
                                                   Serial3.end(); */
            char html_buffer[HTML_BUFFER_SIZE];
            uint8_t reset = 0;
            uint8_t get = 0;
            uint8_t post = 0;
            uint8_t update = 0;
            //			uint8_t upload = 0;
            uint8_t download = 0;
            while (client.connected()) {
                wdt_reset();
                if (client.available()) {
                    uint8_t b_read = client.readBytesUntil('\n', html_buffer, HTML_BUFFER_SIZE);
                    uint8_t image = 0;
                    html_buffer[b_read] = 0;
#ifdef DEBUG
                    UART0_Println(html_buffer);
#endif
                    // Logo image
                    /*if (!strncmp(html_buffer, "GET /logo ", 10)) {
                            image = 1;
                            client.print((const __FlashStringHelper*) H3);
                            client.print(LOGO_SIZE, DEC);
                            client.println("\r\n");

                            PGM_P l = logo;
                            uint8_t fb; // flash byte
                            for (uint16_t c = 0; c < LOGO_SIZE; c++) {
                                    fb = pgm_read_byte(l++);
                                    client.write(fb);
                            }

                            // I can't stand to have 404 requests ;-)
                    } else */
                    if (!strncmp(html_buffer, "GET /favicon.ico ", 17)) {
                        image = 1;
                        client.print((const __FlashStringHelper*)HEAD_FAVICON_1);
                        client.print(FAVICON_SIZE, DEC);
                        client.println();
                        client.println();

                        PGM_P l = (char*)FAVICON;
                        uint8_t fb;  // flash byte
                        for (uint16_t c = 0; c < FAVICON_SIZE; c++) {
                            fb = pgm_read_byte(l++);
                            client.write(fb);
                        }
                        // main HTML page
                    } else if (!strncmp(html_buffer, "POST /update ", 13)) {
                        post = 1;
                        update = 1;
                    } else if (!strncmp(html_buffer, "POST /download ", 13)) {
                        post = 1;
                        download = 1;
                    } else if (!strncmp(html_buffer, "POST /reset ", 12)) {
                        post = 1;
                        reset = 1;
                    } else /*if (!strncmp(html_buffer, "POST /upload ", 13)) {
                            post = 1;
                            upload = 1;
                    } else */
                        if (!strncmp(html_buffer, "POST / ", 7)) {
                            post = 1;
                        } else if (!strncmp(html_buffer, "HEAD / ", 7)) {
                            client.print((const __FlashStringHelper*)HEAD);
                        } else if (!strncmp(html_buffer, "GET /", 5)) {
                            get = 1;
                        }

                    // ----- Parsing POST Data

                    if (b_read == EMPTY_LINE && post && download) {
                        EthernetClient c;

                        if (c.connect(update_host, 80)) {
                            c.print(F("GET /controller.php?device=MUX4s-1e&firmware="));
                            c.print(build_available, DEC);
                            c.println(F(" HTTP/1.1"));
                            c.print(F("Host: "));  // devices.njk-it.de"));
                            c.println(update_host);
                            c.println(F("Referer: /controller.php?device=MUX4s-1e"));
                            c.print(F("User-Agent: MUX4s-1e/"));
                            c.println(build, DEC);
                            c.print(F("X-current-build: "));
                            c.println(build, DEC);
                            c.println(F("X-current-version: 1.1"));
                            c.println(F("Connection: close"));
                            c.println();  // end HTTP request header
                            char h_buf[128];
                            uint8_t skipped = 0;
                            if (SD.exists(FIRMWARE_FILE)) {
                                SD.remove(FIRMWARE_FILE);
                            }

                            File fh = SD.open(F(FIRMWARE_FILE), FILE_WRITE);
                            uint16_t b_read = 0;
                            while (c.connected() && !c.available()) delay(1);  // wait for datas
                            // while (c.connected()) {
                            while (c.connected() || c.available()) {
                                wdt_reset();
                                if (!skipped)
                                    b_read = c.readBytesUntil('\n', h_buf, 128);

                                if (!skipped && b_read <= 1) {
                                    skipped = 1;
                                    continue;
                                }
                                static uint32_t b_count;
                                if (skipped) {
                                    b_read = c.readBytes(h_buf, 16);
                                    b_count += b_read;

                                    if (!(b_count % 512)) {
                                        led_toggle();
                                    }

                                    fh.write(h_buf, b_read);
                                }
                                // }
                            }
                            build = build_available;
                            fh.flush();
                            fh.close();
                            fh = SD.open(F(FIRMWARE_FILE), FILE_READ);
#ifdef DEBUG
                            const static char txt1[] PROGMEM = "Calculating CRC...";
                            UART0_Println(txt1);
#endif
                            uint32_t len;
                            crc_32_calc = crc32(fh, len);
#ifdef DEBUG
                            printsf("File length is %lu\r\n", len);
                            printsf("Calculated CRC %lX\r\n", crc_32_calc);
                            printsf("Current CRC %lX\r\n", crc_32);
#endif
                            if (crc_32 == crc_32_calc) {
                                // eeprom_write_byte((uint8_t*)EEPROM_FLAG, EEPROM_FLAG_ENABLE);  // set the flag to tell the bootloader to check for new firmware
                            }

                            fh.close();
                            led_off();
                        }
                        c.stop();
                        post = 0;
                        get = 1;
                        b_read = 0;
                    } else

                        if (b_read == EMPTY_LINE && post && !image) {
                        // SAVE SETTINGS
                        while (client.available()) {
                            wdt_reset();
                            b_read = client.readBytesUntil('&', html_buffer, HTML_BUFFER_SIZE);
                            html_buffer[b_read] = 0;
                            char a[] = "%3A";
                            char b[] = ":";
                            str_replace(html_buffer, a, b);
                            strcpy(a, "%2B");
                            strcpy(b, "+");
                            str_replace(html_buffer, a, b);
#ifdef DEBUG
                            UART0_Println(html_buffer);
#endif
                            parseParameter(html_buffer);
                        }
                        // invalidate EEPROM date by changing stored CRC value
                        if (reset_settings) {
                            uint32_t crc = 0;
                            crc = eeprom_read_word(0);
                            eeprom_write_word(0, crc - 1);
                        }
                        if (key == key_up) {
                            reset = 1;
                            client.println((const __FlashStringHelper*)UPDATE_WAIT);
                            break;
                        } else if (!update && !reset) {
                            saveParamsToEEPROM();
                            post = 0;
                            get = 1;
                        }

                        b_read = 0;
                        uptime_admin = uptime;
                    }
                    // -----------------
                    if (b_read < 3 && get) {
                        // send a standard http response header
                        client.println((const __FlashStringHelper*)H1);
                        if ((uptime - uptime_admin) < 60 * ADMIN_TIMEOUT) {
                            client.print((const __FlashStringHelper*)F3);
                            for (uint8_t j = 1; j <= PORTS; j++) {
                                // String num = String(j, DEC);
                                char num[3];
                                itoa(j, num, 10);

                                if (j == PORTS) {
                                    client.print(F("<div>\r\n<h3>Port "));
                                    client.print(num);
                                    client.println(F(", Ethernet</h3>\r\n<table>"));
                                    client.print(F("<tr><td class='fc'>IP Port:</td><td>"));
                                    if (!has_config) {
                                        client.print(F("<input type='text' size='5' name='p"));
                                        client.print(num);
                                        client.print(F("_port' value='"));
                                    }
                                    client.print(port1, DEC);
                                    if (!has_config) {
                                        client.print(F("'>*"));
                                    }
                                    client.println(F("</td></tr>"));
                                } else {
                                    client.print(F("<div>\r\n<h3>Port "));
                                    client.print(num);
                                    client.print(F(", Serial</h3>\r\n<table>\r\n<tr><td class='fc'>Baud:</td><td>"));
                                    if (!has_config) {
                                        client.print(F("<select name='p"));
                                        client.print(num);
                                        client.print(F("_baud'>"));
                                        for (uint8_t k = 0; k < ARRAY_SIZE(baud); k++) {
                                            if (p_config[j - 1].baud == baud[k]) {
                                                client.print(F("<option selected>"));
                                            } else {
                                                client.print(F("<option>"));
                                            }
                                            client.print(baud[k], DEC);
                                            client.println(F("</option>"));
                                        }

                                        client.println(F("</select>*"));
                                    } else {
                                        for (uint8_t k = 0; k < ARRAY_SIZE(baud); k++) {
                                            if (p_config[j - 1].baud == baud[k]) {
                                                client.print(baud[k], DEC);
                                                break;
                                            }
                                        }
                                    }
                                    client.println(F("</td></tr>"));
                                    client.print(F("<tr><td class='fc'>Parameter:</td><td>"));
                                    if (!has_config) {
                                        client.print(F("<select name='p"));
                                        client.print(num);
                                        client.print(F("_parameter'>"));
                                        for (uint8_t k = 0; k < 3; k++) {
                                            if (p_config[j - 1].parameter == params[k]) {
                                                client.print(F("<option value='"));
                                                client.print(params[k], DEC);
                                                client.print(F("' selected>"));
                                            } else {
                                                client.print(F("<option value='"));
                                                client.print(params[k], DEC);
                                                client.print(F("'>"));
                                            }
                                            client.print(params_name[k]);
                                            client.println(F("</option>"));
                                        }
                                        client.println(F("</select>*"));
                                    } else {
                                        for (uint8_t k = 0; k < 3; k++) {
                                            if (p_config[j - 1].parameter == params[k]) {
                                                client.print(params_name[k]);
                                                break;
                                            }
                                        }
                                    }
                                    client.println(F("</td></tr>"));
                                }

                                /**
                                 * Direction
                                 */
                                client.print(F("<tr><td class='fc'>Direction:</td><td>"));
                                for (uint8_t k = 0; k < 4; k++) {
                                    if (!has_config) {
                                        client.print(F("<input type='radio' name='p"));
                                        client.print(num);
                                        if (k == p_config[j - 1].direction) {
                                            client.print(F("_direction' checked value='"));
                                        } else {
                                            client.print(F("_direction' value='"));
                                        }
                                        client.print(k, DEC);
                                        client.print("'>");
                                        client.print(direction_name[k]);
                                        client.write(' ');
                                    } else {
                                        if (k == p_config[j - 1].direction) {
                                            client.print(direction_name[k]);
                                            break;
                                        }
                                    }
                                }
                                client.println(F("</td></tr>"));
                                /**
                                 * FILTER
                                 */
                                client.print(F("<tr><td class='fc'>Input Filter:</td><td>"));
                                if (!has_config) {
                                    client.print(F("<input placeholder='+all' onkeypress='return checkKey(event)' size='64' type='text' "));
                                    client.print(F("name='p"));
                                    client.print(num);
                                    client.print(F("_ifilter' value='"));
                                }
                                printFilter(client, &(p_config[j - 1].ifilter));
                                if (!has_config) {
                                    client.println(F("'>"));
                                }
                                client.println(F("</td></tr>"));
                                client.print(F("<tr><td class='fc'>Output Filter:</td><td>"));
                                if (!has_config) {
                                    client.print(F("<input placeholder='+all' onkeypress='return checkKey(event)' size='64' type='text' "));
                                    client.print(F("name='p"));
                                    client.print(num);
                                    client.print(F("_ofilter' value='"));
                                }
                                printFilter(client, &(p_config[j - 1].ofilter));
                                if (!has_config) {
                                    client.print(F("'>"));
                                }
                                client.println(F("</td></tr>\r\n</table>\r\n</div>"));
                            }
                            wdt_reset();    // Needed here as reading the strings from flash takes some time
                            /**
                             * ETHERNET SETUP
                             */
                            client.print((const __FlashStringHelper*)F4);
                            client.print(Ethernet.localIP());
                            client.print(F(" ("));
                            client.print(HOST_NAME);
                            client.print(F("), DNS: "));
                            client.print(Ethernet.dnsServerIP());
                            client.print(F(", Gateway: "));
                            client.print(Ethernet.gatewayIP());
                            client.print(F(", Subnet Mask: "));
                            client.print(Ethernet.subnetMask());
                            client.println(F("<br><br>Network settings:</td></tr>"));
                            client.print(F("<tr><td class='fc'>Hostname:</td><td>"));
                            if (!has_config) {
                                client.print(F("<input maxlength='20' type='text' size='15' name='hostname' value='"));
                            }
                            client.print(hostname);
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td></tr>"));
                            client.print(F("<tr><td class='fc'>IP Address:</td><td>"));
                            if (!has_config) {
                                client.print(F("<input onkeypress='return checkKey2(event)' type='text' size='15' name='ip_address' value='"));
                            }
                            if (ip_address[0] + ip_address[1] + ip_address[2] + ip_address[3]) {
                                client.print(IPAddress(ip_address));
                            }
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td></tr>"));
                            client.print(F("<tr><td class='fc'>DNS:</td><td>"));
                            if (!has_config) {
                                client.print(F("<input onkeypress='return checkKey2(event)' type='text' size='15' name='dns' value='"));
                            }
                            if (dns_a[0] + dns_a[1] + dns_a[2] + dns_a[3]) {
                                client.print(IPAddress(dns_a));
                            }
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td></tr>"));
                            client.print(F("<tr><td class='fc'>Gateway:</td><td>"));
                            if (!has_config) {
                                client.print(F("<input onkeypress='return checkKey2(event)' type='text' size='15' name='gateway' value='"));
                            }
                            if (gateway[0] + gateway[1] + gateway[2] + gateway[3]) {
                                client.print(IPAddress(gateway));
                            }
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td></tr>"));
                            client.print(F("<tr><td class='fc'>Subnet:</td><td>"));
                            if (!has_config) {
                                client.print(F("<input onkeypress='return checkKey2(event)' type='text' size='15' name='subnet' value='"));
                            }
                            if (subnet[0] + subnet[1] + subnet[2] + subnet[3]) {
                                client.print(IPAddress(subnet));
                            }
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td></tr>"));
                            client.print((const __FlashStringHelper*)ETH1);
                            client.print(F("<tr><td class='fc'>MQTT Host</td><td>"));
                            if (!has_config) {
                                client.print(F("<input maxlength='20' type='text' size='15' name='mqtt_host' value='"));
                            }
                            client.print(mqtt_broker);
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td><tr>"));
                            client.print(F("<tr><td class='fc'>MQTT Port</td><td>"));
                            if (!has_config) {
                                client.print(F("<input maxlength='20' type='text' size='15' name='mqtt_port' value='"));
                            }
                            client.print(mqtt_port, DEC);
                            if (!has_config) {
                                client.print(F("'>*"));
                            }
                            client.println(F("</td><tr>"));
                            client.print((const __FlashStringHelper*)F2);
                        }

                        // Uptime Calculation
                        uint32_t days = uptime / 86400;
                        uint32_t hours = (uptime - days * 86400) / 3600;
                        uint32_t minutes = (uptime - days * 86400 - hours * 3600) / 60;
                        uint32_t seconds = uptime - days * 86400 - hours * 3600 - minutes * 60;
                        client.print(F("Uptime: "));
                        client.print(days, DEC);
                        client.print(F(" days, "));
                        client.print(hours, DEC);
                        client.print(F(" hours, "));
                        client.print(minutes, DEC);
                        client.print(F(" minutes, "));
                        client.print(seconds, DEC);
                        client.println(F(" seconds<br>"));
                        client.print(F("Messages received: "));
                        client.print(mesg_recv, DEC);
                        client.print(F(", sent: "));
                        client.print(mesg_sent, DEC);
                        client.println(F("<br>"));
                        client.print(F("Free memory: "));
                        client.print(freeMemory(), DEC);
                        client.println(F(" Bytes"));

                        if ((uptime - uptime_admin) < 60 * ADMIN_TIMEOUT && !image) {
                            if (!SD.exists(CONFIG_FILE)) {
                                client.print(F("<br><br><button type='submit' value='save'>SAVE SETTINGS</button>"));
                            }
                            client.print((const __FlashStringHelper*)F1);
                            client.print(F("<br>v1.1 ("));
                            client.print(BUILD, DEC);
                            client.println(F(") "));
                            if ((build_available - build) > 0 && has_sd) {
                                client.print(F("<b>An update is available ("));
                                client.print(build_available, DEC);
                                client.print(
                                    F("). Download now (You must have a SD card in the slot!)?"));

                                if (has_sd) {
                                    client.print(F("<form method='POST' action='/download' ><button type='submit' name='btn' value='update'>YES</button></form>"));
                                }
                                client.print(F("</b>"));
                            }
                            client.println(F("<br>"));

#ifdef SDC
                            // If a firmware file is present, display UPDATE button
                            // TODO What if file is on SD card only without CRC online check?
                            // // Serial.println(crc_32_calc, HEX);// Serial.println(crc_32,HEX);
                            if (crc_32_calc == crc_32 && SD.exists(FIRMWARE_FILE)) {
                                client.print((const __FlashStringHelper*)UPDATE_FORM_1);
                                client.print(key, DEC);
                                client.print((const __FlashStringHelper*)UPDATE_FORM_2);
                            }
#endif
                        }
                        client.print((const __FlashStringHelper*)F5);
                        break;
                    } else if (b_read < 3 && reset && !image && !update) {
                        client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-cache, no-store, must-revalidate\r\nPragma: no-cache\r\nExpires: 0\r\nRefresh:10; url=/\r\nConnection: close\r\n\r\n<html lang='en'>\r\n<head>\r\n<meta charset='utf-8' />\r\n<meta name='viewport' content='width=device-width, initial-scale=1' />\r\n<title>MUX4s-1e Updater</title>\r\n<style>* {font-family: monospace;}</style>\r\n<body>Please wait. Page reloads after 10 seconds.</body></html>"));
                        // client.flush();
                        // client.stop();
                    }
                }
            }
            client.flush();
            delay(1);
            client.stop();

            if (reset) {
                wdt_enable(WDTO_2S);
                // Update requested?
                if (update) {
                    update = 0;
                    char t[] = "Updating Firmware";
                    sendFreeText(t);
                    send2All();
                }
                // client.print((const __FlashStringHelper*)H2);
                server->flush();
                server1.flush();
                free(server);
                c.stop();

                // Software reset
                char t[40];
                sprintf(t, "%s - Performing Reset", hostname);
                sendFreeText(t);
                send2All();
                stop();
                while (1);
            }

            /* #ifndef DEBUG
                        Serial.begin(p_config[P1].baud, p_config[P1].parameter);
                        UCSR0B |= _BV(TXCIE0);
            #endif
                        Serial1.begin(p_config[P2].baud, p_config[P2].parameter);
                        Serial2.begin(p_config[P3].baud, p_config[P3].parameter);
                        Serial3.begin(p_config[P4].baud, p_config[P4].parameter); */

            // wdt_enable(WDTO_8S);
        }
#endif
    }
#endif

    if (static_send) {
        char buf[40];

        sprintf(buf, "%s - Sent: %lu Recv: %lu Uptime: %lus", hostname, mesg_sent, mesg_recv, uptime);
        sendFreeText(buf);
#ifdef ETHERNET
        if (Ethernet.link() == 1) {
            if (mqttClient.connected()) {
                char buf2[48];
                sprintf(buf2, "vessels/self/mux/%s/text", hostname);
                mqttClient.beginMessage(buf2);
                mqttClient.print(buf);
                mqttClient.endMessage();
            } else {
                connectMQTT(mqtt_broker, mqtt_port);
            }
        }
#endif
        static_send = false;
    }

    // Send out all messages
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }

    // Check, if button was pressed to enable admin page
    if (!(PINL & _BV(RESET_BTN_PIN))) {
        uptime_admin = uptime;
        PORTB |= _BV(BUILTIN_LED);
    } else {
        PORTB &= ~_BV(BUILTIN_LED);
    }
}

struct nmea* extractNMEA(char* nmea_line) {
    struct nmea* result = (struct nmea*)malloc(sizeof(struct nmea));
    // Get the sentence identifier
    if (nmea_line[0] != '$' && nmea_line[0] != '!') {
        strcpy(result->id, "NULL");
        result->id[5] = 0;
        return result;
    }

    /* if (!checkChecksum(index))
        return; */

    // id = (char*)malloc(6);  // The NMEA0183 sentence name
    strncpy(result->id, nmea_line + 1, 5);
    // id[5] = 0;
    strcpy(result->sentence, nmea_line + 7);
    result->id[5] = 0;
    return result;
}

/**
 * Timer Routine.
 * Just counting seconds
 */
ISR(TIMER1_COMPA_vect) {
    uptime++;
    if (!(uptime % 05)) {
        static_send = true;
    }
}

/**
 * Read the bytes from a stream
 */
uint8_t readStream(Stream& stream, uint8_t port_num) {
    uint8_t read = 0;
    char nmea_line[NMEA_LINE_LENGTH];

    // Is IN active or buffer full
    if (!(p_config[port_num].direction & IN) || fifo_is_full(msgout_buf)) {
        return 0;
    }

    // We have three conditions for a "full" read
    // 1. bytes available less then MAX
    // 2. '\n' in read buffer
    // 3. bytes do not contain '\n' (or '\r') but equals MAX

    // Case 1. normal read
    // Case 2. check for '\n' ('\r')
    // Case 3. continue read with next buffer (but not if then full)

    uint8_t b_read = 0;
    while (stream.available()) {
        // There is something that wants to be read

        b_read = stream.readBytesUntil('\n', nmea_line, NMEA_LINE_LENGTH);

        if (!b_read) {
            continue;
        }

        // We have a full line
        if (nmea_line[b_read - 1] == '\r') {
            nmea_line[b_read - 1] = 0;
            if (!matchFilter(nmea_line, &(p_config[port_num].ifilter)) || !checkChecksum(nmea_line)) {
                continue;
            }

            /**
             *  We are setting a mark/mask to avoid sending back the traffic from where it arrives (see send2All())
             */
            switch (port_num) {
                // P1
                case 0:
                    nmea_line[0] &= 0b00111111;
                    break;
                    // P2
                case 1:
                    nmea_line[0] |= 0b01000000;
                    break;
                    // P3
                case 2:
                    nmea_line[0] |= 0b10000000;
                    break;
                    // P4
                case 3:
                    nmea_line[0] |= 0b11000000;
                    break;
                    // PETH1
                case 4:
                    nmea_line[0] |= 0b00000010;
                    break;
            }

#ifdef TRANSLATE

            // Get the sentence identifier
            char id[4];  // The NMEA0183 sentence name
            id[3] = 0;   // zero for a char array
            strncpy(id, nmea_line + 3, 3);
            if (!strcmp(id, NMEA_DPT) && checkChecksum(nmea_line)) {
                // Store the original message as it will be changed bleow
                fifo_add(msgout_buf, nmea_line);
                mesg_recv++;  // got one more

                // store the input port check character
                char check = nmea_line[0];

                // $--DPT,x.x,x.x*hh<CR><LF>
                // Field Number:
                //   1) Depth, meters
                //   2) Offset from transducer,
                //      positive means distance from tansducer to water line
                //      negative means distance from transducer to keel
                //   3) Checksum

                // this is a comma separated item within
                // char* token = (char*)malloc(10);
                /* get the first token */
                char* token = strtok(nmea_line, ",");

                if (token != NULL) {
                    token = strtok(NULL, ",");

                    // if (i == 1) {
                    float depth_m = atof(token);
                    float depth_f = depth_m * 3.28084;
                    float depth_fh = depth_m * 0.546807;

                    char buf_f[7];  // can easily be greater
                    char buf_m[6];
                    char buf_fh[6];
                    dtostrf(depth_f, 3, 1, buf_f);
                    dtostrf(depth_m, 3, 1, buf_m);
                    dtostrf(depth_fh, 3, 1, buf_fh);

                    char txt[] = "$NMDBS,%s,f,%s,M,%s,F*";
                    char buf[40];
                    sprintf(buf, txt, buf_f, buf_m, buf_fh);
                    const uint8_t cs = calcChecksum(buf);

                    sprintf(nmea_line, "%s%02X", buf, cs);
                    nmea_line[0] = check;
                }
            }
            // $IIDPT,3.5,0.5*43
            // $IIDPT,30.5,0.5*73
            // $IIDPT,200.5,0.5*42
            // $IIDPT,600.5,0.5*46

#endif

            /*if (clearDoubleSentence(13, index)) {
             continue;
             }*/
            /* read++;
            index++;
            if (index >= NMEA_LINES) {
                    index--;
                    break;
            } */
            fifo_add(msgout_buf, nmea_line);
            mesg_recv++;
            continue;
        }
    }
    // Flush
    while (stream.read() > -1);
    return read;
}

/**
 * Send the bytes from a stream
 */
void sendStream(Stream& stream, uint8_t port_num, char* nmea_line) {
    // Is OUT active
    if (!(p_config[port_num].direction & OUT)) {
        return;
    }
    // stream.flush();
    // while (!// Serial.availableForWrite())

    const char* ptr = strchr(nmea_line, '\0');
    if (ptr) {
        int16_t i = ptr - nmea_line;
        if (i > 0) {
            stream.write(nmea_line, i);
            stream.write(crlf, 2);  // line break
        }
    } else {
        stream.write(nmea_line, NMEA_LINE_LENGTH);
    }
    mesg_sent++;
}

/**
 * Compare the sentence with the filter element
 * @return 0 if no match, 1 if match
 */

uint8_t matchFilter(const char* nmea_line, Filter* filter) {
    uint8_t match = 1;
    for (uint8_t j = 0; j < FILTER_SIZE; j++) {
        match = 1;
        for (uint8_t i = 1; i <= 5; i++) {
            if (filter->pattern[j][i] == '*')
                continue;
            if (nmea_line[i] != filter->pattern[j][i]) {
                match = 0;
                break;
            }
        }

        if (match) {
            return filter->pattern[j][0] != '-';
        }
    }
    return filter->pm_all;
}

uint8_t checkChecksum(char* nmea_line) {
    if ((nmea_line[0] & '$') != '$' && nmea_line[0] != '!') {
        return 0;
    }

    uint8_t cs = calcChecksum(nmea_line);

    // Get the checksum characters
    char t[3];
    const char* ptr = strchr(nmea_line, '*');
    if (ptr) {
        int16_t i = ptr - nmea_line;
        t[0] = nmea_line[i + 1];
        t[1] = nmea_line[i + 2];
        t[2] = '\0';
        if (cs != hexStr2Int(t)) {
            return 0;
        }
    } else {
        return 0;
    }

    return 1;
}

/**
 *
 */
void sendFreeText(char* text) {
    char nmea_line[NMEA_LINE_LENGTH];

    sprintf(nmea_line, "$MXTXT,%s*", text);
    const uint8_t cs = calcChecksum(nmea_line);
    sprintf(nmea_line, "$MXTXT,%s*%02X", text, cs);
    fifo_add(msgout_buf, nmea_line);
}

void printErrorMessage(uint8_t e, bool eol) {
#ifdef SDC
#ifdef DEBUG
    switch (e) {
        case IniFile::errorNoError:
            const static char txt1[] PROGMEM = "no error";
            UART0_Print_P(txt1);
            break;
        case IniFile::errorFileNotFound:
            const static char txt2[] PROGMEM = "file not found";
            UART0_Print_P(txt2);
            break;
        case IniFile::errorFileNotOpen:
            const static char txt3[] PROGMEM = "file not open";
            UART0_Print_P(txt3);
            break;
        case IniFile::errorBufferTooSmall:
            const static char txt4[] PROGMEM = "buffer too small";
            UART0_Print_P(txt4);
            break;
        case IniFile::errorSeekError:
            const static char txt5[] PROGMEM = "seek error";
            UART0_Print_P(txt5);
            break;
        case IniFile::errorSectionNotFound:
            const static char txt6[] PROGMEM = "section not found";
            UART0_Print_P(txt6);
            break;
        case IniFile::errorKeyNotFound:
            const static char txt7[] PROGMEM = "key not found";
            UART0_Print_P(txt7);
            break;
        case IniFile::errorEndOfFile:
            const static char txt8[] PROGMEM = "end of file";
            UART0_Print_P(txt8);
            break;
        case IniFile::errorUnknownError:
            const static char txt9[] PROGMEM = "unknown error";
            UART0_Print_P(txt9);
            break;
        default:
            const static char txt10[] PROGMEM = "unknown error value";
            UART0_Print_P(txt10);
            break;
    }
    if (eol) {
        UART0_Println("");
    }
#endif
#endif
}

uint32_t crc32(File& file, uint32_t& charcnt) {
    uint32_t oldcrc32 = 0xFFFFFFFF;
    charcnt = 0;

    while (file.available()) {
        wdt_reset();
        uint8_t c = file.read();
        charcnt++;
        oldcrc32 = updateCRC32(c, oldcrc32);
    }

    return ~oldcrc32;
}

inline uint32_t updateCRC32(uint8_t ch, uint32_t crc) {
    uint32_t idx = ((crc) ^ (ch)) & 0xff;
    uint32_t tab_value = pgm_read_dword(crc_32_tab + idx);
    return tab_value ^ ((crc) >> 8);
}

/*
 uint8_t clearDoubleSentence(uint8_t range, uint8_t idx) {
 // Iterate over buffer and look for similar
 if (idx == 0) return 0;
 for (uint8_t i = 0; i < idx; i++) {
 // compare and if found, replace
 if (!strncmp(nmea_buffer[idx], nmea_buffer[i], range)) {
 memcpy(nmea_buffer[i], nmea_buffer[idx], NMEA_LINE_LENGTH);
 return 1;
 }
 }
 return 0;

 }
 */
#ifdef WEB_GUI
void printFilter(Stream& stream, Filter* filter) {
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        if (!filter->pattern[i][0]) {
            stream.print(filter->pm_all ? (i ? "+all" : "") : "-all");
            return;
        }
        stream.print(filter->pattern[i]);
        stream.write(':');
    }
}
void parseParameter(char* p) {
    // index search
    int8_t i = -1;
    uint8_t k = 0;
    while (p[k++]) {
        i++;
    }

    for (k = 0; k <= i; k++) {
        if (p[k] == '=') {
            break;
        }
    }
    char t[k + 1];
    t[k] = 0;
    strncpy(t, p, k);
    // ---

    /*if (!(i-k)) {
     return;
     }*/

    char v[i - k + 1];
    v[i - k] = 0;
    strncpy(v, p + k + 1, i - k);

    for (uint8_t i = 0; i < 4; i++) {
        char k[] = "p1_bau";
        k[1] = i + 49;
        if (!strncmp(t, k, 6)) {
            p_config[i].baud = atol(v);
        }
    }
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_dir";
        k[1] = i + 49;
        if (!strncmp(t, k, 6)) {
            p_config[i].direction = v[0] - 48;
        }
    }
    for (uint8_t i = 0; i < 4; i++) {
        char k[] = "p1_par";
        k[1] = i + 49;
        if (!strncmp(t, k, 6)) {
            p_config[i].parameter = atoi(v);
        }
    }

    /**
     * Input Filter
     */
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_ifi";
        k[1] = i + 49;
        if (!strncmp(t, k, 6)) {
            char* token;

            char* rest = v;
            uint8_t j = 0;

            while ((token = strtok_r(rest, ":", &rest))) {
                // strcpy(v, trim(v));
                if (!strcmp(token, "-all")) {
                    p_config[i].ifilter.pm_all = 0;
                } else if (!strcmp(token, "+all")) {
                    p_config[i].ifilter.pm_all = 1;
                } else {
                    token[6] = 0;
                    strcpy(p_config[i].ifilter.pattern[j++], token);
                }
            }
            for (uint8_t a = j; a < FILTER_SIZE; a++) {
                p_config[i].ifilter.pattern[a][0] = 0;
            }
        }
    }
    /**
     * Output Filter
     */
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_ofi";
        k[1] = i + 49;
        if (!strncmp(t, k, 6)) {
            char* token;

            char* rest = v;
            uint8_t j = 0;

            while ((token = strtok_r(rest, ":", &rest))) {
                // strcpy(v, trim(v));
                if (!strcmp(token, "-all")) {
                    p_config[i].ofilter.pm_all = 0;
                } else if (!strcmp(token, "+all")) {
                    p_config[i].ofilter.pm_all = 1;
                } else {
                    // uint8_t size = sizeof token / sizeof *token;
                    token[6] = 0;
                    strcpy(p_config[i].ofilter.pattern[j++], token);
                }
            }
            for (uint8_t a = j; a < FILTER_SIZE; a++) {
                p_config[i].ofilter.pattern[a][0] = 0;
            }
        }
    }

    /**
     * MQTT
     */
    if (!strncmp(t, "mqtt_host", 9)) {
        strcpy(mqtt_broker, v);
    } else if (!strncmp(t, "mqtt_port", 9)) {
        mqtt_port = atoi(v);
    }

    /**
     * IP Address
     */
    if (!strncmp(t, "ip_add", 6)) {
        if (!(i - k)) {
            ip_address[0] = ip_address[1] = ip_address[2] = ip_address[3] = 0;
        } else {
            IPAddress ip;
            ip.fromString(v);
            ip_address[0] = ip[0];
            ip_address[1] = ip[1];
            ip_address[2] = ip[2];
            ip_address[3] = ip[3];
        }
    } else if (!strncmp(t, "hostname", 8)) {
        strcpy(hostname, v);
    } else if (!strncmp(t, "dns", 3)) {
        if (!(i - k)) {
            dns_a[0] = dns_a[1] = dns_a[2] = dns_a[3] = 0;
        } else {
            IPAddress ip;
            ip.fromString(v);
            dns_a[0] = ip[0];
            dns_a[1] = ip[1];
            dns_a[2] = ip[2];
            dns_a[3] = ip[3];
        }
    } else if (!strncmp(t, "gateway", 7)) {
        if (!(i - k)) {
            gateway[0] = gateway[1] = gateway[2] = gateway[3] = 0;
        } else {
            IPAddress ip;
            ip.fromString(v);
            gateway[0] = ip[0];
            gateway[1] = ip[1];
            gateway[2] = ip[2];
            gateway[3] = ip[3];
        }
    }

    else if (!strncmp(t, "subnet", 6)) {
        if (!(i - k)) {
            subnet[0] = subnet[1] = subnet[2] = subnet[3] = 0;
        } else {
            IPAddress ip;
            ip.fromString(v);
            subnet[0] = ip[0];
            subnet[1] = ip[1];
            subnet[2] = ip[2];
            subnet[3] = ip[3];
        }
    } else if (!strncmp(t, "p5_por", 6)) {
        port1 = atoi(v);
    } else if (!strncmp(t, "key", 3)) {
        key_up = atoi(v);
    } else if (!strncmp(t, "reset_settings", 14)) {
        reset_settings = atoi(v);
    } else if (!strncmp(t, "build", 5)) {
        build_available = atoi(v);
    } else if (!strncmp(t, "crc32", 5)) {
        crc_32 = hexStr2Int(v);
    }
}

#endif

void timerSetup() {
    cli();

    // Clear registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = _TCNT1;

    // CTC
    TCCR1B |= _BV(WGM12);
    // Prescaler
#if PRESC == 1
    TCCR1B &= ~_BV(CS11);
    TCCR1B &= ~_BV(CS12);
    TCCR1B |= _BV(CS10);
#elif PRESC == 8
    TCCR1B &= ~_BV(CS10);
    TCCR1B &= ~_BV(CS12);
    TCCR1B |= _BV(CS11);
#elif PRESC == 64
    TCCR1B &= ~_BV(CS12);
    TCCR1B |= _BV(CS10);
    TCCR1B |= _BV(CS11);
#elif PRESC == 256
    TCCR1B &= ~_BV(CS10);
    TCCR1B &= ~_BV(CS11);
    TCCR1B |= _BV(CS12);
#elif PRESC == 1024
    TCCR1B &= ~_BV(CS11);
    TCCR1B |= _BV(CS10);
    TCCR1B |= _BV(CS12);
#else
#error No prescaler defined
#endif

    TIMSK1 |= _BV(OCIE1A);

    sei();
}

/**
 * Get parameters from the EEPROM and initialize variables
 */
void loadParamsFromEEPROM() {
    // We do have stored parameters, so load them
    uint16_t c = sizeof(uint32_t);  // skip to the next memory address
    for (uint8_t i = 0; i < PORTS; i++) {
        eeprom_read_block((void*)&p_config[i], (const uint16_t*)c, sizeof(p_config[i]));
        c += sizeof(p_config[i]);
    }
    eeprom_read_block((void*)&mac, (const uint16_t*)c, sizeof(mac));
    c += sizeof(mac);
    eeprom_read_block((void*)&ip_address, (const uint16_t*)c, sizeof(ip_address));
    c += sizeof(ip_address);
    eeprom_read_block((void*)&dns_a, (const uint16_t*)c, sizeof(dns_a));
    c += sizeof(dns_a);
    eeprom_read_block((void*)&gateway, (const uint16_t*)c, sizeof(gateway));
    c += sizeof(gateway);
    eeprom_read_block((void*)&subnet, (const uint16_t*)c, sizeof(subnet));
    c += sizeof(subnet);
    debug = eeprom_read_byte((const uint8_t*)c);
    c += sizeof(debug);
    port1 = eeprom_read_word((const uint16_t*)c);
    c += sizeof(port1);
    eeprom_read_block((void*)&hostname, (const uint16_t*)c, sizeof(hostname));
    c += sizeof(hostname);
    eeprom_read_block((void*)&mqtt_broker, (const uint16_t*)c, sizeof(mqtt_broker));
    c += sizeof(mqtt_broker);
    mqtt_port = eeprom_read_word((const uint16_t*)c);
    c += sizeof(mqtt_port);
    admin_timeout = eeprom_read_byte((const uint8_t*)c);
}

/**
 * Save variables to the EEPROM as parameters, calculate and store the CRC value
 */
void saveParamsToEEPROM() {
    // We do have stored parameters, so load them
    uint16_t c = sizeof(uint32_t);  // skip to the next memory address

    for (uint8_t i = 0; i < PORTS; i++) {
        eeprom_write_block((void*)&p_config[i], (void*)c, sizeof(p_config[i]));
        c += sizeof(p_config[i]);
    }
    eeprom_write_block((void*)&mac, (void*)c, sizeof(mac));
    c += sizeof(mac);
    eeprom_write_block((void*)&ip_address, (void*)c, sizeof(ip_address));
    c += sizeof(ip_address);
    eeprom_write_block((void*)&dns_a, (void*)c, sizeof(dns_a));
    c += sizeof(dns_a);
    eeprom_write_block((void*)&gateway, (void*)c, sizeof(gateway));
    c += sizeof(gateway);
    eeprom_write_block((void*)&subnet, (void*)c, sizeof(subnet));
    c += sizeof(subnet);
    eeprom_write_byte((uint8_t*)c, debug);
    c += sizeof(debug);
    eeprom_write_word((uint16_t*)c, port1);
    c += sizeof(port1);
    eeprom_write_block((void*)&hostname, (void*)c, sizeof(hostname));
    c += sizeof(hostname);
    eeprom_write_block((void*)&mqtt_broker, (void*)c, sizeof(mqtt_broker));
    c += sizeof(mqtt_broker);
    eeprom_write_word((uint16_t*)c, mqtt_port);
    c += sizeof(mqtt_port);
    eeprom_write_byte((uint8_t*)c, admin_timeout);
    c += sizeof(admin_timeout);
    // now store the CRC

    const uint16_t crc = eeprom_crc();
#ifdef DEBUG
    printsf("%x\r\n", crc);
#endif

    eeprom_write_word(0, crc);
#ifdef DEBUG
    const static char txt9[] PROGMEM = "Saved to EEPROM";
    UART0_Println_P(txt9);
#endif
}

void connectMQTT(char* broker, uint16_t port) {
    mqttClient.setId(hostname);
    mqttClient.connect(broker, port);
}

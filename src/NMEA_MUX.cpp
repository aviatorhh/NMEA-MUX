#include "NMEA_MUX.h"

#include <Arduino.h>
#include <FIFO.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <string.h>
#ifdef __AVR_ATmega328P__
#include <ArduinoUniqueID.h>
#endif
#include <ArduinoMqttClient.h>
#ifdef HAS_I2C_LCD
#include <LiquidCrystal_I2C.h>
#define COLS 20
#define ROWS 4
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif
#include "custom_chars.h"
#include "dgps.h"
#include "utils.h"
#ifdef HAS_CAN
#include "canbus.h"

#endif
#ifdef WEB_GUI
EthernetServer server1(80);
#endif

/* ---------------- */
float lat, lon;
gga_t gga;
rmc_t rmc;
uint8_t valid_sats = 0;
uint32_t last_brd03_time = 0;
uint32_t last_sat_n_time = 0;

#ifdef DGPS

#define MAX_SATS 24
static sat_t sats[MAX_SATS];
static float current_lat = 0.0;
static float current_lon = 0.0;

float ref_lat, ref_lon, ref_alt;
static uint16_t dgps_station_id = 0;
static float dgps_time_s = 0.0;
static float dgps_ref_x = 0.0;
static float dgps_ref_y = 0.0;
static float dgps_ref_z = 0.0;
static uint8_t dgps_ref_valid = 0;
static uint32_t dgps_ref_timestamp = 0;

static uint8_t gsv_cycle_complete = 0;
static uint8_t gsv_total = 0;
static uint8_t gsv_seen = 0;
static uint8_t gsv_mask = 0;
static uint8_t gsv_msg = 0;

static kf_t kf;

static spoof_score_t sp;
#endif

#ifdef HAS_I2C_LCD
uint8_t show_state;
uint8_t show_ipa;
uint8_t show_mesg_rate;
uint8_t show_time;
#endif

uint16_t mps_in;
uint16_t mps_out;

uint32_t ap_last_time;
uint32_t dgps_last_time;

uint8_t spoof_flag = 0;
float cx = 0, cy = 0;
char g_buff[48];

fifo_t msgout_buf;
volatile bool static_send;

// struct nmea {
//     char id[6];
//     char sentence[79];
// };

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
// struct nmea* extractNMEA(const char* nmea_line);
// void extractNMEA(const char* nmea_line, nmea* result);
void loadParamsFromEEPROM(void);
void saveParamsToEEPROM(void);

char mqtt_broker[] = "192.168.000.253";  // fully set all digits for memory
int mqtt_port = 1883;

char update_host[32];

uint8_t admin_timeout;
// volatile uint8_t zda_out_counter;
#ifdef HAS_CAN
CAN can(SPI_SS);
#endif

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

/**
 * Check if the given NMEA line matches the filter. The filter can be either a pattern match or a +all/-all match.
 */
void setup() {
#if defined(__AVR_ATmega328P__)
    const uint32_t uid = ((uint32_t)UniqueID8[4] << 24) + ((uint32_t)UniqueID8[5] << 16) + ((uint32_t)UniqueID8[6] << 8) + UniqueID8[7];
#endif
    wdt_enable(WDTO_8S);

    uptime = 0;
    last_uptime = 0;
    uptime_admin = 0;

#ifdef HAS_I2C_LCD
    // Initiate the LCD:
    lcd.begin(COLS, ROWS);
    lcd.createChar(1, (uint8_t*)upper_min_comma);
    lcd.createChar(2, (uint8_t*)stop_char);

    for (size_t i = 0; i < 2; i++) {
        delay(250);
        lcd.noBacklight();
        delay(250);
        lcd.backlight();
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NMEA MUX4s-1e "));
    lcd.print(BUILD, DEC);
    delay(2000);
    lcd.setCursor(0, 0);
    lcd.write(0b01111110);  // ->
    lcd.print(F(" 0                 "));
    lcd.setCursor(0, 1);
    lcd.write(0b01111111);  // <-
    lcd.print(" 0");
#endif

    build = BUILD;
    build_available = 0;
    crc_32_calc = 0;
    crc_32 = 0;
#ifdef DEBUG

    uint16_t ubrr = (F_CPU / (DEBUG_BAUD * 8)) - 1;
    UART0_Init(ubrr);

    sprintf_P(g_buff, PSTR("%ld - DEBUG enabled"), build);
    UART0_Println(g_buff);
    UART0_Flush();
#endif

    // Setup CAN interface
#ifdef HAS_CAN
    can.init(SPI_SS, true);
#endif

    randomSeed(analogRead(0));  // init random engine

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
    p_config[P4].direction = OUT;

    p_config[PETH1].ofilter.pm_all = 1;
    p_config[PETH1].ifilter.pm_all = 1;
    p_config[PETH1].direction = OUT;
    port1 = SERVER_PORT;

    debug = 0;
    // Network settings
    ip_address[0] = ip_address[1] = ip_address[2] = ip_address[3] = 0;
    dns_a[0] = dns_a[1] = dns_a[2] = dns_a[3] = 0;
    gateway[0] = gateway[1] = gateway[2] = gateway[3] = 0;
    subnet[0] = subnet[1] = subnet[2] = subnet[3] = 0;

    strcpy(hostname, HOST_NAME);
    strcpy(update_host, DEFAULT_UPDATE_HOST);
    admin_timeout = ADMIN_TIMEOUT;

    uint16_t crc = eeprom_read_word(0);  // Get the CRC from EEPROM, if.
#ifdef DEBUG
    printsf("%x\r\n", crc);
    uint8_t eeprom_data_loaded = 0;  // EEPROM data loaded flag
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
            // char d_buf[5];
            dtostrf((float)volumesize / 1024.0, 4, 1, g_buff);
            printsf("Volume size (Gb):  %s\r\n", g_buff);
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
                            printErrorMessage(ini.getError());
#endif
                        }
                        if (ini.getValue(port, "ifilter", buffer, bufferLen)) {
                            char* p = strtok(buffer, ":");
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
                            char* p = strtok(buffer, ":");
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
                            printErrorMessage(ini.getError());
#endif
                        }
                    }
                }
                ini.close();
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
        eeprom_data_loaded = 0;
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
            eeprom_data_loaded = 1;
#endif
        }

#ifdef DEBUG
        if (eeprom_data_loaded) {
            const static char txt[] PROGMEM = "Loaded configs from EEPROM";
            UART0_Println_P(txt);
        } else {
            const static char txt[] PROGMEM = "Set default config";
            UART0_Println_P(txt);
        }
#endif
    }
    wdt_reset();
    static fifo_descriptor fifo_storage;
    static uint8_t fifo_itemspace[NMEA_LINES][NMEA_LINE_LENGTH * sizeof(uint8_t)];
    fifo_storage.itemspace = (uint8_t*)fifo_itemspace;
    fifo_storage.itemsize = NMEA_LINE_LENGTH * sizeof(uint8_t);
    fifo_storage.allocatedbytes = NMEA_LINES * NMEA_LINE_LENGTH * sizeof(uint8_t);
    fifo_storage.readoffset = 0;
    fifo_storage.writeoffset = 0;
    fifo_storage.storedbytes = 0;

    msgout_buf = fifo_create_static(&fifo_storage, fifo_itemspace, NMEA_LINES, NMEA_LINE_LENGTH);

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
#ifdef HAS_I2C_LCD
    show_state = 1;
    show_ipa = 1;
    show_mesg_rate = 0;
    show_time = 0;

    ap_last_time = uptime;
    dgps_last_time = uptime;

    if (show_ipa) {
        lcd.setCursor(0, 3);
        IPAddress ipa = Ethernet.localIP();
        sprintf_P(g_buff, PSTR("%d.%d.%d.%d"), ipa[0], ipa[1], ipa[2], ipa[3]);

        for (uint8_t i = strlen(g_buff); i < 18; i++) {
            g_buff[i] = ' ';
        }
        lcd.print(g_buff);
    }
#endif
#ifdef WEB_GUI

#ifdef DEBUG
    const static char upd_txt[] PROGMEM = "Looking for updates on update server if SD card available ";
    UART0_Print_P(upd_txt);
    UART0_Println(update_host);
#endif
#ifdef SDC
    if (has_sd) {
        c.setTimeout(5000);
        if (c.connect(update_host, 80)) {
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
            c.stop();
#ifdef DEBUG
            sprintf_P(g_buff, PSTR("Build on update server: %ld local: %ld"), build_available, build);
            UART0_Println(g_buff);
#endif
            if (build == build_available) {
                if (SD.exists(FIRMWARE_FILE)) {
                    SD.remove(FIRMWARE_FILE);
#ifdef DEBUG
                    const static char txt1[] PROGMEM = "Remove old firmware file from SD card.";
                    UART0_Println_P(txt1);
#endif
                }
#ifdef HAS_I2C_LCD
                lcd.setCursor(0, 2);
                lcd.print(F("Firmware up to date "));
                delay(1500);
                lcd.setCursor(0, 2);
                lcd.print(F("                    "));
#endif

            }
#ifdef HAS_I2C_LCD
            else if (build_available > build) {
                lcd.setCursor(0, 2);
                lcd.print(F(">Firmware available<"));
                delay(1500);
                lcd.setCursor(0, 2);
                lcd.print(F("                    "));
            }
#endif
        }
    }
#endif
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

    sprintf_P(g_buff, PSTR("%s - MUX4s-1e started"), hostname);
    sendFreeText(g_buff);
    while (!fifo_is_empty(msgout_buf)) {
        send2All();
    }
#ifdef DEBUG
    sprintf_P(g_buff, PSTR("%s - MUX4s-1e started"), hostname);
    UART0_Println(g_buff);
    UART0_Flush();
#endif
    // Timer setup
    timerSetup();
    last_brd03_time = uptime;
    last_sat_n_time = uptime;
    key = random(0xffff);

    // enable serial interrupts on Serial 1 for LED blinking
    UCSR0B |= _BV(TXCIE0);

    static_send = false;
}

/**
 * Main loop, which is running all the time after setup() has finished. It checks for incoming data on all interfaces, and sends out messages from the output buffer.
 */
void loop() {
    wdt_reset();
    /**
     * READING & WRITING
     */

    // Get the incoming data

    if (Serial.available()) {
        readStream(Serial, P1);
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
        while (client) {                              // client is true only if it is connected and has data to read
            readStream(client, PETH1);
            while (!fifo_is_empty(msgout_buf)) {
                send2All();
            }
            client = server->available();
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
                    if (!memcmp(html_buffer, "GET /favicon.ico ", 17)) {
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
                    } else if (!memcmp(html_buffer, "POST /update ", 13)) {
                        post = 1;
                        update = 1;
                    } else if (!memcmp(html_buffer, "POST /download ", 13)) {
                        post = 1;
                        download = 1;
                    } else if (!memcmp(html_buffer, "POST /reset ", 12)) {
                        post = 1;
                        reset = 1;
                    } else /*if (!memcmp(html_buffer, "POST /upload ", 13)) {
                            post = 1;
                            upload = 1;
                    } else */
                        if (!memcmp(html_buffer, "POST / ", 7)) {
                            post = 1;
                        } else if (!memcmp(html_buffer, "HEAD / ", 7)) {
                            client.print((const __FlashStringHelper*)HEAD);
                        } else if (!memcmp(html_buffer, "GET /", 5)) {
                            get = 1;
                        }

                    // ----- Parsing POST Data

                    if (b_read == EMPTY_LINE && post && download) {
                        perform_download(build_available);
                        build = build_available;
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
                            wdt_reset();  // Needed here as reading the strings from flash takes some time
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
#ifdef DEBUG
                            sprintf_P(g_buff, PSTR("CRC from file: %lX CRC online: %lX Firmware: %s"), crc_32_calc, crc_32, SD.exists(FIRMWARE_FILE) ? "Present" : "Absent");
                            UART0_Println(g_buff);
#endif
                            if (crc_32_calc == crc_32) {  // && SD.exists(FIRMWARE_FILE)) {
                                client.print((const __FlashStringHelper*)UPDATE_FORM_1);
                                client.print(key, DEC);
                                client.print((const __FlashStringHelper*)UPDATE_FORM_2);
                            }
#endif
                        }
                        client.print((const __FlashStringHelper*)F5);
                        break;
                    } else if (b_read < 3 && reset && !image && !update) {
                        client.print((const __FlashStringHelper*)RESET_WAIT);
                        break;
                    }
                }
            }
            client.flush();
            // delay(1);
            client.stop();

            if (reset) {
                // Update requested?
                // if (update) {
                //     update = 0;
                //     char t[] = "Upd. Firmware";
                //     sendFreeText(t);
                //     while (!fifo_is_empty(msgout_buf)) {
                //         send2All();
                //     }
                // }

                // Software reset
                // char t[40];
                // sprintf_P(g_buff, PSTR("%s - Performing Reset"), hostname);
                // sendFreeText(g_buff);
                // while (!fifo_is_empty(msgout_buf)) {
                //     send2All();
                // }
                perform_reset();
            }
        }
#endif
    }
#endif

    if (uptime > last_uptime) {
#ifdef HAS_I2C_LCD
        // Calculate messages per minute coming in and going out and display on LCD. Also display if AP or DGPS data is received (last 5 seconds) and spoofing state. Send all this info as free text to all ports and MQTT if enabled.
        if (show_mesg_rate) {
            lcd.setCursor(1, 0);
            sprintf_P(g_buff, PSTR("%3umsg/s "), mps_in);
            lcd.print(g_buff);

            lcd.setCursor(1, 1);
            sprintf_P(g_buff, PSTR("%3umsg/s "), mps_out);
            lcd.print(g_buff);
        } else {
            lcd.setCursor(1, 0);
            sprintf_P(g_buff, PSTR("%7u  "), mesg_recv);
            lcd.print(g_buff);
            lcd.setCursor(1, 1);
            sprintf_P(g_buff, PSTR("%7u  "), mesg_sent);
            lcd.print(g_buff);
        }

#endif
        last_uptime = uptime;
    }

    if (static_send) {
        mps_in = (mesg_recv - last_mesg_recv) / 5;
        mps_out = (mesg_sent - last_mesg_sent) / 5;
        last_mesg_recv = mesg_recv;
        last_mesg_sent = mesg_sent;

        char has_ap_str[3];
        has_ap_str[2] = '\0';
        char has_dgps_str[5];
        has_dgps_str[4] = '\0';

        // sprintf_P(g_buff, PSTR("########## %ld %ld"), ap_last_time, uptime);
        // UART0_Println(g_buff);

        if (ap_last_time + 5 > uptime) {
            has_ap_str[0] = 'A';
            has_ap_str[1] = 'P';
        } else {
            has_ap_str[0] = ' ';
            has_ap_str[1] = ' ';
        }
        if (dgps_last_time + 5 > uptime) {
            has_dgps_str[0] = 'D';
            has_dgps_str[1] = 'G';
            has_dgps_str[2] = 'P';
            has_dgps_str[3] = 'S';
        } else {
            has_dgps_str[0] = ' ';
            has_dgps_str[1] = ' ';
            has_dgps_str[2] = ' ';
            has_dgps_str[3] = ' ';
        }
        lcd.setCursor(8, 2);

        sprintf_P(g_buff, PSTR(" %s %s "), has_ap_str, has_dgps_str);
        lcd.print(g_buff);

        uint16_t spoof_score_int = (uint16_t)spoof_score;
        uint16_t spoof_score_dec = (uint16_t)((spoof_score - spoof_score_int) * 10);

        sprintf_P(g_buff, PSTR("%s,%lu,%lu,%lu,%u,%lu,%u,%u.%u"), hostname, uptime, mesg_sent, mesg_recv, dgps_station_id, (uint32_t)dgps_time_s, valid_sats, spoof_score_int, spoof_score_dec);
        sendFreeText(g_buff);
#ifdef ETHERNET
        if (Ethernet.link() == 1) {
            if (mqttClient.connected()) {
                char buf2[48];
                sprintf_P(buf2, PSTR("vessels/self/mux/%s/text"), hostname);
                mqttClient.beginMessage(buf2);
                mqttClient.print(g_buff);
                mqttClient.endMessage();
            } else {
                connectMQTT(mqtt_broker, mqtt_port);
            }
        }

#ifdef HAS_I2C_LCD
        if (show_ipa) {
            lcd.setCursor(0, 3);
            IPAddress ipa = Ethernet.localIP();
            // ipa.printTo(lcd);
            memset(g_buff, 0, 20);
            sprintf_P(g_buff, PSTR("%d.%d.%d.%d"), ipa[0], ipa[1], ipa[2], ipa[3]);

            for (uint8_t i = strlen(g_buff); i < 18; i++) {
                g_buff[i] = ' ';
            }
            lcd.print(g_buff);
            show_ipa = 0;
        }

        if (show_state) {
            lcd.setCursor(17, 2);
            if (spoof_flag == 2) {
                lcd.print("SPF");
            } else if (spoof_flag == 1) {
                lcd.print("SUS");
            } else {
                lcd.print(" OK");
            }
            lcd.setCursor(18, 3);
            if (uptime - last_sat_n_time > 10) {
                g_buff[0] = '-';
                g_buff[1] = '-';
                g_buff[2] = '\0';
            } else {
                sprintf(g_buff, "%02u", valid_sats);
            }
            lcd.print(g_buff);
        }
        // if (show_pos) {

        /* dtostrf(lat, 7, 4, g_buff);
        lcd.print(g_buff);
        lcd.print(" ");
        dtostrf(lon, 8, 4, g_buff);
        lcd.print(g_buff);
        lcd.print(" "); */
        // }

#endif

#endif
        show_time = show_time ? 0 : 1;
        // show_state = show_state ? 0 : 1;
        show_mesg_rate = show_mesg_rate ? 0 : 1;
        static_send = false;
    }

    // Send out all messages left
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

/**
 * Send the buffer data to all ports.
 */
void send2All() {
    if (fifo_is_empty(msgout_buf)) {
        return;
    }

    static char nmea_line[NMEA_LINE_LENGTH];
    fifo_get(msgout_buf, nmea_line);

    if (matchFilter(nmea_line, &(p_config[P1].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        // if (nmea_line[0] & 0b11000010) {
        // uint8_t t = nmea_line[0];
        // nmea_line[0] &= 0b00111101;
#ifndef DEBUG
        sendStream(Serial, P1, nmea_line);
#else
        UART0_Println(nmea_line);
#endif
        // led_off();
        // nmea_line[0] = t;
        // }
    }
    if (matchFilter(nmea_line, &(p_config[P2].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        // if ((nmea_line[0] & 0b11000010) != 0b01000000) {
        // uint8_t t = nmea_line[0];
        // nmea_line[0] &= 0b00111101;
        sendStream(Serial1, P2, nmea_line);
        // nmea_line[0] = t;
        // }
    }
    if (matchFilter(nmea_line, &(p_config[P3].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        // if ((nmea_line[0] & 0b11000010) != 0b10000000) {
        // uint8_t t = nmea_line[0];
        // nmea_line[0] &= 0b00111101;
        sendStream(Serial2, P3, nmea_line);
        // nmea_line[0] = t;
        // }
    }
    if (matchFilter(nmea_line, &(p_config[P4].ofilter))) {
        // Check if it has come from the same source. Avoid to echo if so.
        // if ((nmea_line[0] & 0b11000010) != 0b11000000) {
        // uint8_t t = nmea_line[0];
        // nmea_line[0] &= 0b00111101;
        sendStream(Serial3, P4, nmea_line);
        // nmea_line[0] = t;
        // }
    }
#ifdef ETHERNET
    // Is OUT active?
    if (Ethernet.link() == 1) {
        if ((p_config[PETH1].direction & OUT)) {
            if (matchFilter(nmea_line, &(p_config[PETH1].ofilter))) {
                // Check if it has come from the same source. Avoid to echo if so.
                // if ((nmea_line[0] & 0b11000010) != 0b000000010) {
                // nmea_line[0] &= 0b00111101;
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
                // #ifdef HAS_I2C_LCD
                //                 lcd.setCursor(2, 1);
                //                 lcd.print(mesg_sent, DEC);
                // #endif
                // }
            }
        }

        if (mqttClient.connected()) {
            // char buf[48];
            nmea_line[0] = '$';
            if (checkChecksum(nmea_line) != 0) {
                // nmea* result;
                static char id[6];
                static char sentence[NMEA_LINE_LENGTH - 7];
                // extractNMEA(nmea_line, result);
                memcpy(id, nmea_line + 1, 5);
                memcpy(sentence, nmea_line + 7, NMEA_LINE_LENGTH - 7);
                id[5] = 0;
                if (id[0] != '\0') {
                    sprintf_P(g_buff, PSTR("vessels/self/nmea0183/%s/%s"), hostname, id);
                    mqttClient.beginMessage(g_buff);
                    mqttClient.print(sentence);
                    mqttClient.endMessage();
                }
                // free(result);
                // mqttClient.stop();
            }
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

/**
 * USART0 Transmit Complete Interrupt Service Routine.
 * Just to switch off the LED after sending a message.
 */
ISR(USART0_TX_vect) {
    led_off();
}

/**
 * Timer Routine.
 * Just counting seconds
 */
ISR(TIMER1_COMPA_vect) {
    uptime++;
    // zda_out_counter++;
    if (!(uptime % 05)) {
        static_send = true;
    }
}

/**
 * Update the validity of the satellites based on the last GSV message and the SNR and elevation values.
 */
void update_sat_validity(void) {
    for (uint8_t i = 0; i < MAX_SATS; i++) {
        sat_t* s = &sats[i];

        s->valid = 0;

        if (!s->has_gsv)
            continue;

        if ((uptime - s->t_gsv) > SAT_TIMEOUT)
            continue;

        if (s->snr < 8)
            continue;

        if (s->el < 3)
            continue;

        s->valid = 1;
    }
}
/**
 * Read the bytes from a stream
 */
uint8_t readStream(Stream& stream, const uint8_t port_num) {
    // Is IN active or buffer full
    if (!(p_config[port_num].direction & IN) || fifo_is_full(msgout_buf)) {
        return 0;
    }

    if (port_num == P1) {
        led_on();
    }

    // uint8_t read = 0;
    static char nmea_line[NMEA_LINE_LENGTH];

    // We have three conditions for a "full" read
    // 1. bytes available less then NMEA_LINE_LENGTH
    // 2. '\n' in read buffer
    // 3. bytes do not contain '\n' (or '\r') but equals NMEA_LINE_LENGTH

    // Case 1. normal read
    // Case 2. check for '\n' ('\r')
    // Case 3. continue read with next buffer (but not if then full)

    int8_t b_read = 0;
    while (stream.available()) {
        // There is something that wants to be read

        b_read = stream.readBytesUntil('\n', nmea_line, NMEA_LINE_LENGTH);
        if (b_read < 1) {
            continue;
        }

        // We have a full line
        if (nmea_line[b_read - 1] == '\r') {
            nmea_line[b_read - 1] = 0;
            if (!matchFilter(nmea_line, &(p_config[port_num].ifilter)) || !checkChecksum(nmea_line)) {
                continue;
            }

            parseNMEA(nmea_line);

            /**
             *  We are setting a mark/mask to avoid sending back the traffic from where it arrives (see send2All())
             */
            /* switch (port_num) {
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
            } */
#ifdef TRANSLATE_

            // Get the sentence identifier
            char id[4];  // The NMEA0183 sentence name
            id[3] = 0;   // zero for a char array
            strncpy(id, nmea_line + 3, 3);
            if (!strcmp(id, NMEA_DPT) && checkChecksum(nmea_line)) {
                // Store the original message as it will be changed bleow
                fifo_add(msgout_buf, nmea_line);
                mesg_recv++;  // got one more
#ifdef HAS_I2C_LCD
                lcd.setCursor(2, 1);
                lcd.print(mesg_recv, DEC);
#endif

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

#ifdef HAS_I2C_LCD
            lcd.setCursor(9, 0);
            if (fifo_add(msgout_buf, nmea_line)) {
                mesg_recv++;
                lcd.write(' ');
            } else {
                lcd.write(0x02);  // full symbol
            }
#else
            if (fifo_add(msgout_buf, nmea_line)) {
                mesg_recv++;
            }
#endif

            // #ifdef HAS_I2C_LCD

            //             lcd.print(mesg_recv, DEC);
            // #endif
            continue;
        } else {
            // No valid line rexeived
            b_read = 0;
            break;
        }
    }
    // Flush
    while (stream.read() > -1);

    if (port_num == P1) {
        led_off();
    }

    return b_read;
}

/**
 * Send the bytes to a stream
 */
void sendStream(Stream& stream, const uint8_t port_num, char* nmea_line) {
    // Is OUT active on that port?
    if (!(p_config[port_num].direction & OUT)) {
        return;
    }

    if (port_num == P1) {
        led_on();
    }

    // Send the line to the stream. We are looking for the end of the string to avoid sending empty characters. If there is no end, we send the whole buffer.
    const char* ptr = strchr(nmea_line, '\0');
    if (ptr) {
        int16_t i = ptr - nmea_line;
        if (i > 0) {
            stream.write(nmea_line, i);
            stream.write(crlf, 2);  // line break
        }
    } else {
        stream.write(nmea_line, NMEA_LINE_LENGTH);
        stream.write(crlf, 2);  // line break
    }
    mesg_sent++;
    if (port_num == P1) {
        led_off();
    }
}

/**
 * Compare the sentence with the filter element
 * @return 0 if no match, 1 if match
 */
// +MXTXT:-all
uint8_t matchFilter(const char* nmea_line, Filter* filter) {
    uint8_t match = 1;
    for (uint8_t j = 0; j < FILTER_SIZE; j++) {
        match = 1;
        for (uint8_t i = 1; i <= 5; i++) {
            if (filter->pattern[j][0] == 0) {
                return filter->pm_all;
            } else if (filter->pattern[j][i] == '*')
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

/**
 * Send a free text message to all ports. The text is sent as $MXTXT,text*hh, where hh is the checksum of the message without the $ and *hh. The text can be up to 30 characters long (NMEA_LINE_LENGTH - 7).
 */
void sendFreeText(char* text) {
    char nmea_line[NMEA_LINE_LENGTH];

    sprintf_P(nmea_line, PSTR("$MXTXT,%s*"), text);
    const uint8_t cs = calcChecksum(nmea_line);
    sprintf_P(nmea_line, PSTR("$MXTXT,%s*%02X"), text, cs);
    fifo_add(msgout_buf, nmea_line);
}

#ifdef SDC
#ifdef DEBUG
/**
 * Print the error message for the given error code
 */
void printErrorMessage(uint8_t e, bool eol) {
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
}
#endif
#endif

/**
 * Calculate the CRC32 of a file. The charcnt is the number of characters read from the file.
 */
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

/**
 * Update the CRC32 with the given byte. The crc is the current CRC value.
 */
inline uint32_t updateCRC32(uint8_t ch, uint32_t crc) {
    uint32_t idx = ((crc) ^ (ch)) & 0xff;
    uint32_t tab_value = pgm_read_dword(crc_32_tab + idx);
    return tab_value ^ ((crc) >> 8);
}

#ifdef WEB_GUI
/**
 * Print the filter pattern to the stream. If the pattern is empty, print +all or -all depending on the pm_all flag.
 */
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
#ifdef DGPS

/* ================= SAT MANAGEMENT ================= */
/**
 * Get the satellite struct for the given PRN. If it does not exist, allocate an empty slot. If there is no empty slot, return NULL.
 */
sat_t* get_sat(uint8_t prn) {
    for (int i = 0; i < MAX_SATS; i++) {
        if (sats[i].prn == prn)
            return &sats[i];
    }

    // allocate empty slot
    for (int i = 0; i < MAX_SATS; i++) {
        if (sats[i].prn == 0) {
            sats[i].prn = prn;
            return &sats[i];
        }
    }

    return NULL;
}

/* ================= PARSERS ================= */

/**
 * Parse the BRD03 message to extract the DGPS reference station information
 * Example:
 * $BRD03,A,1234,5678.9,0,123456.789,12.345,67.890*hh
 * Field Number:
 * 1) Status (A=active, V=void)
 * 2) Station ID
 * 3) Time (s)
 * 4) (not used)
 * 5) X (m)
 * 6) Y (m)
 * 7) Z (m)
 * 8) Checksum
 */
void parse_brd03(char* local) {
    char* p = local;
    char* f;
    int field = 0;

    char status = 'V';
    uint16_t station_id = 0;
    float time_s = 0;

    float x = 0, y = 0, z = 0;

    while ((f = next_field(&p))) {
        field++;

        if (field == 2) {
            status = f[0];
        } else if (field == 3) {
            station_id = (uint16_t)atoi(f);
        } else if (field == 4) {
            time_s = atof(f);
        } else if (field == 6) {
            x = atof(f);
        } else if (field == 7) {
            y = atof(f);
        } else if (field == 8) {
            z = atof(f);
        }
    }

    if (status != 'A')
        return;

    // store globally
    dgps_station_id = station_id;
    dgps_time_s = time_s;

    dgps_ref_x = x;
    dgps_ref_y = y;
    dgps_ref_z = z;

    dgps_ref_valid = 1;
    dgps_ref_timestamp = uptime;
}

/**
 * Parse the BRD09 message to extract the DGPS correction information
 * Example:
 * $BRD09,A,1234,5678.9,0,123456.789,12.345,67.890*hh
 * Field Number:
 * 1) Status (A=active, V=void)
 * 2) Station ID
 * 3) Time (s)
 * 4) (not used)
 * 5) PRN
 * 6) Correction (m)
 * 7) (not used)
 * 8) Azimuth (degrees)
 * 9) Checksum
 */
void parse_brd09(char* local) {
    char* p = local;
    char* f;
    int field = 0;

    uint8_t prn = 0;
    float corr = 0.0f;
    float az = 0.0f;

    while ((f = next_field(&p))) {
        field++;

        if (field == 6)
            prn = (uint8_t)atoi(f);
        else if (field == 7)
            corr = atof(f);
        else if (field == 9)
            az = atof(f);
    }

    sat_t* s = get_sat(prn);
    if (s) {
        float alpha = 0.2f;  // smoothing factor

        s->corr = (1.0f - alpha) * s->corr + alpha * corr;
        s->az = az;
        s->t_corr = uptime;
        s->has_corr = 1;
    }
}

/**
 * Parse the GSV message to extract the satellite information
 * Example:
 * $GPGSV,2,1,08,01,40,083,41,02,17,273,43,03,27,123,42,04,13,213,40*hh
 * Field Number:
 * 1) Total number of messages of this type in this cycle
 * 2) Message number
 * 3) Total number of satellites in view
 */
void parse_gsv(char* local) {
    char* p = local;
    char* f;
    uint8_t field = 0;

    uint8_t prn = 0;

    while ((f = next_field(&p))) {
        field++;

        // --- GSV header ---
        if (field == 2) {
            gsv_total = atoi(f);

            // new cycle starts → reset tracking
            if (gsv_total > 0 && gsv_msg == 1) {
                gsv_mask = 0;
                gsv_seen = 0;
            }
        } else if (field == 3) {
            gsv_msg = atoi(f);
        }

        // --- Satellite blocks start at field 5 ---
        else if (field >= 5) {
            uint8_t offset = (field - 5) % 4;

            if (offset == 0) {
                // PRN
                prn = (uint8_t)atoi(f);
            } else {
                sat_t* s = get_sat(prn);
                if (!s) continue;

                if (offset == 1) {
                    // Elevation
                    s->el = atof(f);
                } else if (offset == 2) {
                    // Azimuth
                    s->az = atof(f);
                } else if (offset == 3) {
                    // SNR
                    if (f[0] != '\0')
                        s->snr = atof(f);
                    else
                        s->snr = 0.0f;

                    // IMPORTANT:
                    // Do NOT set valid or has_gsv here
                    // Just timestamp raw reception
                    s->t_gsv = uptime;
                }
            }
        }
    }
    if (gsv_total > 0 && gsv_msg > 0 && gsv_msg <= 8) {
        gsv_mask |= (1 << (gsv_msg - 1));
        gsv_seen++;
    }

    uint8_t expected = (1 << gsv_total) - 1;

    if (gsv_mask == expected) {
        gsv_cycle_complete = 1;
    }
}
void finalize_gsv_cycle(void) {
    if (!gsv_cycle_complete)
        return;

    for (int i = 0; i < MAX_SATS; i++) {
        if (sats[i].snr > 0) {
            sats[i].has_gsv = 1;
            sats[i].t_gsv = uptime;
        }
    }

    gsv_cycle_complete = 0;

    // reset cycle tracking
    gsv_total = 0;
    gsv_msg = 0;
#ifdef DEBUG
    UART0_Println("GSV cycle complete");
#endif
}

/**
 * Parse the RMC message to extract the current position and time information
 * Example:
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*hh
 * Field Number:
 * 1) Time (UTC)
 * 2) Status (A=active, V=void)
 * 3) Latitude (ddmm.mmm)
 * 4) N/S Indicator
 * 5) Longitude (dddmm.mmm)
 * 6) E/W Indicator
 * 7) Speed over ground (knots)
 * 8) Track angle in degrees
 * 9) Date (ddmmyy)
 * 10) Magnetic Variation (degrees)
 * 11) Magnetic Variation E/W Indicator
 * 12) Checksum
 */
void parse_rmc(char* local) {
    char* p = local;
    char* f;
    int field = 0;

    // char status = 'V';
    float lat = 0.0f, lon = 0.0f;
    char ns = 'N', ew = 'E';

    while ((f = next_field(&p))) {
        field++;
        if (field == 2) {
            memcpy(rmc.time, f, 9);
            rmc.time[9] = '\0';
        } else if (field == 3)
            rmc.status = f[0];
        if (field == 4)
            lat = atof(f);
        else if (field == 5)
            ns = f[0];
        else if (field == 6)
            lon = atof(f);
        else if (field == 7)
            ew = f[0];
        else if (field == 8)
            rmc.sog = atof(f);
        else if (field == 9)
            rmc.cog = atof(f);
        else if (field == 10) {
            memcpy(rmc.date, f, 6);
            rmc.date[6] = '\0';
        } else if (field == 11)
            rmc.mv = atof(f);
        else if (field == 12)
            rmc.mv_direction = f[0];
    }

    // int lat_d = (int)(lat / 100);
    // float lat_m = lat - lat_d * 100;
    // current_lat = lat_d + lat_m / 60.0f;

    // int lon_d = (int)(lon / 100);
    // float lon_m = lon - lon_d * 100;
    // current_lon = lon_d + lon_m / 60.0f;

    // if (ns == 'S') current_lat = -current_lat;
    // if (ew == 'W') current_lon = -current_lon;
}

/**
 * Parse the GGA message to extract the current position and DGPS information
 * Example:
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*hh
 * Field Number:
 * 1) Time (UTC)
 * 2) Latitude (ddmm.mmm)
 * 3) N/S Indicator
 * 4) Longitude (dddmm.mmm)
 * 5) E/W Indicator
 * 6) Fix Quality (0 = invalid, 1 = GPS fix, 2 = DGPS fix)
 * 7) Number of Satellites
 * 8) Horizontal Dilution of Precision (HDOP)
 * 9) Altitude (m)
 * 10) Altitude Units
 * 11) Geoidal Separation (m)
 * 12) Geoidal Separation Units
 * 13) DGPS Age (s)
 * 14) DGPS Station ID
 * 15) Checksum
 */
void parse_gga(char* local) {
    /*     char local[NMEA_LINE_LENGTH];
        strncpy(local, line, NMEA_LINE_LENGTH);
        local[NMEA_LINE_LENGTH - 1] = '\0'; */

    char* p = local;
    char* f;
    int field = 0;

    float lat = 0.0f, lon = 0.0f;
    char ns = 'N', ew = 'E';

    while ((f = next_field(&p))) {
        field++;
        if (field == 2) {
            memcpy(gga.time, f, 9);
            gga.time[9] = '\0';
        }
        if (field == 3)
            lat = atof(f);
        else if (field == 4)
            ns = f[0];
        else if (field == 5)
            lon = atof(f);
        else if (field == 6)
            ew = f[0];
        else if (field == 7)
            gga.fix_quality = atoi(f);
        else if (field == 8)
            gga.num_satellites = atoi(f);
        else if (field == 9)
            gga.horizontal_dilution = atof(f);
        else if (field == 10)
            gga.altitude = atof(f);
        else if (field == 11)
            gga.altitude_units = f[0];
        else if (field == 12)
            gga.undulation = atof(f);
        else if (field == 13)
            gga.undulation_units = f[0];
        else if (field == 14)
            gga.dgps_age = atof(f);
        // else if (field == 15)
        //     strncpy(gga.dgps_station_id, f, 6);
    }

    int lat_d = (int)(lat / 100);
    float lat_m = lat - lat_d * 100;
    current_lat = lat_d + lat_m / 60.0f;

    int lon_d = (int)(lon / 100);
    float lon_m = lon - lon_d * 100;
    current_lon = lon_d + lon_m / 60.0f;

    if (ns == 'S') current_lat = -current_lat;
    if (ew == 'W') current_lon = -current_lon;
}
/* void parse_zda(char* local) {
    zda_out_counter = 0;

    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t dd;
    uint8_t mt;
    uint16_t yr;
    char* p = local;
    char* f;
    uint8_t field = 0;
    uint8_t run = 1;
    char id[3];
    id[2] = 0;

    while (run && (f = next_field(&p))) {
        field++;
        if (field == 2) {
            hh = atoi(strncpy(id, f, 2));
            mm = atoi(strncpy(id, f + 2, 2));
            ss = atoi(strncpy(id, f + 4, 2));
        } else if (field == 3) {
            dd = atoi(strncpy(id, f, 2));
        } else if (field == 4) {
            mt = atoi(strncpy(id, f, 2));
        } else if (field == 5) {
            yr = atoi(strncpy(id, f, 4));
            char buf[11];
            lcd.setCursor(0, 3);
            sprintf(buf, "%02u:%02u:%02uz %02u.%02u.%u", hh, mm, ss, dd, mt, yr);
            lcd.print(buf);

            run = 0;
        }
    }
} */

/**
 * Solve the least squares problem for DGPS correction
 * @param dE Pointer to the eastward correction
 * @param dN Pointer to the northward correction
 * @return 1 if successful, 0 otherwise
 */
static uint8_t solve_lsq(float* dE, float* dN) {
    float HtH[3][3] = {0};
    float HtV[3] = {0};

    uint8_t count = 0;

    for (int i = 0; i < MAX_SATS; i++) {
        sat_t* s = &sats[i];

        if (!s->valid) continue;
        if (s->snr < 8) continue;
        if (s->el < 3) continue;
        if (s->corr < -20 || s->corr > 20) continue;

        float az = deg2rad(s->az);
        float el = deg2rad(s->el);

        float uE = cosf(el) * sinf(az);
        float uN = cosf(el) * cosf(az);

        float H[3] = {uE, uN, 1.0f};
        float v = s->corr;

        float w = (s->snr / 50.0f);

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                HtH[r][c] += H[r] * H[c] * w;
            }
            HtV[r] += H[r] * v * w;
        }

        count++;
    }

    if (count < 4)
        return 0;

    // --- solve ---
    float A[3][4] = {
        {HtH[0][0], HtH[0][1], HtH[0][2], HtV[0]},
        {HtH[1][0], HtH[1][1], HtH[1][2], HtV[1]},
        {HtH[2][0], HtH[2][1], HtH[2][2], HtV[2]}};

    for (int i = 0; i < 3; i++) {
        float pivot = A[i][i];
        if (fabsf(pivot) < 1e-6f)
            return 0;

        for (int j = i; j < 4; j++)
            A[i][j] /= pivot;

        for (int k = 0; k < 3; k++) {
            if (k == i) continue;
            float f = A[k][i];
            for (int j = i; j < 4; j++)
                A[k][j] -= f * A[i][j];
        }
    }

    *dE = A[0][3];
    *dN = A[1][3];

    return 1;
}

/**
 * Compute the corrected position using the DGPS corrections and outlier rejection
 * This is the main function that implements the DGPS correction algorithm
 * It consists of two passes:
 * 1. Compute the initial correction using all valid satellites
 * 2. Reject outliers based on the residuals and recompute the correction
 */
void compute_corrected(float* lat_out, float* lon_out) {
    float lat = current_lat;
    float lon = current_lon;

    float dE = 0, dN = 0;

    // --- PASS 1 ---
    if (!solve_lsq(&dE, &dN)) {
        *lat_out = lat;
        *lon_out = lon;
        return;
    }

    // --- OUTLIER REJECTION (THIS IS POINT 3) ---
    for (int i = 0; i < MAX_SATS; i++) {
        sat_t* s = &sats[i];
        if (!s->valid) continue;

        float az = deg2rad(s->az);
        float el = deg2rad(s->el);

        float uE = cosf(el) * sinf(az);
        float uN = cosf(el) * cosf(az);

        float predicted = uE * dE + uN * dN;
        float residual = s->corr - predicted;

        if (fabsf(residual) > 5.0f) {
            s->valid = 0;  // 🚨 reject bad satellite
        }
    }

    // --- PASS 2 (clean solution) ---
    if (!solve_lsq(&dE, &dN)) {
        *lat_out = lat;
        *lon_out = lon;
        return;
    }
    cx = dE;
    cy = dN;
    // --- APPLY ---
    float lat0 = lat;
    lat += dN / 111320.0f;
    lon += dE / (111320.0f * cosf(deg2rad(lat0)));

    *lat_out = lat;
    *lon_out = lon;
}
/* ================= KALMAN ================= */

void kf_update(float lat, float lon) {
    if (!kf.init) {
        kf.lat = lat;
        kf.lon = lon;
        kf.p_lat = kf.p_lon = 1.0f;
        kf.init = 1;
        return;
    }

    float q = 1e-5f;
    float r = 1e-4f;

    kf.p_lat += q;
    kf.p_lon += q;

    float k_lat = kf.p_lat / (kf.p_lat + r);
    float k_lon = kf.p_lon / (kf.p_lon + r);

    kf.lat += k_lat * (lat - kf.lat);
    kf.lon += k_lon * (lon - kf.lon);

    kf.p_lat *= (1.0f - k_lat);
    kf.p_lon *= (1.0f - k_lon);
}
#endif

/**
 * Count the number of valid satellites
 */
uint8_t count_valid_sats(void) {
    uint8_t n = 0;

    for (uint8_t i = 0; i < MAX_SATS; i++) {
        sat_t* s = &sats[i];
#ifdef DEBUG
        char buf[32];
        if (s->has_gsv) {
            sprintf_P(buf, PSTR("GSV PRN %d OK"), s->prn);
            UART0_Println(buf);
        }
        if (s->has_corr) {
            sprintf_P(buf, PSTR("CORR PRN %d OK"), s->prn);
            UART0_Println(buf);
        }
#endif
        if (s->valid)
            n++;
    }

    return n;
}
#ifdef TRANSLATE
void parse_dpt(char* local) {
    // Store the original message as it will be changed bleow
    fifo_add(msgout_buf, local);
    mesg_recv++;  // got one more
                  // #ifdef HAS_I2C_LCD
                  //     lcd.setCursor(2, 0);
                  //     lcd.print(mesg_recv, DEC);
                  // #endif

    // store the input port check character
    // char check = local[0];

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
    char* token = strtok(local, ",");

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

        sprintf_P(g_buff, PSTR("$MXDBS,%s,f,%s,M,%s,F*"), buf_f, buf_m, buf_fh);
        const uint8_t cs = calcChecksum(g_buff);

        sprintf(local, "%s%02X", g_buff, cs);
        // nmea_line[0] = check;
    }

    // $IIDPT,3.5,0.5*43
    // $IIDPT,30.5,0.5*73
    // $IIDPT,200.5,0.5*42
    // $IIDPT,600.5,0.5*46
}
#endif
void perform_reset(void) {
#ifdef HAS_I2C_LCD
    lcd.clear();
    lcd.print(F("Performing Reset"));
#endif
    cli();
    ADCSRA = 0;
    TCCR0A = 0;
    TCCR0B = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR3A = 0;
    TCCR3B = 0;
    TCCR4A = 0;
    TCCR4B = 0;
    UCSR0A = 0;
    UCSR0B = 0;
    UCSR1A = 0;
    UCSR1B = 0;
    UCSR2A = 0;
    UCSR2B = 0;
    UCSR3A = 0;
    UCSR3B = 0;
    c.stop();
    server->flush();
    server1.flush();
    free(server);
    wdt_enable(WDTO_1S);
    stop();
    while (1);
}
void perform_download(const uint16_t build_available) {
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
        // uint8_t h_buf[128];

        if (SD.exists(FIRMWARE_FILE)) {
            SD.remove(FIRMWARE_FILE);
        }

        uint16_t b_read = 0;
        while (c.connected() && !c.available()) delay(1);  // wait for data
        File fh = SD.open(FIRMWARE_FILE, O_WRITE | O_CREAT);
        uint8_t skipped = 0;
#ifdef HAS_I2C_LCD
        lcd.setCursor(0, 2);
        lcd.print(F("                    "));  // clear line
        lcd.setCursor(0, 3);
        lcd.print(F("                    "));  // clear line
#endif
        static uint32_t b_count = 0;
        while (c.connected() || c.available()) {
            wdt_reset();
            if (!skipped)
                b_read = c.readBytesUntil('\n', g_buff, 48);

            // skip HTTP header
            if (!skipped && b_read <= 1) {
                skipped = 1;
                continue;
            }

            if (skipped) {
                b_read = c.readBytes(g_buff, 48);
                b_count += b_read;
                fh.write(g_buff, b_read);
                if (!(b_count % 512)) {
                    led_toggle();
#ifdef HAS_I2C_LCD
                    lcd.setCursor(0, 2);
                    sprintf_P(g_buff, PSTR("Downloading: %luKb"), b_count / 1024);
                    lcd.print(g_buff);
                    uint8_t steps = ((float)b_count / (float)filesize) * 19;
                    for (uint8_t i = 0; i <= steps; i++) {
                        g_buff[i] = 0xff;
                    }
                    g_buff[steps] = 0xff;
                    g_buff[steps + 1] = 0;
                    lcd.setCursor(0, 3);
                    lcd.print(g_buff);
#endif
                }
            }
            // }
        }

        fh.close();
        fh = SD.open(FIRMWARE_FILE, FILE_READ);
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
        // if (crc_32 == crc_32_calc) {
        //     // eeprom_write_byte((uint8_t*)EEPROM_FLAG, EEPROM_FLAG_ENABLE);  // set the flag to tell the bootloader to check for new firmware
        // }

        fh.close();
        led_off();
#ifdef HAS_I2C_LCD
        lcd.setCursor(0, 3);
        lcd.print(F("                    "));  // clear line
        lcd.setCursor(0, 3);
        sprintf_P(g_buff, PSTR("%lX / %lX "), crc_32_calc, crc_32);
        lcd.print(g_buff);
#endif
    }
    c.stop();
}
void parse_cmd(char* cmd_line) {
    // $MXCMD,command*hh

    char* token = strtok(cmd_line, "*");

    token = strtok(token, ",");  // this should be $MXCMD
    token = strtok(NULL, ",");
    if (strcmp(token, "download") == 0) {
        token = strtok(NULL, ",");
        if (token) {
            const uint16_t version = (uint16_t)strtoul(token, NULL, 10);
            build_available = version;
            perform_download(version);
            build = version;
        }
    } else if (strcmp(token, "reset") == 0) {
        perform_reset();
    } else if (strcmp(token, "unlock") == 0) {
        uptime_admin = uptime;
    }
}

/**
 * Parse the NMEA line and update internal state accordingly
 */
void parseNMEA(const char* nmea_line) {
    static char tempstr[NMEA_LINE_LENGTH];
    memcpy(tempstr, nmea_line, NMEA_LINE_LENGTH);
    tempstr[NMEA_LINE_LENGTH - 1] = '\0';

    static char nmea_buf[NMEA_LINE_LENGTH - 9];

#ifdef TRANSLATE
    if (memcmp(tempstr + 3, NMEA_DPT, 3) == 0) {
        // DPT is used for testing the translation feature, so we parse it even if translation is disabled
        parse_dpt(tempstr);
    } else
#endif
        if (memcmp(tempstr + 1, "AP", 2) == 0) {
        ap_last_time = uptime;
    } else if (memcmp(tempstr + 1, "MXCMD", 5) == 0) {
        parse_cmd(tempstr);
    }
#ifdef DGPS
    else if (memcmp(tempstr + 3, "D03", 3) == 0) {
        parse_brd03(tempstr);

#ifdef HAS_I2C_LCD

        /* ecef_to_llh(dgps_ref_x, dgps_ref_y, dgps_ref_z,
                    &ref_lat, &ref_lon, &ref_alt);
        char ref_lat_str[8];
        dtostrf(ref_lat, 7, 4, ref_lat_str);
        char ref_lon_str[9];
        dtostrf(ref_lon, 8, 4, ref_lon_str);
        lcd.setCursor(0, 0);
        lcd.print(dgps_station_id);
        lcd.print(" ");
        lcd.print(ref_lat_str);
        lcd.print(" ");
        lcd.print(ref_lon_str);

        // char buffer[19];
        g_buff[18] = '\0';
         */

        last_brd03_time = uptime;
#endif
    } else if (memcmp(tempstr + 3, "D09", 3) == 0) {
        parse_brd09(tempstr);
        dgps_last_time = uptime;
    } else if (memcmp(tempstr + 2, "PGSV", 4) == 0) {
        parse_gsv(tempstr);
        // if (gsv_cycle_complete) {
        finalize_gsv_cycle();
        // }
    } else if (memcmp(tempstr + 3, "RMC", 3) == 0) {
        parse_rmc(tempstr);
        // build new sentence with DGPS
        uint8_t degrees_lat = (uint8_t)lat;
        uint8_t minutes_lat = (uint8_t)((lat - degrees_lat) * 60);

        uint8_t degrees_lon = (uint8_t)lon;
        uint8_t minutes_lon = (uint8_t)((lon - degrees_lon) * 60);

        uint32_t ssss = (((lat - degrees_lat) * 60 - minutes_lat)) * 1000000;
        uint32_t lon_ssss = (((lon - degrees_lon) * 60 - minutes_lon)) * 1000000;

        uint16_t sog_int = (uint16_t)rmc.sog;
        uint16_t sog_dec = (uint16_t)((rmc.sog - sog_int) * 10);

        uint16_t cog_int = (uint16_t)rmc.cog;
        uint16_t cog_dec = (uint16_t)((rmc.cog - cog_int) * 10);

        uint16_t mv_int = (uint16_t)rmc.mv;
        uint16_t mv_dec = (uint16_t)((rmc.mv - mv_int) * 10);

        snprintf_P(nmea_buf, sizeof(nmea_buf), PSTR("%s,%c,%02u%02u.%06lu,%c,%03u%02u.%06lu,%c,%u.%01u,%u.%01u,%s,%u.%01u,%c"), rmc.time, rmc.status, degrees_lat, minutes_lat, ssss, (lat >= 0) ? 'N' : 'S', degrees_lon, minutes_lon,
                   lon_ssss, (lon >= 0) ? 'E' : 'W', sog_int, sog_dec, cog_int, cog_dec, rmc.date, mv_int, mv_dec, rmc.mv_direction);
        char nmea_line[NMEA_LINE_LENGTH];
        sprintf_P(nmea_line, PSTR("$MXRMC,%s*"), nmea_buf);
        const uint8_t cs = calcChecksum(nmea_line);
        sprintf_P(nmea_line, PSTR("$MXRMC,%s*%02X"), nmea_buf, cs);

        // add to fifo for output
        fifo_add(msgout_buf, nmea_line);
        mesg_sent++;  // got one more
    } else if (memcmp(tempstr + 2, "PGGA", 4) == 0) {
        parse_gga(tempstr);
        update_sat_validity();
        valid_sats = count_valid_sats();
        last_sat_n_time = uptime;
        if (valid_sats >= 4) {
            compute_corrected(&lat, &lon);
        } else {
            lat = current_lat;
            lon = current_lon;
        }
        // float correction_magnitude = sqrtf(cx * cx + cy * cy);
        float correction_mag2 = cx * cx + cy * cy;
        float corr_score = 0.0f;
        if (correction_mag2 > 25.0f) corr_score = 0.5f;
        if (correction_mag2 > 225.0f) corr_score = 1.0f;

        float avg_snr = 0;
        int count = 0;

        for (int i = 0; i < MAX_SATS; i++) {
            if (!sats[i].valid) continue;
            avg_snr += sats[i].snr;
            count++;
        }

        if (count > 0) avg_snr /= count;

        float snr_score = 0.0f;
        if (avg_snr < 20) snr_score = 0.7f;
        if (avg_snr < 10) snr_score = 1.0f;

        float sat_score = 0.0f;

        if (gga.num_satellites < 4 && gga.fix_quality > 0)
            sat_score = 0.8f;

        float hdop_score = 0.0f;

        if (gga.horizontal_dilution > 3.0f) hdop_score = 0.5f;
        if (gga.horizontal_dilution > 6.0f) hdop_score = 1.0f;

        float residual = sqrtf((lat - kf.lat) * (lat - kf.lat) +
                               (lon - kf.lon) * (lon - kf.lon));

        float kf_score = 0.0f;

        if (residual > 0.0001f) kf_score = 0.5f;
        if (residual > 0.0005f) kf_score = 1.0f;

        kf_update(lat, lon);

        sp_init(&sp);

        sp_add(&sp, "corr", corr_score, 0.25f);
        sp_add(&sp, "snr", snr_score, 0.20f);
        sp_add(&sp, "sat", sat_score, 0.15f);
        sp_add(&sp, "hdop", hdop_score, 0.15f);
        sp_add(&sp, "kf", kf_score, 0.25f);

        spoof_score = sp_final(&sp);

        if (spoof_score > 0.8f) {
            spoof_flag = 2;  // strong spoof
        } else if (spoof_score > 0.6f) {
            spoof_flag = 1;  // suspicious
        } else {
            spoof_flag = 0;
        }

        uint8_t degrees_lat = (uint8_t)lat;
        uint8_t minutes_lat = (uint8_t)((lat - degrees_lat) * 60);

        uint8_t degrees_lon = (uint8_t)lon;
        uint8_t minutes_lon = (uint8_t)((lon - degrees_lon) * 60);

        uint32_t ssss = (((lat - degrees_lat) * 60 - minutes_lat)) * 1000000;
        uint32_t lon_ssss = (((lon - degrees_lon) * 60 - minutes_lon)) * 1000000;
        char hdop_prec[4];
        dtostrf(gga.horizontal_dilution, 3, 1, hdop_prec);

        int16_t antenna_int = (int16_t)gga.altitude;
        uint16_t antenna_dec = (uint16_t)((abs(gga.altitude) - abs(antenna_int)) * 10);

        uint16_t undulation_int = (uint16_t)gga.undulation;
        uint16_t undulation_dec = (uint16_t)((gga.undulation - undulation_int) * 10);

        snprintf_P(nmea_buf, sizeof(nmea_buf), PSTR("%s,%02u%02u.%06lu,%c,%03u%02u.%06lu,%c,%u,%02u,%s,%d.%u,%c,%u.%u,%c,%02u,,"), gga.time, degrees_lat, minutes_lat, ssss, (lat >= 0) ? 'N' : 'S', degrees_lon, minutes_lon,
                   lon_ssss, (lon >= 0) ? 'E' : 'W', gga.fix_quality, gga.num_satellites, hdop_prec, antenna_int, antenna_dec, gga.altitude_units, undulation_int, undulation_dec, gga.undulation_units, (uint8_t)gga.dgps_age);
        static char nmea_line[NMEA_LINE_LENGTH];
        sprintf_P(nmea_line, PSTR("$MXGGA,%s*"), nmea_buf);
        const uint8_t cs = calcChecksum(nmea_line);
        sprintf_P(nmea_line, PSTR("$MXGGA,%s*%02X"), nmea_buf, cs);
        fifo_add(msgout_buf, nmea_line);
#ifdef HAS_I2C_LCD
        lcd.setCursor(10, 0);
        int8_t lat_deg = (int8_t)lat;
        int16_t lon_deg = (int16_t)lon;

        uint8_t lat_min = abs(lat - lat_deg) * 60;
        uint8_t lon_min = abs(lon - lon_deg) * 60;

        uint8_t lat_sec = (abs(lat - lat_deg) * 60 - lat_min) * 60;
        uint8_t lon_sec = (abs(lon - lon_deg) * 60 - lon_min) * 60;

        char lat_dir = lat >= 0 ? 'N' : 'S';
        char lon_dir = lon >= 0 ? 'E' : 'W';

        sprintf_P(g_buff, PSTR("%2d\xdf%02d\x01%02d\"%c"), abs(lat_deg), lat_min, lat_sec, lat_dir);
        lcd.print(g_buff);
        lcd.setCursor(9, 1);
        sprintf_P(g_buff, PSTR("%3d\xdf%02d\x01%02d\"%c"), abs(lon_deg), lon_min, lon_sec, lon_dir);
        lcd.print(g_buff);

        lcd.setCursor(0, 2);

        sprintf_P(g_buff, PSTR("%c%c:%c%c:%c%c"), gga.time[0], gga.time[1], gga.time[2], gga.time[3], gga.time[4], gga.time[5]);
        lcd.print(g_buff);

        if (uptime > 10) {  // Hide for 10 seconds to display startup info like IP address
            lcd.setCursor(0, 3);
            if (uptime - last_brd03_time > 360) {  // less than 6 minutes old
                lcd.setCursor(0, 3);
                lcd.print(F("DGPS data too old "));
            } else {
                const char* p = dgps_lookup(dgps_station_id);
                memset(g_buff, 0, 20);
                strcpy_P(g_buff, p);

                for (uint8_t i = strlen(g_buff); i < 18; i++) {
                    g_buff[i] = ' ';
                }

                lcd.print(g_buff);
            }
        }
#endif
#ifdef DEBUG
        char s_buf[16];
        UART0_Println(nmea_line);
        UART0_Print_P(PSTR("Spoof score: "));
        dtostrf(spoof_score, 5, 3, s_buf);
        UART0_Println(s_buf);
        /* dtostrf(lat - degrees_lat, 7, 4, s_buf);
        UART0_Println(s_buf); */

        /* sprintf(buf, "$MXGGA,%02u%02u%02u,%02u%02u.%06u,%c,%03u%02u.%06u,%c*",
                00, 00, 00,
                degrees_lat, minutes_lat, (uint16_t)((lat - degrees_lat - minutes_lat / 60.0) * 10000),
                (lat >= 0) ? 'N' : 'S',
                degrees_lon, minutes_lon, (uint16_t)((lon - degrees_lon - minutes_lon / 60.0) * 10000),
                (lon >= 0) ? 'E' : 'W');
        UART0_Println(buf)*/
        dtostrf(lat, 14, 7, s_buf);
        UART0_Print_P(PSTR("Current lat: "));
        UART0_Print(s_buf);
        dtostrf(lon, 14, 7, s_buf);
        UART0_Print_P(PSTR(" lon:"));
        UART0_Println(s_buf);
#endif
    }
#endif
}

/**
 * Parse a parameter line of the form "key=value" and update configuration accordingly.
 */
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

    char v[i - k + 1];
    v[i - k] = 0;
    strncpy(v, p + k + 1, i - k);

    for (uint8_t i = 0; i < 4; i++) {
        char k[] = "p1_bau";
        k[1] = i + 49;
        if (!memcmp(t, k, 6)) {
            p_config[i].baud = atol(v);
        }
    }
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_dir";
        k[1] = i + 49;
        if (!memcmp(t, k, 6)) {
            p_config[i].direction = v[0] - 48;
        }
    }
    for (uint8_t i = 0; i < 4; i++) {
        char k[] = "p1_par";
        k[1] = i + 49;
        if (!memcmp(t, k, 6)) {
            p_config[i].parameter = atoi(v);
        }
    }

    /**
     * Input Filter
     */
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_ifi";
        k[1] = i + 49;
        if (!memcmp(t, k, 6)) {
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
            for (; j < FILTER_SIZE; j++) {
                p_config[i].ifilter.pattern[j][0] = 0;
            }
        }
    }
    /**
     * Output Filter
     */
    for (uint8_t i = 0; i < PORTS; i++) {
        char k[] = "p1_ofi";
        k[1] = i + 49;
        if (!memcmp(t, k, 6)) {
            char* token;

            char* rest = v;
            uint8_t j = 0;

            /* for (uint8_t a = 0; a < FILTER_SIZE; a++) {
                char c = 0;

                for (uint8_t b = 0; b < 6; b++) {
                    if (j == 0) {
                        c = v[a * 7 + b];
                        if (c == 0) {
                            j = 1;
                        }

                        if (c == ':') {
                            break;
                        }
                        p_config[i].ofilter.pattern[a][b] = c;
                    }
                }
            } */

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
            for (; j < FILTER_SIZE; j++) {
                p_config[i].ofilter.pattern[j][0] = 0;
            }
        }
    }

    /**
     * MQTT
     */
    if (!memcmp(t, "mqtt_host", 9)) {
        strcpy(mqtt_broker, v);
    } else if (!memcmp(t, "mqtt_port", 9)) {
        mqtt_port = atoi(v);
    }

    /**
     * IP Address
     */
    IPAddress ip;
    if (!memcmp(t, "ip_add", 6)) {
        if (!(i - k)) {
            ip_address[0] = ip_address[1] = ip_address[2] = ip_address[3] = 0;
        } else {
            ip.fromString(v);
            ip_address[0] = ip[0];
            ip_address[1] = ip[1];
            ip_address[2] = ip[2];
            ip_address[3] = ip[3];
        }
    } else if (!memcmp(t, "hostname", 8)) {
        strcpy(hostname, v);
    } else if (!memcmp(t, "dns", 3)) {
        if (!(i - k)) {
            dns_a[0] = dns_a[1] = dns_a[2] = dns_a[3] = 0;
        } else {
            ip.fromString(v);
            dns_a[0] = ip[0];
            dns_a[1] = ip[1];
            dns_a[2] = ip[2];
            dns_a[3] = ip[3];
        }
    } else if (!memcmp(t, "gateway", 7)) {
        if (!(i - k)) {
            gateway[0] = gateway[1] = gateway[2] = gateway[3] = 0;
        } else {
            ip.fromString(v);
            gateway[0] = ip[0];
            gateway[1] = ip[1];
            gateway[2] = ip[2];
            gateway[3] = ip[3];
        }
    }

    else if (!memcmp(t, "subnet", 6)) {
        if (!(i - k)) {
            subnet[0] = subnet[1] = subnet[2] = subnet[3] = 0;
        } else {
            ip.fromString(v);
            subnet[0] = ip[0];
            subnet[1] = ip[1];
            subnet[2] = ip[2];
            subnet[3] = ip[3];
        }
    } else if (!memcmp(t, "p5_por", 6)) {
        port1 = atoi(v);
    } else if (!memcmp(t, "key", 3)) {
        key_up = atoi(v);
    } else if (!memcmp(t, "reset_settings", 14)) {
        reset_settings = atoi(v);
    } else if (!memcmp(t, "build", 5)) {
        build_available = atoi(v);
    } else if (!memcmp(t, "crc32", 5)) {
        crc_32 = hexStr2Int(v);
    } else if (!memcmp(t, "size", 4)) {
        filesize = atol(v);
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
void loadParamsFromEEPROM(void) {
    // We do have stored parameters, so load them
    uint16_t c = sizeof(uint32_t);  // skip to the next memory address
    for (uint8_t i = 0; i < PORTS; i++) {
        eeprom_read_block((void*)&p_config[i], (const uint16_t*)c, sizeof(p_config[i]));
        c += sizeof(p_config[i]);
#ifdef DEBUG
        for (uint8_t a = 0; a < FILTER_SIZE; a++) {
            sprintf_P(g_buff, txt8, i + 1, a + 1, p_config[i].ofilter.pattern[a], p_config[i].ofilter.pm_all);
            UART0_Println(g_buff);
        }
#endif
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
void saveParamsToEEPROM(void) {
    // We do have stored parameters, so load them
    uint16_t c = sizeof(uint32_t);  // skip to the next memory address

    for (uint8_t i = 0; i < PORTS; i++) {
#ifdef DEBUG
        for (uint8_t a = 0; a < FILTER_SIZE; a++) {
            sprintf_P(g_buff, txt8, i + 1, a + 1, p_config[i].ofilter.pattern[a], p_config[i].ofilter.pm_all);
            UART0_Println(g_buff);
        }
#endif
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

/**
 * Connect to the MQTT broker using the configured hostname and port
 */
void connectMQTT(char* broker, uint16_t port) {
    mqttClient.setId(hostname);
    mqttClient.setTimeout(5000);
    mqttClient.connect(broker, port);
}

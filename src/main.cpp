#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <SparkFun_SHTC3.h>
#include <CayenneLPP.h>

/** Definitions ***************************************************************/

#ifdef DEBUG
  #define DEBUG_PRINT(message) Serial.print(message)
  #define DEBUG_PRINTLN(message) Serial.println(message)
#else
  #define DEBUG_PRINT(message)
  #define DEBUG_PRINTLN(message)
#endif

#define INITIAL_SEND_DELAY 15000
#define DEFAULT_SEND_DELAY 60000

/** Declarations **************************************************************/

SHTC3 shtc3;
float shtc3_temperature;
float shtc3_humidity;

TimerEvent_t lorawan_timer;

uint8_t lorawan_device_eui[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t lorawan_application_eui[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t lorawan_application_key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void lorawan_rx_handler(lmh_app_data_t *app_data);
void lorawan_has_joined_handler(void);
void lorawan_confirm_class_handler(DeviceClass_t Class);
void lorawan_join_failed_handler(void);

static lmh_callback_t lorawan_callbacks = {
  BoardGetBatteryLevel,
  BoardGetUniqueId,
  BoardGetRandomSeed,
  lorawan_rx_handler,
  lorawan_has_joined_handler,
  lorawan_confirm_class_handler,
  lorawan_join_failed_handler
};

static lmh_param_t lorawan_param_init = {
  LORAWAN_ADR_ON,
  DR_0,                   // LORAWAN_DATARATE
  LORAWAN_PUBLIC_NETWORK,
  48,                     // JOINREQ_NBTRIALS
  TX_POWER_5,             // LORAWAN_TX_POWER
  LORAWAN_DUTYCYCLE_OFF
};

static uint8_t lorawan_app_data_buffer[64];
static lmh_app_data_t lorawan_app_data = {lorawan_app_data_buffer, 0, 0, 0, 0};

CayenneLPP lpp(64);

/** Setup *********************************************************************/

void setup_serial(void) {
#if DEBUG
  Serial.begin(115200);
  while (!Serial) delay(10);
#endif
}

void setup_leds(void) {
  DEBUG_PRINT(F("[  Setup] Setting up LEDs... "));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  DEBUG_PRINTLN(F("Done"));
}

void setup_shtc3(void) {
  DEBUG_PRINT(F("[  Setup] Setting up SHTC3 Sensor... "));

  Wire.begin();
  Wire.setClock(400000); // 400Mhz
  shtc3.begin();

  DEBUG_PRINTLN(F("Done"));
}

void lorawan_periodic_handler(void);

void setup_timer(void) {
  DEBUG_PRINT(F("[  Setup] Setting up periodic timer... "));

  TimerInit(&lorawan_timer, lorawan_periodic_handler);

  DEBUG_PRINTLN(F("Done"));
}

void lorawan_rx_handler(lmh_app_data_t *app_data) {
  DEBUG_PRINT(F("[LoRaWAN] Packet received on port "));
  DEBUG_PRINT(app_data->port);
  DEBUG_PRINT(F(", size: "));
  DEBUG_PRINT(app_data->buffsize);
  DEBUG_PRINT(F(", rssi: "));
  DEBUG_PRINT(app_data->rssi);
  DEBUG_PRINT(F(", snr: "));
  DEBUG_PRINT(app_data->snr);
  DEBUG_PRINT(F(", data: "));
  //DEBUG_PRINTLN(app_data->buffer);
  DEBUG_PRINTLN(F("TODO"));
}

void lorawan_has_joined_handler(void) {
  DEBUG_PRINTLN(F("[LoRaWAN] OTAA Mode, Network joined!"));

  lmh_error_status status = lmh_class_request(CLASS_A);
  if (status != LMH_SUCCESS) {
    DEBUG_PRINTLN(F("[LoRaWAN] Class request failed!"));
    while (1);
  }

  DEBUG_PRINTLN(F("[LoRaWAN] Class request successful!"));

  digitalWrite(LED_GREEN, HIGH);
  delay(1000);

  TimerSetValue(&lorawan_timer, INITIAL_SEND_DELAY);
  TimerStart(&lorawan_timer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class) {
  lorawan_app_data.buffsize = 0;
  lorawan_app_data.port = 15;

  lmh_send(&lorawan_app_data, LMH_UNCONFIRMED_MSG);
}

void lorawan_join_failed_handler(void) {
  DEBUG_PRINTLN(F("[LoRaWAN] OTAA join failed!"));
}

void setup_lorawan(void) {
  lora_rak4630_init();

  lmh_setDevEui(lorawan_device_eui);
  lmh_setAppEui(lorawan_application_eui);
  lmh_setAppKey(lorawan_application_key);

  uint32_t error = lmh_init(&lorawan_callbacks, lorawan_param_init, true);
  if (error != 0) {
    DEBUG_PRINTLN(F("[LoRaWAN] Initialization failed!"));
    while (1);
  } else {
    DEBUG_PRINTLN(F("[LoRaWAN] Initialization successful!"));
  }

  lmh_join();
}

void setup(void) {
  setup_serial();
  setup_leds();

  setup_shtc3(); // temperature & humidity

  setup_timer();
  setup_lorawan();
}

/** Runtime *******************************************************************/

void update_shtc3(void) {
  shtc3.update();

  shtc3_temperature = shtc3.toDegC();
  DEBUG_PRINT(F("[  SHTC3] Temperature: "));
  DEBUG_PRINT(shtc3_temperature);
  DEBUG_PRINTLN(F(" °C"));

  shtc3_humidity = shtc3.toPercent();
  DEBUG_PRINT(F("[  SHTC3] Humidity: "));
  DEBUG_PRINT(shtc3_humidity);
  DEBUG_PRINTLN(F(" %"));
}

void update_sensors(void) {
  DEBUG_PRINTLN(F("[ Update] Gathering sensor data"));

  update_shtc3();

  DEBUG_PRINTLN(F("[ Update] Done"));
}

void lorawan_periodic_handler(void) {
  digitalWrite(LED_BLUE, HIGH);

  update_sensors();

  TimerSetValue(&lorawan_timer, DEFAULT_SEND_DELAY);
  TimerStart(&lorawan_timer);

  if (lmh_join_status_get() == LMH_SET) {
    DEBUG_PRINT(F("[LoRaWAN] Building CayennLPP payload... "));

    lpp.reset();
    lpp.addTemperature(1, shtc3_temperature);
    lpp.addRelativeHumidity(2, shtc3_humidity);

    DEBUG_PRINTLN(F("Done"));

    memset(lorawan_app_data.buffer, 0, 64);
    lorawan_app_data.port = 15;
    lpp.copy(lorawan_app_data.buffer);
    lorawan_app_data.buffsize = lpp.getSize();

    lmh_error_status status = lmh_send(&lorawan_app_data, LMH_UNCONFIRMED_MSG);
    if (status == LMH_SUCCESS) {
      DEBUG_PRINTLN(F("[LoRaWAN] Sending message successful!"));
    } else {
      DEBUG_PRINTLN(F("[LoRaWAN] Sending message failed!"));
    }
  } else {
    DEBUG_PRINTLN(F("[LoRaWAN] Not connected to network. Not sending data!"));
  }

  digitalWrite(LED_BLUE, LOW);
}

void loop(void) {
  Radio.IrqProcess();
}

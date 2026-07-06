#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_netif.h"

// ======================================================================
// --- CONFIGURAÇÕES GERAIS — MESA 4 ---
// ======================================================================

// Pinos I2C
#define I2C_SDA_PIN                 GPIO_NUM_6
#define I2C_SCL_PIN                 GPIO_NUM_7
#define I2C_PORT                    I2C_NUM_0

// Endereço do multiplexador I2C e canal do sensor de luz da Mesa 4
#define TCA9548A_ADDR               0x70
#define CH_MESA4_BH1750             1

// Sensor de luz BH1750: endereço, comando e fatores de correção
#define BH1750_SENSOR_ADDR          0x23
#define BH1750_CMD_START            0x10
#define BH1750_CAL_FACTOR           1.12f   // Correção para o luxímetro de referência
//#define BH1750_POS_FACTOR_M4        1.35f   // Correção de posição do sensor na Mesa 4
#define BH1750_POS_FACTOR_M4        4.0f   // Correção de posição do sensor na Mesa 4

// ──────────────────────────────────────────────────────────────────────
// LD2420 — Mesa 4 (UART1 — neste ESP32-C6 dedicado à Mesa 4, o UART1
// está livre, já que não há outro sensor LD2420 disputando o periférico)
// ──────────────────────────────────────────────────────────────────────
#define LD2420_M4_UART_PORT         UART_NUM_1
#define LD2420_M4_PIN_TX            GPIO_NUM_5   // ESP TX → sensor RX
#define LD2420_M4_PIN_RX            GPIO_NUM_4   // ESP RX ← sensor OT1

#define LD2420_BAUD_RATE            115200       // use 256000 se fw < v1.5.3
#define LD2420_UART_BUF_SIZE        2048         // buffer grande — LD2420 envia ~10 frames/s continuamente

// Protocolo — frame de dados (modo normal)
#define LD2420_DATA_HEADER_0        0xAA
#define LD2420_DATA_HEADER_1        0xFF
#define LD2420_DATA_HEADER_2        0x03
#define LD2420_DATA_HEADER_3        0x00
#define LD2420_DATA_TAIL_0          0x55
#define LD2420_DATA_TAIL_1          0xCC
#define LD2420_DATA_FRAME_LEN       10   // header(4) + payload(4) + tail(2)

// Protocolo — frame de comando
#define LD2420_CMD_HEADER_0         0xFD
#define LD2420_CMD_HEADER_1         0xFC
#define LD2420_CMD_HEADER_2         0xFB
#define LD2420_CMD_HEADER_3         0xFA
#define LD2420_CMD_TAIL_0           0x04
#define LD2420_CMD_TAIL_1           0x03
#define LD2420_CMD_TAIL_2           0x02
#define LD2420_CMD_TAIL_3           0x01

#define LD2420_CMD_ENABLE_CFG       0x00FF
#define LD2420_CMD_DISABLE_CFG      0x00FE
#define LD2420_CMD_SET_MIN_GATE     0x0001
#define LD2420_CMD_SET_MAX_GATE     0x0002

// Gates de distância — geometria: sensor a 135 cm acima da mesa (75x160 cm)
// gate_min=1 descarta a superfície da mesa (reflexo muito próximo)
// gate_max=1 cobre cabeça (~70 cm) e cantos da mesa (~88 cm)
#define LD2420_GATE_MIN             1
#define LD2420_GATE_MAX             1

// Confirmações consecutivas necessárias para mudar o estado da mesa
#define LD2420_CONFIRM_OCUPADO      0
#define LD2420_CONFIRM_LIVRE        5

#define LD2420_OCUPADO              1
#define LD2420_LIVRE                0

// ──────────────────────────────────────────────────────────────────────
// Filtro de distância por software (modo Simple não tem gates binários)
// Geometria: sensor a 135 cm acima da mesa (75 x 160 cm).
//   - Cabeça de pessoa sentada: ~70 cm do sensor
//   - Canto mais distante da mesa: ~88 cm do sensor
// Margem de segurança aplicada para variação de postura.
// Qualquer "ON" com Range fora dessa faixa é tratado como fora da área
// de interesse (corredor, mesa vizinha, parede) e descartado.
// ──────────────────────────────────────────────────────────────────────
#define LD2420_RANGE_MIN_CM         110
#define LD2420_RANGE_MAX_CM         140

// Credenciais Wi-Fi e endereço do broker MQTT
#define WIFI_SSID       "PROJETO"
#define WIFI_PASS       "12345678"
#define MQTT_URI        "mqtt://192.168.0.101:1884"

// ======================================================================
// --- VARIÁVEIS GLOBAIS ---
// ======================================================================

static const char *TAG = "SENSORES_MESA4";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool wifi_connected = false;
static bool mqtt_recently_connected = false;

// ======================================================================
// --- MQTT ---
// ======================================================================

static void mqtt_reconnect_delay_task(void *pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    mqtt_recently_connected = false;
    ESP_LOGI(TAG, "MQTT pronto para publicação após reconexão.");
    vTaskDelete(NULL);
}

static void mqtt_publish(const char *topic, const char *msg)
{
    if (!mqtt_connected || !mqtt_client || mqtt_recently_connected) return;
    esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT → [%s] = %s", topic, msg);
}

static void mqtt_publish_float(const char *topic, float value)
{
    char payload[32];
    snprintf(payload, sizeof(payload), "%.2f", value);
    mqtt_publish(topic, payload);
}

static void mqtt_publish_int(const char *topic, int value)
{
    char payload[8];
    snprintf(payload, sizeof(payload), "%d", value);
    mqtt_publish(topic, payload);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            mqtt_recently_connected = true;
            ESP_LOGI(TAG, "✅ Conectado ao broker MQTT");
            esp_mqtt_client_publish(mqtt_client, "ambiente/status", "online", 0, 1, 1);
            xTaskCreate(mqtt_reconnect_delay_task, "mqtt_reconnect_delay", 2048, NULL, 5, NULL);
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            mqtt_recently_connected = false;
            ESP_LOGW(TAG, "⚠️  Desconectado do broker MQTT, tentando reconectar...");
            esp_mqtt_client_publish(mqtt_client, "ambiente/status", "offline", 0, 1, 1);
            esp_mqtt_client_reconnect(mqtt_client);
            break;

        default:
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .session.keepalive = 30,
        .session.disable_clean_session = false
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "Iniciando cliente MQTT...");
}

// ======================================================================
// --- WIFI ---
// ======================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi desconectado, tentando reconectar...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        ESP_LOGI(TAG, "✅ Conectado à rede Wi-Fi");
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ======================================================================
// --- MULTIPLEXADOR I2C (TCA9548A) ---
// ======================================================================

static esp_err_t tca9548a_select_channel(uint8_t channel)
{
    uint8_t cmd = 1 << channel;
    esp_err_t ret = i2c_master_write_to_device(I2C_PORT, TCA9548A_ADDR, &cmd, 1, pdMS_TO_TICKS(200));
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

// ======================================================================
// --- LD2420 ---
// ======================================================================

typedef struct {
    bool     ocupado;
    uint16_t distancia_cm;
    uint8_t  energia;
    int      falhas_consecutivas;
} ld2420_data_t;

typedef struct {
    int estado_atual;
    int cont_ocupado;
    int cont_livre;
} ld2420_debounce_t;

static void ld2420_send_command(uart_port_t uart_port, uint16_t cmd_id,
                                 const uint8_t *data, uint8_t data_len)
{
    uint16_t intra_len = 2 + data_len;
    uint8_t frame[64];
    int i = 0;

    frame[i++] = LD2420_CMD_HEADER_0;
    frame[i++] = LD2420_CMD_HEADER_1;
    frame[i++] = LD2420_CMD_HEADER_2;
    frame[i++] = LD2420_CMD_HEADER_3;

    frame[i++] = (uint8_t)(intra_len & 0xFF);
    frame[i++] = (uint8_t)(intra_len >> 8);

    frame[i++] = (uint8_t)(cmd_id & 0xFF);
    frame[i++] = (uint8_t)(cmd_id >> 8);

    for (int j = 0; j < data_len; j++) {
        frame[i++] = data[j];
    }

    frame[i++] = LD2420_CMD_TAIL_0;
    frame[i++] = LD2420_CMD_TAIL_1;
    frame[i++] = LD2420_CMD_TAIL_2;
    frame[i++] = LD2420_CMD_TAIL_3;

    uart_write_bytes(uart_port, (const char *)frame, i);
}

static void ld2420_enable_config_mode(uart_port_t uart_port)
{
    uint8_t payload[2] = {0x00, 0x01};
    ld2420_send_command(uart_port, LD2420_CMD_ENABLE_CFG, payload, 2);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void ld2420_disable_config_mode(uart_port_t uart_port)
{
    ld2420_send_command(uart_port, LD2420_CMD_DISABLE_CFG, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void ld2420_set_distance_gates(uart_port_t uart_port,
                                       uint8_t gate_min, uint8_t gate_max)
{
    ld2420_enable_config_mode(uart_port);

    uint8_t min_payload[2] = {gate_min, 0x00};
    ld2420_send_command(uart_port, LD2420_CMD_SET_MIN_GATE, min_payload, 2);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t max_payload[2] = {gate_max, 0x00};
    ld2420_send_command(uart_port, LD2420_CMD_SET_MAX_GATE, max_payload, 2);
    vTaskDelay(pdMS_TO_TICKS(20));

    ld2420_disable_config_mode(uart_port);
    ESP_LOGI(TAG, "LD2420 UART%d — gates configurados: min=%d max=%d",
             uart_port, gate_min, gate_max);
}

static bool ld2420_parse_frame(const uint8_t *buf, int len, ld2420_data_t *out)
{
    bool achou_estado = false;
    bool achou_dist   = false;
    bool ocupado_novo = out->ocupado;
    uint16_t distancia_novo = out->distancia_cm;

    int i = 0;
    bool primeira_linha = true;
    while (i < len) {
        int start = i;
        while (i < len && buf[i] != '\r' && buf[i] != '\n') i++;
        int line_len = i - start;
        bool linha_completa = (i < len);

        if (primeira_linha) {
            primeira_linha = false;
            while (i < len && (buf[i] == '\r' || buf[i] == '\n')) i++;
            continue;
        }

        if (line_len > 0 && linha_completa) {
            if (line_len >= 2 && buf[start] == 'O' && buf[start+1] == 'N') {
                ocupado_novo = true;
                achou_estado = true;
            } else if (line_len >= 3 && buf[start] == 'O' && buf[start+1] == 'F' && buf[start+2] == 'F') {
                ocupado_novo = false;
                achou_estado = true;
            } else if (line_len >= 6 &&
                       buf[start] == 'R' && buf[start+1] == 'a' && buf[start+2] == 'n' &&
                       buf[start+3] == 'g' && buf[start+4] == 'e' && buf[start+5] == ' ') {
                int val = 0;
                int j = start + 6;
                while (j < i && buf[j] >= '0' && buf[j] <= '9') {
                    val = val * 10 + (buf[j] - '0');
                    j++;
                }
                distancia_novo = (uint16_t)val;
                achou_dist = true;
            }
        }

        while (i < len && (buf[i] == '\r' || buf[i] == '\n')) i++;
    }

    bool achou_algo = achou_estado && achou_dist;

    if (achou_algo) {
        bool dentro_da_faixa = (distancia_novo >= LD2420_RANGE_MIN_CM &&
                                distancia_novo <= LD2420_RANGE_MAX_CM);

        out->ocupado      = ocupado_novo && dentro_da_faixa;
        out->distancia_cm = distancia_novo;
        out->energia      = 0;
    }
    return achou_algo;
}

#define LD2420_MAX_FALHAS_ANTES_DE_EXPIRAR   3

static bool ld2420_ler(uart_port_t uart_port, ld2420_data_t *out)
{
    uint8_t buf[LD2420_UART_BUF_SIZE];
    int len = uart_read_bytes(uart_port, buf, sizeof(buf), pdMS_TO_TICKS(110));

    if (len <= 0) {
        out->falhas_consecutivas++;
        if (out->falhas_consecutivas >= LD2420_MAX_FALHAS_ANTES_DE_EXPIRAR) {
            out->ocupado      = false;
            out->distancia_cm = 0;
        }
        return false;
    }

    char texto[LD2420_UART_BUF_SIZE + 1];
    int n = (len > LD2420_UART_BUF_SIZE) ? LD2420_UART_BUF_SIZE : len;
    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];
        texto[i] = (c == '\r' || c == '\n') ? ' ' : c;
    }
    texto[n] = '\0';
    ESP_LOGI(TAG, "LD2420 UART%d — %d bytes: \"%s\"", uart_port, len, texto);

    bool ok = ld2420_parse_frame(buf, len, out);
    if (ok) {
        out->falhas_consecutivas = 0;
    } else {
        out->falhas_consecutivas++;
        if (out->falhas_consecutivas >= LD2420_MAX_FALHAS_ANTES_DE_EXPIRAR) {
            out->ocupado      = false;
            out->distancia_cm = 0;
        }
    }
    return ok;
}

static int ld2420_debounce_atualizar(ld2420_debounce_t *db, int leitura_bruta)
{
    if (leitura_bruta == LD2420_OCUPADO) {
        db->cont_ocupado++;
        db->cont_livre = 0;
        if (db->cont_ocupado >= LD2420_CONFIRM_OCUPADO) {
            db->estado_atual = LD2420_OCUPADO;
            db->cont_ocupado = LD2420_CONFIRM_OCUPADO;
        }
    } else {
        db->cont_livre++;
        db->cont_ocupado = 0;
        if (db->cont_livre >= LD2420_CONFIRM_LIVRE) {
            db->estado_atual = LD2420_LIVRE;
            db->cont_livre = LD2420_CONFIRM_LIVRE;
        }
    }
    return db->estado_atual;
}

static void ld2420_init(const char *nome, uart_port_t uart_port, int pin_tx, int pin_rx)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = LD2420_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_is_driver_installed(uart_port)) {
        uart_driver_delete(uart_port);
    }

    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, pin_tx, pin_rx,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_port, LD2420_UART_BUF_SIZE * 2,
                                        0, 0, NULL, 0));

    ESP_LOGI(TAG, "LD2420 [%s] UART%d iniciado: TX=GPIO%d RX=GPIO%d @ %d baud",
             nome, uart_port, pin_tx, pin_rx, LD2420_BAUD_RATE);

    vTaskDelay(pdMS_TO_TICKS(500));
    // ld2420_set_distance_gates(uart_port, LD2420_GATE_MIN, LD2420_GATE_MAX);
    ESP_LOGI(TAG, "LD2420 [%s] UART%d — configuração de gates SKIPADA (modo debug)", nome, uart_port);
}

// ======================================================================
// --- FUNÇÃO PRINCIPAL — MESA 4 ---
// ======================================================================

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    mqtt_app_start();

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Inicializa sensor LD2420 Mesa 4 (UART1 completo)
    ld2420_init("Mesa4", LD2420_M4_UART_PORT, LD2420_M4_PIN_TX, LD2420_M4_PIN_RX);

    // Inicializa sensor de luz BH1750 da Mesa 4
    uint8_t bh_cmd = BH1750_CMD_START;
    tca9548a_select_channel(CH_MESA4_BH1750);
    i2c_master_write_to_device(I2C_PORT, BH1750_SENSOR_ADDR, &bh_cmd, 1, pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Inicialização concluída — Mesa 4.");

    ld2420_data_t leitura_m4 = { .ocupado = false, .distancia_cm = 0, .energia = 0, .falhas_consecutivas = 0 };
    ld2420_debounce_t db_m4 = { .estado_atual = LD2420_LIVRE, .cont_ocupado = 0, .cont_livre = 0 };

    while (1)
    {
        float lux4 = 0.0f;
        uint8_t data[2];

        // Leitura de luminosidade — Mesa 4
        // Fórmula: valor_bruto / 1.2 (conversão padrão BH1750) × fator de
        // calibração do luxímetro de referência × fator de posição da Mesa 4
        tca9548a_select_channel(CH_MESA4_BH1750);
        if (i2c_master_read_from_device(I2C_PORT, BH1750_SENSOR_ADDR, data, 2, pdMS_TO_TICKS(200)) == ESP_OK)
            lux4 = (((data[0] << 8) | data[1]) / 1.2f) * BH1750_CAL_FACTOR * BH1750_POS_FACTOR_M4;

        // Leitura de ocupância — Mesa 4 (LD2420 via UART1)
        ld2420_ler(LD2420_M4_UART_PORT, &leitura_m4);
        int bruto4 = leitura_m4.ocupado ? LD2420_OCUPADO : LD2420_LIVRE;
        int ocupado_mesa4 = ld2420_debounce_atualizar(&db_m4, bruto4);

        mqtt_publish_float("mesa4/lux",        lux4);
        mqtt_publish_float("mesa4/distancia",  (float)leitura_m4.distancia_cm);
        mqtt_publish_int  ("mesa4/ocupado",     ocupado_mesa4);

        ESP_LOGI(TAG, "Mesa4 → Lux: %.2f lx | Dist: %u cm | Energia: %u | Ocupada: %d",
                 lux4, leitura_m4.distancia_cm, leitura_m4.energia, ocupado_mesa4);
        ESP_LOGI(TAG, "------------------------------------------------------");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
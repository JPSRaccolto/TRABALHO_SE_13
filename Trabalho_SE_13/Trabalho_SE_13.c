// Transmissor LoRa com Sensores AHT20 e BMP280

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "lib/aht20.h"
#include "lib/bmp280.h"

// ========== CONFIGURAÇÕES DO PAYLOAD ==========
#define PAYLOAD_LENGTH      10  // Tamanho do payload em bytes

// ========== PROTOCOLO DE DADOS ==========
// Byte 0: Tipo de mensagem (0x01 = dados de sensores)
// Bytes 1-2: Temperatura (int16_t, x100)
// Bytes 3-4: Umidade (uint16_t, x100)
// Bytes 5-6: Pressão (uint16_t, offset 900hPa)
// Byte 7: Contador de mensagem
// Byte 8: Checksum
// Byte 9: Reservado

// ========== CONFIGURAÇÕES LORA ==========
#define LORA_FREQUENCY      915.0    // MHz
#define LORA_SF             7         // Spreading Factor
#define LORA_BW             125000    // Bandwidth em Hz
#define LORA_CR             1         // Coding Rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
#define LORA_PREAMBLE       8         // Tamanho do preâmbulo
#define LORA_CRC            0         // 0=OFF, 1=ON
#define LORA_LDR            0         // Low Data Rate Optimizer
#define LORA_HEADER         0         // 0=Implícito, 1=Explícito
#define LORA_TX_POWER       17        // dBm

// ========== CONFIGURAÇÕES DE DEBOUNCE ==========
#define DEBOUNCE_TIME_MS    250      // Tempo de debounce em milissegundos

// ========== PINOS LORA ==========
#define LORA_SPI_PORT       spi0
#define LORA_PIN_SCK        18
#define LORA_PIN_MOSI       19
#define LORA_PIN_MISO       16
#define LORA_PIN_CS         17
#define LORA_PIN_RESET      20
#define LORA_PIN_INT        8

// ========== PINOS I2C SENSORES ==========
#define I2C_PORT            i2c0
#define I2C_SDA             0
#define I2C_SCL             1

// ========== PINOS CONTROLE ==========
#define PIN_LED_RED         13
#define PIN_LED_GREEN       11
#define PIN_LED_BLUE        12
#define PIN_BUTTON_A        5
#define PIN_BUTTON_B        6

// ========== REGISTRADORES LORA ==========
#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_PA_RAMP         0x0A
#define REG_OCP             0x0B
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_IRQ_FLAGS_MASK  0x11
#define REG_IRQ_FLAGS       0x12
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_SYMB_TIMEOUT    0x1F
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LENGTH  0x22
#define REG_MODEM_CONFIG3   0x26
#define REG_DETECT_OPT      0x31
#define REG_DETECTION_THR   0x37
#define REG_SYNC_WORD       0x39
#define REG_DIO_MAPPING1    0x40
#define REG_VERSION         0x42

// Modos de operação LoRa
#define MODE_SLEEP          0x80
#define MODE_STDBY          0x81
#define MODE_TX             0x83
#define MODE_RX_CONTINUOUS  0x85

// ========== VARIÁVEIS GLOBAIS ==========
static uint8_t msg_counter = 0;

// Flags voláteis para interrupções
static volatile bool flag_button_a_pressed = false;
static volatile bool flag_button_b_pressed = false;
static volatile bool flag_send_manual = false;
static volatile bool flag_toggle_auto = false;
static volatile bool flag_read_sensors = false;
static volatile bool flag_auto_send = false;
static volatile bool flag_heartbeat = false;

// Variáveis de estado
static bool auto_mode = false;

// Timestamps para debounce
static volatile absolute_time_t last_button_a_time = {0};
static volatile absolute_time_t last_button_b_time = {0};

// Dados dos sensores
static AHT20_Data aht20_data;
static struct bmp280_calib_param bmp_params;
static float bmp_temperature = 0;
static float bmp_pressure = 0;

// Timers
static uint32_t last_auto_send = 0;
static uint32_t last_sensor_read = 0;
static uint32_t last_blink = 0;

// ========== HANDLER DE INTERRUPÇÃO GPIO ==========
void gpio_irq_handler(uint gpio, uint32_t events) {
    absolute_time_t now = get_absolute_time();
    
    if (events & GPIO_IRQ_EDGE_FALL) {  // Botão pressionado (edge falling)
        if (gpio == PIN_BUTTON_A) {
            // Debounce para botão A
            int64_t diff = absolute_time_diff_us(last_button_a_time, now);
            if (diff > (DEBOUNCE_TIME_MS * 1000)) {
                last_button_a_time = now;
                flag_button_a_pressed = true;
                flag_send_manual = true;
            }
        } 
        else if (gpio == PIN_BUTTON_B) {
            // Debounce para botão B
            int64_t diff = absolute_time_diff_us(last_button_b_time, now);
            if (diff > (DEBOUNCE_TIME_MS * 1000)) {
                last_button_b_time = now;
                flag_button_b_pressed = true;
                flag_toggle_auto = true;
            }
        }
    }
}

// ========== CALLBACK DO TIMER PARA TAREFAS PERIÓDICAS ==========
bool timer_callback(struct repeating_timer *t) {
    static uint32_t counter_2s = 0;
    static uint32_t counter_1s = 0;
    static uint32_t counter_heartbeat = 0;
    
    // Contador a cada 100ms
    counter_2s++;
    counter_1s++;
    counter_heartbeat++;
    
    // Flag para ler sensores a cada 2 segundos (20 * 100ms)
    if (counter_2s >= 20) {
        counter_2s = 0;
        flag_read_sensors = true;
    }
    
    // Flag para envio automático a cada 1 segundo (10 * 100ms)
    if (auto_mode && counter_1s >= 10) {
        counter_1s = 0;
        flag_auto_send = true;
    }
    
    // Flag para heartbeat a cada 1 segundo (10 * 100ms)
    if (counter_heartbeat >= 10) {
        counter_heartbeat = 0;
        flag_heartbeat = true;
    }
    
    return true;  // Continua repetindo
}

// ========== FUNÇÕES SPI LORA ==========
static void cs_select() {
    gpio_put(LORA_PIN_CS, 0);
    sleep_us(1);
}

static void cs_deselect() {
    sleep_us(1);
    gpio_put(LORA_PIN_CS, 1);
}

static uint8_t readRegister(uint8_t addr) {
    uint8_t buf[2];
    buf[0] = addr & 0x7F;
    
    cs_select();
    spi_write_blocking(LORA_SPI_PORT, &buf[0], 1);
    spi_read_blocking(LORA_SPI_PORT, 0, &buf[1], 1);
    cs_deselect();
    
    return buf[1];
}

static void writeRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | 0x80;
    buf[1] = data;
    
    cs_select();
    spi_write_blocking(LORA_SPI_PORT, buf, 2);
    cs_deselect();
    sleep_ms(1);
}

// ========== FUNÇÕES LORA ==========
static void setFrequency(double frequency) {
    unsigned long frequencyValue = (unsigned long)((frequency * 524288.0) / 32.0);
    
    printf("Frequência: %.3f MHz\n", frequency);
    
    writeRegister(REG_FRF_MSB, (frequencyValue >> 16) & 0xFF);
    writeRegister(REG_FRF_MID, (frequencyValue >> 8) & 0xFF);
    writeRegister(REG_FRF_LSB, frequencyValue & 0xFF);
}

static void resetModule() {
    gpio_put(LORA_PIN_RESET, 0);
    sleep_ms(10);
    gpio_put(LORA_PIN_RESET, 1);
    sleep_ms(50);
}

static void setMode(uint8_t mode) {
    writeRegister(REG_OPMODE, mode);
    sleep_ms(10);
}

static bool initLoRa() {
    printf("Iniciando LoRa...\n");
    
    // Reset do módulo
    resetModule();
    
    // Verifica comunicação
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12 && version != 0x11) {
        printf("Erro: Módulo LoRa não detectado! Versão: 0x%02X\n", version);
        return false;
    }
    
    // Modo LoRa
    setMode(MODE_SLEEP);
    writeRegister(REG_OPMODE, 0x80 | MODE_SLEEP);
    sleep_ms(10);
    
    // Configura frequência
    setFrequency(LORA_FREQUENCY);
    
    // Configurações do modem
    uint8_t bw_reg = 0x70;  // 125kHz
    uint8_t cr_reg = LORA_CR << 1;
    uint8_t implicit_reg = LORA_HEADER ? 0x00 : 0x01;
    
    writeRegister(REG_MODEM_CONFIG1, bw_reg | cr_reg | implicit_reg);
    
    uint8_t sf_reg = LORA_SF << 4;
    uint8_t crc_reg = LORA_CRC ? 0x04 : 0x00;
    writeRegister(REG_MODEM_CONFIG2, sf_reg | crc_reg);
    
    writeRegister(REG_MODEM_CONFIG3, LORA_LDR ? 0x08 : 0x00);
    
    // Preâmbulo
    writeRegister(REG_PREAMBLE_MSB, (LORA_PREAMBLE >> 8) & 0xFF);
    writeRegister(REG_PREAMBLE_LSB, LORA_PREAMBLE & 0xFF);
    
    // Payload length (modo implícito)
    writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
    
    // Detecção
    writeRegister(REG_DETECT_OPT, 0xC3);
    writeRegister(REG_DETECTION_THR, 0x0A);
    
    // Sync Word
    writeRegister(REG_SYNC_WORD, 0x12);
    
    // Configura PA
    if (LORA_TX_POWER == 17) {
        writeRegister(REG_PA_CONFIG, 0x8F);  // PA_BOOST, +17dBm
    } else if (LORA_TX_POWER <= 14) {
        writeRegister(REG_PA_CONFIG, 0x70 | LORA_TX_POWER);  // RFO
    }
    
    // OCP
    writeRegister(REG_OCP, 0x2B);
    
    // FIFO
    writeRegister(REG_FIFO_TX_BASE, 0x00);
    writeRegister(REG_FIFO_RX_BASE, 0x00);
    
    // Modo standby
    setMode(MODE_STDBY);
    
    printf("LoRa configurado com sucesso!\n");
    return true;
}

// ========== ENVIO DE DADOS ==========
static uint8_t calculate_checksum(uint8_t *data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void sendSensorData() {
    uint8_t buffer[PAYLOAD_LENGTH];
    memset(buffer, 0, PAYLOAD_LENGTH);
    
    // Formato do pacote:
    buffer[0] = 0x01;  // Tipo de mensagem: dados de sensores
    
    // Temperatura (multiplicada por 100 para preservar 2 casas decimais)
    int16_t temp_int = (int16_t)(aht20_data.temperature * 100);
    buffer[1] = (temp_int >> 8) & 0xFF;
    buffer[2] = temp_int & 0xFF;
    
    // Umidade (multiplicada por 100)
    uint16_t humid_int = (uint16_t)(aht20_data.humidity * 100);
    buffer[3] = (humid_int >> 8) & 0xFF;
    buffer[4] = humid_int & 0xFF;
    
    // Pressão (offset de 900 hPa para economizar bytes)
    uint16_t press_int = (uint16_t)((bmp_pressure - 900.0) * 10);
    buffer[5] = (press_int >> 8) & 0xFF;
    buffer[6] = press_int & 0xFF;
    
    // Contador de mensagem
    buffer[7] = msg_counter++;
    
    // Checksum
    buffer[8] = calculate_checksum(buffer, 8);
    
    // Byte reservado
    buffer[9] = 0x00;
    
    // Limpa flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    
    // Modo standby
    setMode(MODE_STDBY);
    
    // Reset FIFO
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    
    // Escreve dados no FIFO
    for (int i = 0; i < PAYLOAD_LENGTH; i++) {
        writeRegister(REG_FIFO, buffer[i]);
    }
    
    // Mostra dados enviados
    printf("TX: T=%.2f°C H=%.2f%% P=%.1fhPa Cnt=%d\n", 
           aht20_data.temperature, aht20_data.humidity, bmp_pressure, buffer[7]);
    
    // Muda para modo TX
    setMode(MODE_TX);
    
    // Aguarda transmissão
    uint32_t timeout = 0;
    while (!(readRegister(REG_IRQ_FLAGS) & 0x08) && timeout < 3000) {
        sleep_ms(1);
        timeout++;
    }
    
    // Limpa flag TxDone
    writeRegister(REG_IRQ_FLAGS, 0x08);
    
    // Volta para standby
    setMode(MODE_STDBY);
    
    // Feedback visual
    gpio_put(PIN_LED_GREEN, 1);
    sleep_ms(100);
    gpio_put(PIN_LED_GREEN, 0);
}

// ========== LEITURA DOS SENSORES ==========
static void read_sensors() {
    // Lê AHT20 (temperatura e umidade)
    if (!aht20_read(I2C_PORT, &aht20_data)) {
        printf("Erro ao ler AHT20\n");
        aht20_data.temperature = 0;
        aht20_data.humidity = 0;
    }
    
    // Lê BMP280 (pressão e temperatura)
    int32_t raw_temp, raw_pressure;
    bmp280_read_raw(I2C_PORT, &raw_temp, &raw_pressure);
    bmp_temperature = bmp280_convert_temp(raw_temp, &bmp_params) / 100.0;
    bmp_pressure = bmp280_convert_pressure(raw_pressure, raw_temp, &bmp_params) / 100.0;
    
    printf("Sensores: T=%.2f°C H=%.2f%% P=%.1fhPa\n", 
           aht20_data.temperature, aht20_data.humidity, bmp_pressure);
}

// ========== CONFIGURAÇÃO DAS INTERRUPÇÕES ==========
static void setup_interrupts() {
    // Configura interrupção para botão A (falling edge - quando pressionado)
    gpio_set_irq_enabled(PIN_BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
    
    // Configura interrupção para botão B (falling edge - quando pressionado)
    gpio_set_irq_enabled(PIN_BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
    
    // Define o callback único para todas as interrupções GPIO
    gpio_set_irq_callback(gpio_irq_handler);
    
    // Habilita as interrupções do banco GPIO 0
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    printf("Interrupções configuradas com debounce de %dms\n", DEBOUNCE_TIME_MS);
}

// ========== FUNÇÃO PRINCIPAL ==========
int main() {
    stdio_init_all();
    sleep_ms(4000);
    
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║  Transmissor LoRa com Sensores              ║\n");
    printf("║  Versão com Interrupções e Debounce         ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    // Configura GPIOs dos LEDs
    gpio_init(PIN_LED_RED);
    gpio_init(PIN_LED_GREEN);
    gpio_init(PIN_LED_BLUE);
    gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    
    // Configura GPIOs dos botões com pull-up
    gpio_init(PIN_BUTTON_A);
    gpio_init(PIN_BUTTON_B);
    gpio_set_dir(PIN_BUTTON_A, GPIO_IN);
    gpio_set_dir(PIN_BUTTON_B, GPIO_IN);
    gpio_pull_up(PIN_BUTTON_A);
    gpio_pull_up(PIN_BUTTON_B);
    
    // Configura GPIOs do LoRa
    gpio_init(LORA_PIN_CS);
    gpio_init(LORA_PIN_RESET);
    gpio_init(LORA_PIN_INT);
    gpio_set_dir(LORA_PIN_CS, GPIO_OUT);
    gpio_set_dir(LORA_PIN_RESET, GPIO_OUT);
    gpio_set_dir(LORA_PIN_INT, GPIO_IN);
    gpio_put(LORA_PIN_CS, 1);
    gpio_put(LORA_PIN_RESET, 1);
    
    // Inicializa SPI para LoRa
    spi_init(LORA_SPI_PORT, 1000000);
    gpio_set_function(LORA_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(LORA_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(LORA_PIN_MISO, GPIO_FUNC_SPI);
    
    // Inicializa I2C para sensores
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Inicializa sensores usando as bibliotecas existentes
    printf("Inicializando sensores...\n");
    
    // Reset e inicializa AHT20
    aht20_reset(I2C_PORT);
    sleep_ms(20);
    if (!aht20_init(I2C_PORT)) {
        printf("Erro ao inicializar AHT20!\n");
    }
    
    // Inicializa BMP280
    bmp280_init(I2C_PORT);
    bmp280_get_calib_params(I2C_PORT, &bmp_params);
    
    sleep_ms(100);
    
    // Teste inicial dos sensores
    read_sensors();
    
    // Inicializa LoRa
    if (!initLoRa()) {
        printf("Falha ao inicializar LoRa!\n");
        while (1) {
            gpio_put(PIN_LED_RED, 1);
            sleep_ms(500);
            gpio_put(PIN_LED_RED, 0);
            sleep_ms(500);
        }
    }
    
    // Configura as interrupções dos botões
    setup_interrupts();
    
    // Configura timer para tarefas periódicas (100ms)
    struct repeating_timer timer;
    add_repeating_timer_ms(100, timer_callback, NULL, &timer);
    
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║             SISTEMA PRONTO!                 ║\n");
    printf("╠════════════════════════════════════════════╣\n");
    printf("║ Botão A: Envio manual dos dados             ║\n");
    printf("║ Botão B: Alternar modo automático (1s)      ║\n");
    printf("║                                              ║\n");
    printf("║ LEDs:                                        ║\n");
    printf("║ - Vermelho: Heartbeat (pisca 1Hz)           ║\n");
    printf("║ - Verde: Transmissão LoRa                   ║\n");
    printf("║ - Azul: Modo automático ativo               ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    // Loop principal - processa as flags setadas pelas interrupções
    while (1) {
        // Processa flag de leitura dos sensores
        if (flag_read_sensors) {
            flag_read_sensors = false;
            read_sensors();
        }
        
        // Processa flag de envio manual (Botão A)
        if (flag_send_manual) {
            flag_send_manual = false;
            
            printf("\n>>> Envio manual via interrupção <<<\n");
            sendSensorData();
        }
        
        // Processa flag de toggle do modo automático (Botão B)
        if (flag_toggle_auto) {
            flag_toggle_auto = false;
            
            auto_mode = !auto_mode;
            printf("\n>>> Modo automático: %s <<<\n\n", auto_mode ? "ATIVADO" : "DESATIVADO");
            
            if (auto_mode) {
                gpio_put(PIN_LED_BLUE, 1);
            } else {
                gpio_put(PIN_LED_BLUE, 0);
            }
        }
        
        // Processa flag de envio automático
        if (flag_auto_send && auto_mode) {
            flag_auto_send = false;
            sendSensorData();
        }
        
        // Processa flag de heartbeat
        if (flag_heartbeat) {
            flag_heartbeat = false;
            gpio_put(PIN_LED_RED, !gpio_get(PIN_LED_RED));
        }
        
        // Sleep curto para economizar energia
        sleep_ms(1);
    }
    
    return 0;
}
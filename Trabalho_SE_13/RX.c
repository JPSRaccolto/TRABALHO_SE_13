// Receptor LoRa para Dados dos Sensores com Display OLED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"

// ========== CONFIGURAÇÕES DO DISPLAY OLED ==========
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define OLED_ADDRESS 0x3C

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
#define LORA_FREQUENCY      915.0    // MHz (DEVE SER IGUAL AO TX!)
#define LORA_SF             7         // Spreading Factor
#define LORA_BW             125000    // Bandwidth em Hz
#define LORA_CR             1         // Coding Rate
#define LORA_PREAMBLE       8         // Tamanho do preâmbulo
#define LORA_CRC            0         // 0=OFF, 1=ON
#define LORA_LDR            0         // Low Data Rate Optimizer
#define LORA_HEADER         0         // 0=Implícito, 1=Explícito

// ========== PINOS ==========
#define LORA_SPI_PORT       spi0
#define LORA_PIN_SCK        18
#define LORA_PIN_MOSI       19
#define LORA_PIN_MISO       16
#define LORA_PIN_CS         17
#define LORA_PIN_RESET      20
#define LORA_PIN_INT        8
#define PIN_LED_RED         13
#define PIN_LED_GREEN       11
#define PIN_LED_BLUE        12

// ========== REGISTRADORES ==========
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
#define REG_FIFO_RX_CURRENT 0x10
#define REG_IRQ_FLAGS_MASK  0x11
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_PKT_SNR_VALUE   0x19
#define REG_PKT_RSSI_VALUE  0x1A
#define REG_RSSI_VALUE      0x1B
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

// Modos de operação
#define MODE_SLEEP          0x80
#define MODE_STDBY          0x81
#define MODE_TX             0x83
#define MODE_RX_CONTINUOUS  0x85
#define MODE_RX_SINGLE      0x86

// IRQ Flags
#define IRQ_RX_DONE         0x40
#define IRQ_PAYLOAD_CRC_ERROR 0x20
#define IRQ_VALID_HEADER    0x10

// ========== ESTRUTURA DE DADOS RECEBIDOS ==========
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint8_t message_count;
    int16_t rssi;
    int8_t snr;
    bool valid;
} SensorData;

// ========== VARIÁVEIS GLOBAIS ==========
static SensorData last_data = {0};
static uint32_t packets_received = 0;
static uint32_t packets_error = 0;
static uint8_t last_msg_counter = 255;
static ssd1306_t display;  // Estrutura do display OLED
static bool display_initialized = false;
static uint32_t last_display_update = 0;

// ========== FUNÇÕES SPI ==========
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

// ========== FUNÇÕES DO DISPLAY OLED ==========
static bool initDisplay() {
    // Inicializa I2C
    i2c_init(I2C_PORT, 400 * 1000);  // 400kHz
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Inicializa o display
    ssd1306_init(&display, 128, 64, false, OLED_ADDRESS, I2C_PORT);
    ssd1306_config(&display);
    
    // Limpa o display
    ssd1306_fill(&display, false);
    ssd1306_send_data(&display);
    
    display_initialized = true;
    printf("Display OLED inicializado com sucesso!\n");
    return true;
}

static void updateDisplay(SensorData *data) {
    if (!display_initialized) return;
    
    char buffer[32];
    
    // Limpa o display
    ssd1306_fill(&display, false);
    
    // Desenha bordas
    ssd1306_rect(&display, 0, 0, 127, 63, true, false);
    
    // Título
    ssd1306_draw_string(&display, "RECEPTOR LoRa", 15, 2);
    ssd1306_line(&display, 0, 12, 127, 12, true);
    
    if (data->valid) {
        // Temperatura
        sprintf(buffer, "Temp: %.1fC", data->temperature);
        ssd1306_draw_string(&display, buffer, 5, 16);
        
        // Umidade
        sprintf(buffer, "Umid: %.1f%%", data->humidity);
        ssd1306_draw_string(&display, buffer, 5, 26);
        
        // Pressão
        sprintf(buffer, "Pres: %.0fhPa", data->pressure);
        ssd1306_draw_string(&display, buffer, 5, 36);
        
        // Linha separadora
        ssd1306_line(&display, 0, 46, 127, 46, true);
        
        // RSSI e contador
        sprintf(buffer, "RSSI:%ddBm #%d", data->rssi, data->message_count);
        ssd1306_draw_string(&display, buffer, 5, 50);
    } else {
        // Mensagem de aguardando dados
        ssd1306_draw_string(&display, "Aguardando", 30, 25);
        ssd1306_draw_string(&display, "dados...", 38, 35);
        
        // Contador de pacotes
        sprintf(buffer, "RX:%lu ERR:%lu", packets_received, packets_error);
        ssd1306_draw_string(&display, buffer, 15, 50);
    }
    
    // Envia dados para o display
    ssd1306_send_data(&display);
}

// ========== FUNÇÕES DE CONFIGURAÇÃO ==========
static void setFrequency(double frequency) {
    unsigned long frequencyValue = (unsigned long)((frequency * 524288.0) / 32.0);
    
    printf("Frequência RX: %.3f MHz\n", frequency);
    
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

// ========== CÁLCULO DO CHECKSUM ==========
static uint8_t calculate_checksum(uint8_t *data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// ========== DECODIFICAÇÃO DOS DADOS ==========
static bool decode_sensor_data(uint8_t *buffer, SensorData *data) {
    // Verifica tipo de mensagem
    if (buffer[0] != 0x01) {
        printf("Tipo de mensagem inválido: 0x%02X\n", buffer[0]);
        return false;
    }
    
    // Verifica checksum
    uint8_t checksum = calculate_checksum(buffer, 8);
    if (checksum != buffer[8]) {
        printf("Erro de checksum! Esperado: 0x%02X, Recebido: 0x%02X\n", checksum, buffer[8]);
        return false;
    }
    
    // Decodifica temperatura (int16_t, dividido por 100)
    int16_t temp_raw = (buffer[1] << 8) | buffer[2];
    data->temperature = temp_raw / 100.0;
    
    // Decodifica umidade (uint16_t, dividido por 100)
    uint16_t humid_raw = (buffer[3] << 8) | buffer[4];
    data->humidity = humid_raw / 100.0;
    
    // Decodifica pressão (uint16_t, offset de 900 hPa, dividido por 10)
    uint16_t press_raw = (buffer[5] << 8) | buffer[6];
    data->pressure = 900.0 + (press_raw / 10.0);
    
    // Contador de mensagem
    data->message_count = buffer[7];
    
    // Calcula RSSI e SNR
    int16_t rssi_raw = readRegister(REG_PKT_RSSI_VALUE);
    data->rssi = -157 + rssi_raw;  // Para frequências acima de 862MHz
    
    int8_t snr_raw = (int8_t)readRegister(REG_PKT_SNR_VALUE);
    data->snr = snr_raw / 4;
    
    data->valid = true;
    
    return true;
}

// ========== RECEPÇÃO DE MENSAGEM ==========
static bool receiveMessage(SensorData *data) {
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
    
    // Verifica se há dados recebidos
    if (irqFlags & IRQ_RX_DONE) {
        // Verifica erro de CRC
        if (irqFlags & IRQ_PAYLOAD_CRC_ERROR) {
            printf("Erro de CRC na recepção!\n");
            writeRegister(REG_IRQ_FLAGS, 0xFF);  // Limpa flags
            packets_error++;
            return false;
        }
        
        // Lê o buffer
        uint8_t buffer[PAYLOAD_LENGTH];
        uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT);
        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);
        
        for (int i = 0; i < PAYLOAD_LENGTH; i++) {
            buffer[i] = readRegister(REG_FIFO);
        }
        
        // Limpa flags de interrupção
        writeRegister(REG_IRQ_FLAGS, 0xFF);
        
        // Decodifica os dados
        if (decode_sensor_data(buffer, data)) {
            packets_received++;
            
            // Verifica perda de pacotes
            if (last_msg_counter != 255) {
                uint8_t expected = (last_msg_counter + 1) & 0xFF;
                if (data->message_count != expected) {
                    int lost = (data->message_count - expected) & 0xFF;
                    if (lost > 0 && lost < 100) {  // Evita falsos positivos
                        printf("*** %d pacote(s) perdido(s) ***\n", lost);
                    }
                }
            }
            last_msg_counter = data->message_count;
            
            return true;
        } else {
            packets_error++;
            return false;
        }
    }
    
    return false;
}

// ========== INICIALIZAÇÃO LORA ==========
static bool initLoRa() {
    printf("Iniciando receptor LoRa...\n");
    
    // Reset do módulo
    resetModule();
    
    // Verifica comunicação
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12 && version != 0x11) {
        printf("Erro: Módulo LoRa não detectado! Versão: 0x%02X\n", version);
        return false;
    }
    printf("Módulo LoRa detectado. Versão: 0x%02X\n", version);
    
    // Modo LoRa
    setMode(MODE_SLEEP);
    writeRegister(REG_OPMODE, 0x80 | MODE_SLEEP);
    sleep_ms(10);
    
    // Configura frequência
    setFrequency(LORA_FREQUENCY);
    
    // Configurações do modem (DEVEM SER IDÊNTICAS AO TX!)
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
    
    // Configurações de detecção
    writeRegister(REG_DETECT_OPT, 0xC3);
    writeRegister(REG_DETECTION_THR, 0x0A);
    
    // Sync Word (DEVE SER IGUAL AO TX!)
    writeRegister(REG_SYNC_WORD, 0x12);
    
    // LNA gain máximo
    writeRegister(REG_LNA, 0x20 | 0x03);
    
    // FIFO
    writeRegister(REG_FIFO_TX_BASE, 0x00);
    writeRegister(REG_FIFO_RX_BASE, 0x00);
    
    // Symbol timeout
    writeRegister(REG_SYMB_TIMEOUT, 0xFF);
    
    // Modo recepção contínua
    setMode(MODE_RX_CONTINUOUS);
    
    printf("Receptor LoRa configurado e aguardando dados...\n\n");
    return true;
}

// ========== EXIBIÇÃO DOS DADOS NO CONSOLE ==========
static void display_sensor_data_console(SensorData *data) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║         DADOS DOS SENSORES RECEBIDOS     ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Temperatura:    %6.2f °C              ║\n", data->temperature);
    printf("║ Umidade:        %6.2f %%               ║\n", data->humidity);
    printf("║ Pressão:        %6.1f hPa            ║\n", data->pressure);
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Mensagem #:     %3d                    ║\n", data->message_count);
    printf("║ RSSI:           %4d dBm               ║\n", data->rssi);
    printf("║ SNR:            %4d dB                ║\n", data->snr);
    printf("╠════════════════════════════════════════╣\n");
    printf("║ Pacotes OK:     %5lu                  ║\n", packets_received);
    printf("║ Pacotes Erro:   %5lu                  ║\n", packets_error);
    printf("║ Taxa de Erro:   %5.1f%%                 ║\n", 
           packets_received > 0 ? (100.0 * packets_error / (packets_received + packets_error)) : 0);
    printf("╚════════════════════════════════════════╝\n");
}

// ========== FUNÇÃO PRINCIPAL ==========
int main() {
    stdio_init_all();
    sleep_ms(4000);
    
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║    RECEPTOR LoRa - DADOS DOS SENSORES    ║\n");
    printf("║           COM DISPLAY OLED               ║\n");
    printf("╚════════════════════════════════════════╝\n\n");
    
    // Configura GPIOs dos LEDs
    gpio_init(PIN_LED_RED);
    gpio_init(PIN_LED_GREEN);
    gpio_init(PIN_LED_BLUE);
    gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    
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
    
    // Inicializa o display OLED
    if (!initDisplay()) {
        printf("Aviso: Display OLED não inicializado, continuando sem display...\n");
    }
    
    // Mostra tela inicial
    if (display_initialized) {
        ssd1306_fill(&display, false);
        ssd1306_rect(&display, 0, 0, 127, 63, true, false);
        ssd1306_draw_string(&display, "RECEPTOR LoRa", 25, 15);
        ssd1306_draw_string(&display, "Inicializando", 25, 30);
        ssd1306_draw_string(&display, "Sistema...", 35, 40);
        ssd1306_send_data(&display);
        sleep_ms(2000);
    }
    
    // Inicializa LoRa
    if (!initLoRa()) {
        printf("Falha ao inicializar LoRa!\n");
        
        // Mostra erro no display
        if (display_initialized) {
            ssd1306_fill(&display, false);
            ssd1306_draw_string(&display, "ERRO LoRa!", 30, 25);
            ssd1306_send_data(&display);
        }
        
        while (1) {
            gpio_put(PIN_LED_RED, 1);
            sleep_ms(500);
            gpio_put(PIN_LED_RED, 0);
            sleep_ms(500);
        }
    }
    
    // LED azul indica sistema pronto
    gpio_put(PIN_LED_BLUE, 1);
    
    // Atualiza display para modo de espera
    updateDisplay(&last_data);
    
    printf("Sistema pronto! Aguardando dados dos sensores...\n");
    printf("═══════════════════════════════════════════\n");
    
    // Loop principal
    uint32_t last_heartbeat = 0;
    uint32_t last_reception = 0;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Verifica recepção
        if (receiveMessage(&last_data)) {
            last_reception = now;
            
            // LED verde indica recepção
            gpio_put(PIN_LED_GREEN, 1);
            
            // Exibe dados recebidos no console
            display_sensor_data_console(&last_data);
            
            // Atualiza display OLED
            updateDisplay(&last_data);
            
            sleep_ms(100);
            gpio_put(PIN_LED_GREEN, 0);
        }
        
        // Atualiza display periodicamente mesmo sem novos dados (a cada 5 segundos)
        if (now - last_display_update > 5000) {
            last_display_update = now;
            updateDisplay(&last_data);
        }
        
        // Heartbeat LED (pisca azul)
        if (now - last_heartbeat > 1000) {
            last_heartbeat = now;
            gpio_put(PIN_LED_BLUE, !gpio_get(PIN_LED_BLUE));
        }
        
        // Timeout de recepção (avisa se não receber dados há muito tempo)
        if (last_reception > 0 && (now - last_reception > 10000)) {
            if ((now / 500) % 2) {  // Pisca vermelho
                gpio_put(PIN_LED_RED, 1);
            } else {
                gpio_put(PIN_LED_RED, 0);
            }
            
            // Atualiza display com indicação de timeout
            if (display_initialized && ((now / 2000) % 5 == 0)) {
                ssd1306_fill(&display, false);
                ssd1306_rect(&display, 0, 0, 127, 63, true, false);
                ssd1306_draw_string(&display, "RECEPTOR LoRa", 25, 2);
                ssd1306_line(&display, 0, 12, 127, 12, true);
                ssd1306_draw_string(&display, "TIMEOUT!", 40, 25);
                ssd1306_draw_string(&display, "Sem sinal...", 28, 35);
                char buffer[32];
                sprintf(buffer, "%ds sem dados", (now - last_reception) / 1000);
                ssd1306_draw_string(&display, buffer, 20, 50);
                ssd1306_send_data(&display);
            }
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
![image](https://github.com/user-attachments/assets/f2a5c9b8-6208-4723-8f46-1d74be421827)


# 📡 Sistema de Comunicação LoRa para Monitoramento de Sensores

## 📑 Sumário
- [🎯 Objetivos](#-objetivos)
- [📋 Descrição do Projeto](#-descrição-do-projeto)
- [⚙️ Funcionalidades Implementadas](#️-funcionalidades-implementadas)
- [🛠️ Requisitos do Projeto](#️-requisitos-do-projeto)
- [📂 Estrutura do Código](#-estrutura-do-código)
- [🖥️ Como Compilar](#️-como-compilar)
- [🧑‍💻 Autor](#-autor)
- [🤝 Contribuições](#-contribuições)
- [📽️ Demonstração em Vídeo](#️-demonstração-em-vídeo)
- [💡 Considerações Finais](#-considerações-finais)

## 🎯 Objetivos
- Desenvolver um sistema de comunicação sem fio de longo alcance (LoRa) entre duas placas Raspberry Pi Pico.
- Criar um nó transmissor capaz de ler dados ambientais (temperatura, umidade e pressão) dos sensores AHT20 e BMP280.
- Implementar um nó receptor que recebe os dados via LoRa, valida sua integridade e os exibe em tempo real em um display OLED e no console serial.
- Estabelecer um protocolo de comunicação customizado, incluindo um contador de pacotes e checksum para garantir a confiabilidade dos dados.
- Fornecer uma interface de usuário simples no transmissor, com botões para envio manual e para alternar um modo de transmissão automática.

## 📋 Descrição do Projeto
Este projeto consiste em um sistema completo de telemetria ambiental utilizando a tecnologia LoRa. O sistema é dividido em duas partes principais:

1.  **Transmissor (Estação de Sensores):** Baseado em uma Raspberry Pi Pico, este módulo lê continuamente os dados dos sensores AHT20 (temperatura e umidade) e BMP280 (pressão). Os dados são empacotados em um payload customizado e enviados via LoRa. O usuário pode optar por enviar dados manualmente ao pressionar um botão ou ativar um modo de envio automático em intervalos regulares. LEDs de status fornecem feedback visual sobre a operação.

2.  **Receptor (Estação de Monitoramento):** Também utilizando uma Raspberry Pi Pico, este módulo fica em modo de recepção contínua. Ao receber um pacote LoRa, ele verifica a integridade dos dados através de um checksum. Se o pacote for válido, os dados são decodificados e exibidos em um display OLED, juntamente com informações vitais da comunicação, como a força do sinal (RSSI) e um contador de pacotes. O sistema é ideal para aplicações de monitoramento remoto onde a fiação é impraticável.

## ⚙️ Funcionalidades Implementadas
1.  **Módulo Transmissor (`Trabalho_SE_13.c`):**
    * **Leitura de Sensores:** Aquisição de dados de temperatura/umidade (AHT20) e pressão (BMP280) via I2C.
    * **Payload Customizado:** Empacotamento dos dados em um buffer de 10 bytes, incluindo tipo de mensagem, valores dos sensores, contador e checksum.
    * **Interface de Usuário com Interrupções:**
        * Uso de interrupções de hardware (IRQ) para os botões, com tratamento de *debounce* para evitar leituras falsas.
        * **Botão A:** Envia um pacote de dados manualmente.
        * **Botão B:** Ativa/desativa o modo de transmissão automática (intervalo de 1 segundo).
    * **Feedback Visual:** LEDs indicam o status do sistema (Vermelho: heartbeat, Verde: transmissão, Azul: modo automático ativo).

2.  **Módulo Receptor (`RX.c`):**
    * **Recepção LoRa Contínua:** O módulo fica constantemente escutando por novos pacotes.
    * **Validação de Dados:** Cálculo e verificação do checksum do pacote recebido para garantir a integridade dos dados e descartar pacotes corrompidos.
    * **Exibição em Display OLED:**
        * Interface com um display OLED SSD1306 via I2C.
        * Exibição em tempo real dos dados de temperatura, umidade, pressão, RSSI do último pacote e contador de mensagens.
        * Telas de status para "Aguardando dados", "Timeout" e "Erro".
    * **Monitoramento e Estatísticas:** Saída detalhada no console serial, incluindo cálculo de taxa de erro e detecção de pacotes perdidos com base no contador de mensagens.

## 🛠️ Requisitos do Projeto
- **Hardware:**
    - Raspberry Pi Pico (ou outro RP2040) (x2)
    - Módulo LoRa (baseado em Semtech SX127x, ex: HopeRF RFM95/96) (x2)
    - **Para o Transmissor:**
        - Sensor AHT20 (Temperatura e Umidade)
        - Sensor BMP280 (Pressão)
        - Botões de pressão (Push Buttons) (x2)
    - **Para o Receptor:**
        - Display OLED SSD1306 128x64 (I2C)
    - LEDs, resistores, fios jumper e protoboards
- **Software/Bibliotecas:**
    - Raspberry Pi Pico SDK
    - Bibliotecas `hardware/i2c.h` e `hardware/spi.h`
    - Drivers/bibliotecas para os sensores AHT20, BMP280 e para o display SSD1306 (inclusos ou referenciados nos códigos)
    - Ferramentas de compilação (CMake, Make)
    - IDE (Visual Studio Code com extensão Pico-Go, ou similar)

## 📂 Estrutura do Código
```
Trabalho_SE_13/
├── Trabalho_SE_13.c      # Código principal do MÓDULO TRANSMISSOR
├── RX.c                  # Código principal do MÓDULO RECEPTOR
├── CMakeLists.txt        # Arquivo de configuração de compilação (build)
└── lib/                  # Diretório para bibliotecas e drivers de hardware
    ├── aht20.c
    ├── aht20.h           # Driver para o sensor de umidade/temperatura AHT20
    ├── bmp280.c
    ├── bmp280.h          # Driver para o sensor de pressão BMP280
    ├── ssd1306.c
    ├── ssd1306.h         # Driver para o display OLED SSD1306
    └── font.h            # Mapa de bits da fonte usada no display
```
## 🖥️ Como Compilar
O processo de compilação deve ser realizado separadamente para o transmissor e o receptor.

1.  **Clone o repositório:**
    ```bash
    git clone https://github.com/JPSRaccolto/TRABALHO_SE_13.git
    ```
2.  **Navegue até o diretório do projeto desejado (ex: `TRABALHO_SE_13`):**
    ```bash
    cd TRABALHO_SE_13
    ```
3.  **Compile o projeto com seu ambiente de desenvolvimento configurado para o RP2040.**
    * Se estiver usando CMake/Make a partir da linha de comando:
        ```bash
        mkdir build
        cd build
        cmake ..
        make
        ```
    * Isso gerará um arquivo `.uf2` na pasta `build` (ex: `Trabalho_SE_13.uf2`).

4.  **Carregue o código na placa Raspberry Pi Pico correspondente:**
    * Conecte a Pico ao computador enquanto segura o botão `BOOTSEL`.
    * Um novo volume de armazenamento aparecerá.
    * Arraste o arquivo `.uf2` gerado para este volume. A placa será reiniciada com o novo firmware.

5.  **Repita os passos 2 a 4 para o outro módulo (receptor).**

## 🧑‍💻 Autores
```
Jadson de Jesus Santos,
João Pedro Soares Raccolto,
Mariana Fariasda Silva,
Matheus Pereira Alves,
Rian de Sena Mendes,
Samuel Guedes Canário
```

## 🤝 Contribuições
Este projeto foi desenvolvido por **Jadson de Jesus Santos, João Pedro Soares Raccolto, Mariana Fariasda Silva, Matheus Pereira Alves, Rian de Sena Mendes, Samuel Guedes Canário**.
Contribuições são bem-vindas! Se você deseja contribuir, por favor, siga os passos abaixo:

1.  Faça um Fork deste repositório.
2.  Crie uma nova branch para sua feature:
    ```bash
    git checkout -b minha-nova-feature
    ```
3.  Faça suas modificações e commit:
    ```bash
    git commit -m 'Adiciona minha nova feature'
    ```
4.  Envie suas alterações para o seu Fork:
    ```bash
    git push origin minha-nova-feature
    ```
5.  Abra um Pull Request.

## 📽️ Demonstração em Vídeo
- O vídeo de demonstração do projeto pode ser visualizado aqui: [LINK](Inserir link aqui)

## 💡 Considerações Finais
Este projeto serve como uma base robusta para a criação de redes de sensores sem fio de baixo custo e longo alcance. A combinação de interrupções de hardware, um protocolo de dados confiável e feedback em tempo real o torna uma ferramenta versátil para P&D, automação residencial e prototipagem de soluções de IoT.

Melhorias futuras poderiam incluir:
- Implementação de modos de baixo consumo de energia (*deep sleep*) no transmissor para operação com bateria.
- Adição de criptografia AES ao payload para uma comunicação segura.
- Expansão para uma rede com múltiplos sensores e um único receptor (gateway).
- Criação de um dashboard web para visualização dos dados recebidos.

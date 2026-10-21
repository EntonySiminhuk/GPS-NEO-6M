# GPS NEO-6M com Heltec WiFi LoRa 32 V3

Firmware em **ESP-IDF** que lê um módulo GPS **u-blox NEO-6M** pela UART, interpreta as
sentenças NMEA e mostra **data/hora, latitude, longitude, altitude e número de satélites**
no monitor serial e no display OLED da placa.

```
I (747270) GPS: 03/10/2026 16:10:53.4 | Lat: -26.229174 | Lon: -52.672226 | Alt: 727.9 m
```

```
┌─────────────────────┐
│03/10/2026 16:10:53.4│
│─────────────────────│
│Lat: -26.229174      │
│Lon: -52.672226      │
│Alt: 727.9 m         │
│                     │
│Satelites: 5         │
└─────────────────────┘
```

## Hardware

| Item | Detalhe |
|---|---|
| Placa | Heltec WiFi LoRa 32 **V3** (ESP32-S3 + SX1262) |
| GPS | u-blox NEO-6M (UART, 9600 baud, NMEA) |
| Display | OLED SSD1306 128x64 da própria placa (I2C) |

### Ligações

| NEO-6M | Heltec V3 | Observação |
|---|---|---|
| TX | **GPIO4** | RX da UART1 do ESP32 |
| RX | **GPIO5** | TX da UART1 do ESP32 (necessário para configurar 38400 baud / 5 Hz) |
| VCC | 3V3 ou Vext | Vext é ligado pelo firmware (GPIO36 em LOW) |
| GND | GND | GND precisa ser comum |

Pinos internos da placa usados pelo firmware:

| Função | GPIO |
|---|---|
| Vext (alimentação do OLED / pino Vext) | 36 (LOW = ligado) |
| OLED SDA | 17 |
| OLED SCL | 18 |
| OLED RST | 21 |

## Como funciona

```
NEO-6M --UART1 9600--> gps_task --> nmea_parse --> gps_data_t --> gps_print --> monitor serial
                                                                          \--> display OLED
```

1. **Inicialização (`app_main`)**
   - Liga o **Vext** (GPIO36 em LOW), que alimenta o OLED e o pino Vext.
   - Inicializa o **display OLED** (`oled_init`). Se falhar, registra o erro e segue sem display.
   - Configura a **UART1** em 9600 baud, 8N1, nos pinos 4 (RX) e 5 (TX).
   - **Configura o GPS (`gps_config`)** com comandos binários **UBX** da u-blox:
     - `CFG-PRT`: muda o GPS para **38400 baud**. O comando é enviado em 9600 e em 38400,
       porque o GPS pode já estar configurado se só o ESP32 reiniciou.
     - `CFG-MSG`: desliga as sentenças GLL, GSA, GSV e VTG (só RMC e GGA são usadas).
     - `CFG-RATE`: define a taxa de atualização de **5 Hz** (uma medição a cada 200 ms).
     - Cada comando espera a confirmação (`ACK-ACK`). Se o GPS não responder, por exemplo
       com o fio GPIO5 → RX solto, o firmware volta para 9600 baud / 1 Hz e mostra um aviso.
     - A configuração fica na RAM do GPS e se perde quando ele é desligado, por isso é
       refeita a cada boot.
   - Cria a tarefa `gps_task`.

2. **Leitura (`gps_task`)**
   - Lê bytes da UART e monta uma linha por vez. Um `$` marca o início e `\r`/`\n` o fim.
   - Linhas maiores que o buffer de 128 bytes são descartadas.
   - Os buffers são `static` para não ocupar a pilha da tarefa.

3. **Interpretação NMEA (`nmea_parse`)**
   - Confere o **checksum** (`*XX`) e descarta sentenças corrompidas.
   - Aceita os prefixos `$GP` e `$GN`.
   - **`RMC`**: status (`A` = posição válida), latitude, longitude, hora e data.
   - **`GGA`**: número de satélites e altitude. Como a `GGA` chega depois da `RMC` em cada
     ciclo, a impressão acontece aqui, 5 vezes por segundo.
   - Guarda também a fração do segundo (`ms`), exibida como décimos (`16:10:53.4`).
   - Converte as coordenadas de `ggmm.mmmm` para **graus decimais** (sul e oeste ficam negativos).
   - Converte hora e data para **Unix timestamp** (UTC).

4. **Saída (`gps_print`)**
   - Converte o timestamp para o horário local usando `FUSO_HORARIO`.
   - Sem posição válida, mostra `Aguardando posicao...` e o número de satélites.
   - Com posição, mostra data/hora, latitude, longitude e altitude no terminal e no OLED.

Os dados mais recentes ficam na variável global `gps` (`gps_data_t`):

| Campo | Tipo | Conteúdo |
|---|---|---|
| `valido` | `bool` | `true` quando há posição (fix) |
| `lat`, `lon` | `double` | graus decimais |
| `alt` | `float` | metros acima do nível do mar |
| `sats` | `int` | satélites usados no cálculo |
| `timestamp` | `time_t` | Unix time em UTC |
| `ms` | `int` | milissegundos dentro do segundo (0, 200, 400, 600, 800) |

## Transmissão LoRa

O mesmo firmware funciona em dois modos, escolhidos por `MODO_RECEPTOR` em [main/main.c](main/main.c):

```
 Placa 1: MODO_RECEPTOR 0                         Placa 2: MODO_RECEPTOR 1
 GPS -> gps_task -> gps_ultimo -> lora_tx_task ~~~ 915 MHz ~~~> lora_rx_task -> terminal + OLED
                                  (a cada 5 s)                  (RSSI, SNR, perdidos)
```

- **Transmissor:** a `lora_tx_task` acorda a cada `LORA_INTERVALO_MS`, copia a última leitura
  completa (`gps_ultimo`, protegida por uma seção crítica do FreeRTOS), monta um `pacote_t` e envia.
  O display mostra `LoRa: #N enviado` na última linha.
- **Receptor:** a `lora_rx_task` deixa o rádio em recepção contínua, valida o pacote (tamanho e versão),
  decodifica e mostra a posição, o RSSI, o SNR e os pacotes perdidos (saltos no contador).

### Formato do pacote (21 bytes, binário, little-endian)

| Bytes | Campo | Tipo | Conteúdo |
|---|---|---|---|
| 0 | `versao` | `uint8` | formato do pacote (1) |
| 1 | `flags` | `uint8` | bit 0 = posição válida |
| 2–3 | `contador` | `uint16` | número sequencial |
| 4–7 | `timestamp` | `uint32` | Unix time UTC |
| 8–11 | `lat` | `int32` | graus × 10⁷ |
| 12–15 | `lon` | `int32` | graus × 10⁷ |
| 16–19 | `alt` | `int32` | decímetros |
| 20 | `sats` | `uint8` | satélites |

### Rádio (SX1262)

O driver em [main/lora.c](main/lora.c) conversa com o SX1262 por SPI e configura:
TCXO de 1,8 V pelo DIO3, chave de antena pelo DIO2, regulador DC-DC, modo LoRa,
915 MHz, SF9, 125 kHz, CR 4/5, CRC ligado e cabeçalho explícito.
Com esses parâmetros, cada pacote ocupa o ar por **~185 ms**.

Pinos internos usados: NSS 8, SCK 9, MOSI 10, MISO 11, RST 12, BUSY 13, DIO1 14.

> **Nunca transmita sem a antena conectada**: a potência refletida pode danificar o SX1262.

## Estrutura do projeto

| Arquivo | Conteúdo |
|---|---|
| [main/main.c](main/main.c) | UART, configuração UBX, parser NMEA, saída, tarefas LoRa TX/RX e `app_main` |
| [main/oled.c](main/oled.c) / [main/oled.h](main/oled.h) | Driver do SSD1306 por I2C com fonte 5x7 (21 colunas x 8 linhas) |
| [main/lora.c](main/lora.c) / [main/lora.h](main/lora.h) | Driver do SX1262 por SPI e parâmetros do rádio |
| [main/CMakeLists.txt](main/CMakeLists.txt) | Componentes usados (UART, GPIO, I2C, SPI) e avisos tratados como erro (`-Wall -Wextra -Werror`) |
| [CMakeLists.txt](CMakeLists.txt) | Projeto com `MINIMAL_BUILD`: compila só os componentes necessários |
| [.clangd](.clangd) | Configuração do analisador do editor (veja abaixo) |

## Configuração

No topo de [main/main.c](main/main.c):

| Define | Padrão | Função |
|---|---|---|
| `GPS_RX_PIN` / `GPS_TX_PIN` | 4 / 5 | Pinos da UART do GPS |
| `GPS_BAUD_PADRAO` | 9600 | Velocidade de fábrica do NEO-6M |
| `GPS_BAUD` | 38400 | Velocidade usada depois da configuração |
| `GPS_TAXA_HZ` | 5 | Posições por segundo (1 a 5 no NEO-6M) |
| `FUSO_HORARIO` | -3 | Horas em relação ao UTC (Brasília) |
| `MOSTRAR_NMEA` | 0 | `1` imprime também as sentenças NMEA brutas |
| `MODO_RECEPTOR` | 0 | `0` = transmissor com GPS, `1` = receptor LoRa |
| `LORA_INTERVALO_MS` | 5000 | Intervalo entre transmissões |

Em [main/lora.h](main/lora.h) (precisam ser **iguais** no transmissor e no receptor):

| Define | Padrão | Função |
|---|---|---|
| `LORA_FREQ_HZ` | 915000000 | Frequência (Brasil: 902–907,5 e 915–928 MHz) |
| `LORA_SF` | 9 | Spreading factor 7–12: maior = mais alcance e mais tempo no ar |
| `LORA_BW` | 0x04 (125 kHz) | Largura de banda |
| `LORA_CR` | 0x01 (4/5) | Coding rate |
| `LORA_TX_DBM` | 14 | Potência de transmissão (−9 a 22 dBm) |

Para mudar **o que aparece no terminal ou no display**, edite a função `gps_print()`.
No display, use `oled_printf(coluna, linha, ...)` (coluna 0–20, linha 0–7) entre
`oled_clear()` e `oled_update()`. A fonte só tem ASCII, sem acentos.

## Compilar e gravar

Requer **ESP-IDF v6.1**, com o alvo `esp32s3` já definido no `sdkconfig`.

Pela IDE da Espressif, use **Build, Flash and Monitor**. Pelo terminal do ESP-IDF
(troque `COM10` pela porta da placa):

```bash
idf.py -p COM10 flash monitor
```

Para sair do monitor, use `Ctrl + ]`.

## Problemas comuns

| Sintoma | Causa / solução |
|---|---|
| Só aparece `Aguardando posicao... (satelites: 0)` | O GPS ainda não tem sinal. Use a antena com o céu aberto e espere de 1 a 5 min na primeira vez. O LED do NEO-6M pisca quando há posição. |
| Nenhuma linha do GPS | Confira se o TX do GPS está no GPIO4, o GND comum e a alimentação. Use `MOSTRAR_NMEA 1` para ver os dados brutos. |
| `GPS nao confirmou a configuracao` | O GPS não recebeu os comandos UBX. Confira o fio GPIO5 → RX do GPS. O firmware continua funcionando a 9600 baud / 1 Hz. |
| Posição "passeando" dezenas de metros parada | Sinal fraco ou poucos satélites (comum em ambiente fechado). Ao ar livre o erro típico é de 2 a 5 m. |
| `Could not open COM.. Acesso negado` | A porta está aberta por outro programa (monitor serial, Arduino IDE etc.). Feche-o e grave de novo. |
| `ClearCommError failed` no monitor | O USB desconectou (cabo, mau contato ou queda de alimentação). Reconecte e reabra o monitor. |
| `Falha ao iniciar o display` | O I2C não encontrou o OLED. Confira se a placa é a V3 (pinos 17/18/21). |
| `Falha ao iniciar o radio: ESP_ERR_NOT_FOUND` | O SX1262 não respondeu pelo SPI. Confira se a placa é a V3. |
| Receptor não recebe nada | Os parâmetros de `lora.h` precisam ser iguais nas duas placas, e as duas precisam de antena. Teste primeiro com as placas a poucos metros. |
| `Pacote corrompido (erro de CRC)` frequente | Sinal no limite do alcance ou interferência. Aumente `LORA_SF` ou `LORA_TX_DBM`, ou melhore a posição das antenas. |
| Aviso de Unicode do `idf.py` | Só afeta o terminal do Windows. Ative "UTF-8 Beta" nas configurações regionais ou defina `PYTHONUTF8=1`. |
| Editor acusa `stddef.h file not found` | O pacote `esp-clangd` vem sem os headers internos. O [.clangd](.clangd) aponta `-resource-dir` para o `esp-clang`; ajuste a versão no caminho se atualizar as ferramentas. |

## Próximos passos

- Gravar os pontos em CSV/GPX para visualizar o trajeto em um mapa.
- Filtrar leituras ruins (poucos satélites ou HDOP alto).
- No receptor, repassar os pacotes por Wi-Fi para um mapa em tempo real (Traccar, Node-RED etc.).

## Licença

Domínio público ou CC0, conforme o arquivo [LICENSE](LICENSE).

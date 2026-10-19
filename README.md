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

## Estrutura do projeto

| Arquivo | Conteúdo |
|---|---|
| [main/main.c](main/main.c) | UART, parser NMEA, formatação da saída e `app_main` |
| [main/oled.c](main/oled.c) / [main/oled.h](main/oled.h) | Driver do SSD1306 por I2C com fonte 5x7 (21 colunas x 8 linhas) |
| [main/CMakeLists.txt](main/CMakeLists.txt) | Componentes usados (UART, GPIO, I2C) e avisos tratados como erro (`-Wall -Wextra -Werror`) |
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
| Aviso de Unicode do `idf.py` | Só afeta o terminal do Windows. Ative "UTF-8 Beta" nas configurações regionais ou defina `PYTHONUTF8=1`. |
| Editor acusa `stddef.h file not found` | O pacote `esp-clangd` vem sem os headers internos. O [.clangd](.clangd) aponta `-resource-dir` para o `esp-clang`; ajuste a versão no caminho se atualizar as ferramentas. |

## Próximos passos

- Gravar os pontos em CSV/GPX para visualizar o trajeto em um mapa.
- Filtrar leituras ruins (poucos satélites ou HDOP alto).
- Enviar a posição pelo **LoRa** (915 MHz) em um pacote binário compacto para uma segunda placa
  e mostrar em tempo real (Traccar, Node-RED etc.).

## Licença

Domínio público ou CC0, conforme o arquivo [LICENSE](LICENSE).

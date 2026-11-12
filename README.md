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
NEO-6M --UART1 38400--> gps_task --> nmea_parse --> gps_leitura_t --> gps_validar --> gps_print --> monitor serial
                                                                                           \--> display OLED
```

1. **Inicialização (`app_main`)**
   - Liga o **Vext** (GPIO36 em LOW), que alimenta o OLED e o pino Vext.
   - Inicializa o **display OLED** (`oled_init`). Se falhar, registra o erro e segue sem display.
   - Configura a **UART1** em 9600 baud, 8N1, nos pinos 4 (RX) e 5 (TX).
   - **Configura o GPS (`gps_config`)** com comandos binários **UBX** da u-blox:
     - `CFG-PRT`: muda o GPS para **38400 baud**. O comando é enviado em 9600 e em 38400,
       porque o GPS pode já estar configurado se só o ESP32 reiniciou.
     - `CFG-MSG`: liga **GSA** (tipo de fix e DOP) e **GST** (erro estimado em metros) e
       desliga GLL, GSV e VTG. RMC e GGA já vêm ligadas de fábrica.
     - `CFG-NAV5`: modelo dinâmico **aéreo < 1g** (`GPS_MODELO_DINAMICO`) e **fix somente 3D**.
       Com 3 satélites o NEO-6M faria um fix 2D, supondo a altitude, e a posição poderia
       errar centenas de metros.
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
   - **`RMC`**: status (`A` = posição válida), latitude, longitude, velocidade, curso, hora e data.
   - **`GGA`**: qualidade do fix, número de satélites, HDOP e altitude.
   - **`GSA`**: tipo de fix (1 = sem fix, 2 = 2D, 3 = 3D), PDOP e VDOP.
   - **`GST`**: desvio-padrão estimado do erro de latitude, longitude e altitude, em metros.
   - As sentenças do mesmo instante formam uma **época** (`gps_leitura_t`). A época é
     publicada assim que chegam todas as esperadas, ou quando a próxima começa (mudança de
     hora ou tipo repetido). Assim uma sentença perdida não trava a saída. Campos não
     recebidos ficam negativos (desconhecidos).
   - Guarda também a fração do segundo (`ms`), exibida como décimos (`16:10:53.4`).
   - Converte as coordenadas de `ggmm.mmmm` para **graus decimais** (sul e oeste ficam negativos).
   - Converte hora e data para **Unix timestamp** (UTC).

4. **Validação da qualidade (`gps_validar`, em [main/gps_filtro.c](main/gps_filtro.c))**
   - Cada época passa pela camada 1 do filtro e só é **aceita** se todas as condições valerem:

     | Critério | Limite padrão | Motivo da rejeição |
     |---|---|---|
     | RMC `A` e GGA qualidade 1 ou 2 | — | `sem fix` |
     | Fix 3D (GSA) | — | `fix 2D` |
     | Satélites | ≥ `GPS_MIN_SATS` (5) | `poucos sats` |
     | HDOP | ≤ `GPS_MAX_HDOP` (2,5) | `HDOP alto` |
     | Erro horizontal da GST | ≤ `GPS_MAX_ERRO_M` (10 m) | `erro alto` |
     | Leituras boas seguidas | ≥ `GPS_LEITURAS_ESTAVEIS` (5) | `estabiliz.` |

   - Sem GSA ou sem GST (alguns clones não enviam), o critério correspondente é ignorado.
   - Os limites são pontos de partida experimentais: passar neles não garante precisão.
   - A época aceita vai para `gps_aceito`. A última época, aceita ou não, vai para `gps_ultimo`.

5. **Saída (`gps_print`)**
   - Converte o timestamp para o horário local usando `FUSO_HORARIO`.
   - Sem fix, mostra `Aguardando posicao...` e o número de satélites.
   - Com posição aceita, mostra data/hora, latitude, longitude, altitude, HDOP e erro estimado.
   - Com posição rejeitada, mostra o motivo (`Posicao rejeitada: fix 2D`, por exemplo).
   - A linha 5 do display mostra sempre `3D HDOP:1.2 e:3m` (tipo de fix, HDOP, erro estimado).

Cada época é guardada em um `gps_leitura_t` ([main/gps_filtro.h](main/gps_filtro.h)):

| Campo | Tipo | Origem | Conteúdo |
|---|---|---|---|
| `rmc_ativo` | `bool` | RMC | `true` quando o status é `A` |
| `lat`, `lon` | `double` | RMC | graus decimais |
| `vel_ms`, `curso` | `float` | RMC | velocidade sobre o solo (m/s) e direção (graus) |
| `timestamp`, `ms` | `time_t`, `int` | RMC | Unix time em UTC e milissegundos (0, 200, 400...) |
| `qualidade` | `int` | GGA | 0 sem fix, 1 GPS, 2 DGPS, 6 estimado |
| `sats` | `int` | GGA | satélites usados no cálculo |
| `alt` | `float` | GGA | metros acima do nível do mar |
| `hdop` | `float` | GGA | diluição horizontal da precisão |
| `fix`, `pdop`, `vdop` | `int`, `float` | GSA | tipo de fix (2 = 2D, 3 = 3D) e DOPs |
| `erro_lat`, `erro_lon`, `erro_alt` | `float` | GST | desvio-padrão do erro estimado (m) |

### Saída CSV para análise

Com `SAIDA_CSV 1`, cada época (5 por segundo, aceita ou não) também é impressa como
uma linha `CSV,...`, com cabeçalho no início:

```
CSV,tick_ms,unix,ms,lat,lon,alt,vel_ms,curso,qualidade,fix,sats,hdop,pdop,vdop,erro_lat,erro_lon,erro_alt,motivo
```

Grave o monitor serial em arquivo e filtre as linhas que começam com `CSV,` para
ajustar os limites do filtro no PC.

## Transmissão LoRa

Este projeto é só o **transmissor**. O **receptor** é um projeto separado, gravado em outra
Heltec V3, que precisa usar os mesmos parâmetros de rádio e o mesmo formato de pacote.

```
 Esta placa (transmissor)                                   Outra placa (projeto receptor)
 GPS -> gps_task -> gps_ultimo -> lora_tx_task ~~~ 915 MHz ~~~> recebe, decodifica e mostra
                                  (a cada 5 s)
```

A `lora_tx_task` acorda a cada `LORA_INTERVALO_MS`, copia as últimas épocas
(`gps_aceito` e `gps_ultimo`, protegidas por uma seção crítica do FreeRTOS), monta um
`pacote_t` e envia. A posição só vai no pacote (bit 0 de `flags` ligado) se a época
aceita tiver no máximo `GPS_MAX_IDADE_MS` (1 s). Caso contrário, o pacote vai sem posição
(lat, lon e alt zerados) e o log mostra o motivo: `fix 2D`, `GPS sem dados`, `leitura antiga` etc.
O display mostra `LoRa: #N enviado` na última linha.

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
| [main/main.c](main/main.c) | UART, configuração UBX, parser NMEA, montagem das épocas, saída, tarefa de transmissão LoRa e `app_main` |
| [main/gps_filtro.c](main/gps_filtro.c) / [main/gps_filtro.h](main/gps_filtro.h) | Filtro das leituras (camada 1: validação da qualidade) e seus limites. C puro, sem ESP-IDF |
| [main/oled.c](main/oled.c) / [main/oled.h](main/oled.h) | Driver do SSD1306 por I2C com fonte 5x7 (21 colunas x 8 linhas) |
| [main/lora.c](main/lora.c) / [main/lora.h](main/lora.h) | Driver do SX1262 por SPI (só transmissão) e parâmetros do rádio |
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
| `SAIDA_CSV` | 0 | `1` imprime todas as épocas (5 Hz) como linhas `CSV,...` |
| `GPS_MODELO_DINAMICO` | 6 | Modelo do filtro interno do GPS: 3 pedestre, 4 automotivo, 6/7/8 aéreo < 1g/2g/4g |
| `LORA_INTERVALO_MS` | 5000 | Intervalo entre transmissões |

Em [main/gps_filtro.h](main/gps_filtro.h) (critérios de qualidade):

| Define | Padrão | Função |
|---|---|---|
| `GPS_MIN_SATS` | 5 | Mínimo de satélites (o fix 3D exige 4, mas sem nenhuma redundância) |
| `GPS_MAX_HDOP` | 2.5 | HDOP máximo |
| `GPS_MAX_ERRO_M` | 10.0 | Erro horizontal máximo estimado pela GST (m); `0` desliga |
| `GPS_LEITURAS_ESTAVEIS` | 5 | Leituras boas seguidas antes de aceitar (1 s a 5 Hz) |
| `GPS_MAX_IDADE_MS` | 1000 | Idade máxima da posição para ela ser transmitida |

Em [main/lora.h](main/lora.h) (precisam ser **iguais** no projeto do receptor):

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
| `Posicao rejeitada` por muito tempo | Os critérios de [main/gps_filtro.h](main/gps_filtro.h) estão mais rígidos que o sinal disponível. Veja o motivo no display e no log; com céu aberto costuma passar em 1 a 3 min. |
| `GPS nao aceitou configurar a sentenca GST` | O módulo (alguns clones) não suporta GST. O filtro segue sem o critério de erro estimado. |
| `Could not open COM.. Acesso negado` | A porta está aberta por outro programa (monitor serial, Arduino IDE etc.). Feche-o e grave de novo. |
| `ClearCommError failed` no monitor | O USB desconectou (cabo, mau contato ou queda de alimentação). Reconecte e reabra o monitor. |
| `Falha ao iniciar o display` | O I2C não encontrou o OLED. Confira se a placa é a V3 (pinos 17/18/21). |
| `Falha ao iniciar o radio: ESP_ERR_NOT_FOUND` | O SX1262 não respondeu pelo SPI. Confira se a placa é a V3. |
| Receptor não recebe nada | Os parâmetros de `lora.h` e o formato do pacote precisam ser iguais no projeto do receptor, e as duas placas precisam de antena. Teste primeiro com as placas a poucos metros. |
| Aviso de Unicode do `idf.py` | Só afeta o terminal do Windows. Ative "UTF-8 Beta" nas configurações regionais ou defina `PYTHONUTF8=1`. |
| Editor acusa `stddef.h file not found` | O pacote `esp-clangd` vem sem os headers internos. O [.clangd](.clangd) aponta `-resource-dir` para o `esp-clang`; ajuste a versão no caminho se atualizar as ferramentas. |

## Próximos passos

- Gravar os pontos em CSV/GPX para visualizar o trajeto em um mapa.
- Filtro, camada 2: rejeitar saltos incompatíveis com a velocidade máxima do aeromodelo.
- Filtro, camada 3: suavização (mediana e depois Kalman com a velocidade da RMC).
- No projeto do receptor, repassar os pacotes por Wi-Fi para um mapa em tempo real (Traccar, Node-RED etc.).

## Licença

Domínio público ou CC0, conforme o arquivo [LICENSE](LICENSE).

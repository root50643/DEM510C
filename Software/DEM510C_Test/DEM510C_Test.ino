#include <SoftwareSerial.h>

// SoftwareSerial(rxPin, txPin)
SoftwareSerial UART_PORT(D5, D6);

constexpr unsigned long UART_BAUD_RATE = 1200;

// UART 在 1200 bps 時，每個 Byte 約需要 8.33 ms。
// 若連續 30 ms 沒有收到資料，視為一個回覆封包結束。
constexpr unsigned long RX_FRAME_GAP_MS = 30;

constexpr size_t RX_BUFFER_SIZE = 64;

byte rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
unsigned long lastRxByteTime = 0;


// ============================================================
// Modbus RTU 指令
// ============================================================

// 1. 讀取 Slave 02 的暫存器 0x000F，數量 1
// 02 03 00 0F 00 01 B4 3A
const byte CMD_READ_SLAVE2_REG_000F[] = {
  0x02, 0x03, 0x00, 0x0F,
  0x00, 0x01, 0xB4, 0x3A
};

// 2. 讀取 Slave 01 的暫存器 0x0002，數量 1
// 01 03 00 02 00 01 25 CA
const byte CMD_READ_SLAVE1_REG_0002[] = {
  0x01, 0x03, 0x00, 0x02,
  0x00, 0x01, 0x25, 0xCA
};

// 3. 讀取 Slave 02 的暫存器 0x0002，數量 1
// 02 03 00 02 00 01 25 F9
const byte CMD_READ_SLAVE2_REG_0002[] = {
  0x02, 0x03, 0x00, 0x02,
  0x00, 0x01, 0x25, 0xF9
};

// 4. 讀取 Slave 01 的暫存器 0x000F，數量 1
// 01 03 00 0F 00 01 B4 09
const byte CMD_READ_SLAVE1_REG_000F[] = {
  0x01, 0x03, 0x00, 0x0F,
  0x00, 0x01, 0xB4, 0x09
};

// 5. Slave 01：Latch Relay ON
// 01 05 00 00 FF 00 8C 3A
const byte CMD_RELAY_ON_SLAVE1[] = {
  0x01, 0x05, 0x00, 0x00,
  0xFF, 0x00, 0x8C, 0x3A
};

// 6. Slave 02：Latch Relay ON
// 02 05 00 00 FF 00 8C 09
const byte CMD_RELAY_ON_SLAVE2[] = {
  0x02, 0x05, 0x00, 0x00,
  0xFF, 0x00, 0x8C, 0x09
};


// ============================================================
// 指令步驟結構
// ============================================================

struct CommandStep {
  const char* name;
  const byte* data;
  size_t length;

  // 傳送這個指令後，等待多久再送下一個指令
  unsigned long waitAfterMs;
};


// ============================================================
// 插卡後的固定輪詢順序
// ============================================================

const CommandStep commandSequence[] = {
  {
    "Read Slave 02 Register 0x000F",
    CMD_READ_SLAVE2_REG_000F,
    sizeof(CMD_READ_SLAVE2_REG_000F),
    270
  },
  {
    "Read Slave 01 Register 0x0002",
    CMD_READ_SLAVE1_REG_0002,
    sizeof(CMD_READ_SLAVE1_REG_0002),
    270
  },
  {
    "Read Slave 02 Register 0x0002",
    CMD_READ_SLAVE2_REG_0002,
    sizeof(CMD_READ_SLAVE2_REG_0002),
    270
  },
  {
    "Read Slave 01 Register 0x000F",
    CMD_READ_SLAVE1_REG_000F,
    sizeof(CMD_READ_SLAVE1_REG_000F),
    270
  },
  {
    "Relay ON Slave 01",
    CMD_RELAY_ON_SLAVE1,
    sizeof(CMD_RELAY_ON_SLAVE1),
    270
  },
  {
    "Relay ON Slave 02",
    CMD_RELAY_ON_SLAVE2,
    sizeof(CMD_RELAY_ON_SLAVE2),

    // 前面五個間隔共 1350 ms。
    // 最後等待 650 ms，使完整循環約為 2000 ms。
    650
  }
};

constexpr size_t COMMAND_COUNT =
  sizeof(commandSequence) / sizeof(commandSequence[0]);

size_t currentCommandIndex = 0;
unsigned long nextSendTime = 0;


// ============================================================
// 顯示十六進位 Byte
// ============================================================

void printHexByte(byte value) {
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}


// ============================================================
// 傳送一個 Modbus 指令
// ============================================================

void sendCommand(const CommandStep& command) {
  UART_PORT.write(command.data, command.length);

  // 等待 SoftwareSerial 把資料送完
  UART_PORT.flush();

  Serial.println();
  Serial.print("[TX] ");
  Serial.println(command.name);

  Serial.print("     ");

  for (size_t i = 0; i < command.length; i++) {
    printHexByte(command.data[i]);

    if (i < command.length - 1) {
      Serial.print(' ');
    }
  }

  Serial.println();
}


// ============================================================
// 讀取 UART 回覆
// ============================================================

void readUartResponse() {
  while (UART_PORT.available() > 0) {
    byte value = UART_PORT.read();

    if (rxLength < RX_BUFFER_SIZE) {
      rxBuffer[rxLength] = value;
      rxLength++;
    }

    lastRxByteTime = millis();
  }

  // 已收到資料，而且超過一段時間沒有新資料，
  // 視為一個 Modbus RTU 回覆封包結束。
  if (
    rxLength > 0 &&
    millis() - lastRxByteTime >= RX_FRAME_GAP_MS
  ) {
    Serial.print("[RX] ");

    for (size_t i = 0; i < rxLength; i++) {
      printHexByte(rxBuffer[i]);

      if (i < rxLength - 1) {
        Serial.print(' ');
      }
    }

    Serial.println();

    // 清空 Buffer，準備接收下一個封包
    rxLength = 0;
  }
}


// ============================================================
// Arduino 初始化
// ============================================================

void setup() {
  Serial.begin(115200);
  UART_PORT.begin(UART_BAUD_RATE);

  Serial.println();
  Serial.println("DEM510C post-card polling simulator");
  Serial.println("UART baud rate: 1200 bps");
  Serial.println("Polling cycle: approximately 2 seconds");
  Serial.println();

  // 開機後立即傳送第一個命令
  nextSendTime = millis();
}


// ============================================================
// 主程式
// ============================================================

void loop() {
  // 持續接收並顯示從站回覆
  readUartResponse();

  // 使用非阻塞方式判斷是否應傳送下一個命令
  if ((long)(millis() - nextSendTime) >= 0) {
    const CommandStep& command =
      commandSequence[currentCommandIndex];

    sendCommand(command);

    nextSendTime = millis() + command.waitAfterMs;

    currentCommandIndex++;

    if (currentCommandIndex >= COMMAND_COUNT) {
      currentCommandIndex = 0;

      Serial.println("----- Next polling cycle -----");
    }
  }
}

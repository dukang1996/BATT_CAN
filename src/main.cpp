#include <Arduino.h>
#include "BatteryNode.h"
#include <ModbusMaster.h>
#include <EEPROM.h>

// 定义modbus通信引脚
#define RS485_TX_PIN      0
#define RS485_RX_PIN      1

BatteryNode battery_node;
ModbusMaster modbus_node;
HardwareSerial modbusSerial(1);  // 使用ESP32的Serial2作为Modbus串口

bool print_info_enabled = false;      // 调试信息打印开关

// 电池保护板从站地址
// 直接从BatteryNode获取参数
#define BATTERY_SLAVE_ADDR battery_node.getModbusAddress()
// Modbus寄存器起始地址
const uint16_t VOLTAGE_REG = 0x1290;  // 电压寄存器起始地址连读8个，电压：2个寄存器mv，功率：2个寄存器mw，电流：2个寄存器ma，温度1：1个寄存器，温度2：1个寄存器
const uint16_t SOC_REG = 0x12A6;     // SOC寄存器起始地址连读10个，剩余%：1个高字节，剩余容量mAH：2个寄存器，设计容量mAH：2个寄存器，循环次数：2个寄存器，循环总容量：2个寄存器，SOH：1个字节，低字


BatteryNode::BatteryRawData_t battery_raw_data = {0};
const uint32_t DATA_UPDATE_INTERVAL = 1000;  // Modbus读取间隔(ms)
uint32_t last_modbus_time = 0;

// 串口命令处理相关变量
String serial_input = "";
bool serial_command_ready = false;
bool parameters_modified = false;

void readModbusData();
void printBatteryInfo();
void processSerialCommands();
void handleSerialCommand(String command);
void showHelp();
void checkSerialInput();

// ============================ 串口输入检查函数 ============================
void checkSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_input.length() > 0) {
                serial_command_ready = true;
            }
        } else {
            serial_input += c;
        }
        if (serial_input.length() > 128) {
            Serial.println("\n错误: 输入过长");
            serial_input = "";
            serial_command_ready = false;
        }
    }
}
// ============================ 串口命令处理函数 ============================
void processSerialCommands() {
    if (serial_command_ready) {
        serial_command_ready = false;
        handleSerialCommand(serial_input);
        serial_input = "";
    }
}

void handleSerialCommand(String command) {
    command.trim();

    if (command == "help" || command == "?") {
        showHelp();
    }
    else if (command == "status") {
        Serial.println("\n=== 系统设置===");
        Serial.printf("Modbus站号: 0x%02X\n", battery_node.getModbusAddress());
        Serial.printf("CAN节点ID: %d\n", battery_node.getCANNodeID());
        Serial.printf("CAN波特率: %d bps\n", battery_node.getCANBaudrate());
        Serial.printf("遥测速率: %.1f Hz\n", battery_node.getTelemetryRate());
        Serial.printf("电池索引: %d\n", battery_node.getBatteryIndex());
        Serial.printf("调试信息: %s\n", print_info_enabled ? "开启" : "关闭");
        Serial.println("==================");
    }
    else if (command.startsWith("set ")) {
        command = command.substring(4);
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex > 0) {
            String param = command.substring(0, spaceIndex);
            String value = command.substring(spaceIndex + 1);

            if (param == "modbus_addr") {
                uint8_t new_addr = value.toInt();
                battery_node.setModbusAddress(new_addr);
                // 立即更新Modbus通信
                modbus_node.begin(new_addr, modbusSerial);
                battery_node.saveParameters(); // 保存参数
            }
            else if (param == "can_id") {
                uint8_t new_id = value.toInt();
                battery_node.setCANNodeID(new_id);
                battery_node.saveParameters();
            }
            else if (param == "can_baud") {
                uint32_t new_baud = value.toInt();
                battery_node.setCANBaudrate(new_baud);
                battery_node.saveParameters();
                Serial.println("注意: CAN波特率需要重启生效");
            }
            else if (param == "telem_rate") {
                float new_rate = value.toFloat();
                battery_node.setTelemetryRate(new_rate);
                battery_node.saveParameters();
            }
            else if (param == "battery_index") {
                uint8_t new_index = value.toInt();
                if (new_index <= 255) {
                    battery_node.setBatteryIndex(new_index);
                    battery_node.saveParameters();
                    //Serial.printf("电池索引设置为: %d\n", new_index);
                } else {
                    Serial.println("错误: 电池索引范围0-255");
                }
            }
            else if (param == "print") {
                if (value == "on" || value == "1") {
                    print_info_enabled = true;
                    Serial.println("调试信息打印: 开启");
                }
                else if (value == "off" || value == "0") {
                    print_info_enabled = false;
                    Serial.println("调试信息打印: 关闭");
                } else {
                    Serial.println("错误: 使用 'print on' 或 'print off'");
                }
            }
            else {
                Serial.println("错误: 未知参数");
                showHelp();
            }

        } else {
            Serial.println("错误: 参数设置格式错误");
            showHelp();
        }
    }
    else if (command == "save") {
        battery_node.saveParameters();
    }
    else if (command == "restart") {
        Serial.println("系统重启中...");
        delay(100);
        ESP.restart();
    }
    else if (command == "factory_reset") {
        Serial.println("恢复出厂设置...");
        // 恢复默认参数
        battery_node.setModbusAddress(0x01);
        battery_node.setCANNodeID(73);
        battery_node.setCANBaudrate(1000000);
        battery_node.setTelemetryRate(2.0f);
        battery_node.setBatteryIndex(0);
        print_info_enabled = false;
        battery_node.saveParameters();
        Serial.println("已恢复出厂设置");
    }
    else {
        Serial.println("错误: 未知命令");
        showHelp();
    }
}

void showHelp() {
    Serial.println("\n=== 可用命令 ===");
    Serial.println("help / ?        - 显示帮助信息");
    Serial.println("status          - 显示系统设置");
    Serial.println("set modbus_addr <1-247>  - 设置Modbus站号");
    Serial.println("set can_id <1-127>       - 设置CAN节点ID");
    Serial.println("set can_baud <波特率>     - 设置CAN波特率");
    Serial.println("set telem_rate <0.1-50.0> - 设置遥测速率(Hz)");
    Serial.println("set battery_index <0-255> - 设置电池索引");
    Serial.println("set print <on/off>       - 开启/关闭调试信息");
    Serial.println("save            - 手动保存参数");
    Serial.println("restart         - 重启系统");
    Serial.println("factory_reset   - 恢复出厂设置");
    Serial.println("==================");
}

// ============================ Modbus读取函数 ============================
void readModbusData() {
    uint8_t result;
    uint16_t data[10];  // 存储读取的寄存器值
     bool read_success = true;

    // 读取电压等第一帧 (8个寄存器，16字节)
    result = modbus_node.readHoldingRegisters(VOLTAGE_REG, 8);
    if (result == modbus_node.ku8MBSuccess) {
        // 假设大端字节序: 寄存器0为高16位，寄存器1为低16位
        uint32_t voltage_raw = ((uint32_t)modbus_node.getResponseBuffer(0) << 16) |
                              modbus_node.getResponseBuffer(1);
        battery_raw_data.voltage = voltage_raw / 1000.0f;  // mV转V
        uint32_t power_raw = ((uint32_t)modbus_node.getResponseBuffer(2) << 16) |
                             modbus_node.getResponseBuffer(3);
        battery_raw_data.average_power = power_raw / 1000.0f;  // mW转W
       // uint32_t current_raw = ((uint32_t)modbus_node.getResponseBuffer(4) << 16) |
                             // modbus_node.getResponseBuffer(5);
        battery_raw_data.current = battery_raw_data.average_power / battery_raw_data.voltage;  // 通过功率和电压计算电流
        uint16_t temperature_raw = modbus_node.getResponseBuffer(6);
        battery_raw_data.temperature = temperature_raw / 10.0f;  // 假设温度为整数，单位为°C
    } else {
        //Serial.printf("读取第一帧失败! 错误码: %d\n", result);
        read_success = false;
    }

    // 短暂延迟
    delay(100);

    // 读取第二帧 (10个寄存器，20字节)
    result = modbus_node.readHoldingRegisters(SOC_REG, 10);
    if (result == modbus_node.ku8MBSuccess) {
        uint8_t SOC_raw = modbus_node.getResponseBuffer(0);  // 低字节为SOC百分比
        battery_raw_data.state_of_charge = SOC_raw;  //SOC为百分比
        uint32_t capacity_remaining_raw = ((uint32_t)modbus_node.getResponseBuffer(1) << 16) |
                                          modbus_node.getResponseBuffer(2);
        battery_raw_data.capacity_remaining = capacity_remaining_raw / 1000.0f;  // mAH转AH
        uint32_t capacity_full_raw = ((uint32_t)modbus_node.getResponseBuffer(3) << 16) |
                                     modbus_node.getResponseBuffer(4);
        battery_raw_data.capacity_full = capacity_full_raw / 1000.0f;  // mAH转AH
        uint8_t SOH_raw = modbus_node.getResponseBuffer(9)>>8 ;  // 高字节为SOH百分比
        battery_raw_data.state_of_health = SOH_raw;  //SOH为百分比
    } else {
        //Serial.printf("读取第二帧失败! 错误码: %d\n", result);
        read_success = false;
    }
     if (!read_success) {
        memset(&battery_raw_data, 0, sizeof(battery_raw_data));
        Serial.println("Modbus通讯异常，电池数据已清零");
    }
}
// ============================ 调试输出函数 ============================
void printBatteryInfo() {
    static uint32_t print_count = 0;
    if (!print_info_enabled) return;

    Serial.println("\n========================");
    Serial.printf("数据包 #%lu\n", ++print_count);
    Serial.printf("电压: %.2f V\n", battery_raw_data.voltage);
    Serial.printf("平均功率: %.2f W\n", battery_raw_data.average_power);
    Serial.printf("电流: %.2f A\n", battery_raw_data.current);
    Serial.printf("温度: %.2f °C\n", battery_raw_data.temperature);
    Serial.printf("剩余容量: %.2f AH\n", battery_raw_data.capacity_remaining);
    Serial.printf("满容量: %.2f AH\n", battery_raw_data.capacity_full);
    Serial.printf("健康状态 (SOH): %d %%\n", battery_raw_data.state_of_health);
    Serial.printf("充电状态 (SOC): %d %%\n", battery_raw_data.state_of_charge);
    Serial.println("========================");
}

void setup() {
    Serial.begin(115200);
    delay(1000);


    Serial.println("Starting Battery Node...");
    // 加载系统参数
    battery_node.loadParameters();

    // 显示帮助信息
    showHelp();
    // 初始化Modbus串口
    modbusSerial.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    modbus_node.begin(BATTERY_SLAVE_ADDR, modbusSerial);

    // 初始化电池节点，使用静态节点ID 73
    if (!battery_node.begin(battery_node.getCANBaudrate(), battery_node.getCANNodeID())) {
        Serial.println("Failed to start battery modbus_node!");
        while(1) {
            delay(1000);
        }
    }

    Serial.println("Battery modbus_node started successfully");
}

void loop() {
    checkSerialInput();
    // 处理串口命令
    processSerialCommands();
    // 电池数据更新
    if (millis() - last_modbus_time > DATA_UPDATE_INTERVAL) {
        readModbusData();
        last_modbus_time = millis();
        // 打印调试信息
        printBatteryInfo();
    }

    battery_node.setBatteryData(&battery_raw_data);

    // 更新电池节点状态
    battery_node.update();

}

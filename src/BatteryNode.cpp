/*2026-1-11-dukang 发送nodestatus、batteryinfo广播消息功能正常，节点地址设为0时动态分配地址功能及响应请求功能不能正常工作，
和DRONECAN GUI TOOL连接时，可以响应请求，但GUI无法正常识别。*/
#include "BatteryNode.h"
#include <Arduino.h>
#include <EEPROM.h>

// 修复优先级常量定义（根据DroneCAN_ESP32库的实际定义）
#ifndef CANARD_PRIORITY_LOW
#define CANARD_PRIORITY_LOW 0x1E  // 低优先级
#endif

#ifndef CANARD_PRIORITY_MEDIUM
#define CANARD_PRIORITY_MEDIUM 0x0F  // 中等优先级
#endif

#ifndef CANARD_PRIORITY_HIGH
#define CANARD_PRIORITY_HIGH 0x00   // 高优先级
#endif

BatteryNode::BatteryNode()
    : last_status_time_(0), last_battery_time_(0),
      transfer_id_status_(0), transfer_id_battery_(0) {

    // 初始化默认参数
    settings_.can_node_id = 73;
    settings_.battery_index = 0;
    settings_.telem_rate = 10;
    settings_.crc = 0;

    // 初始化电池数据
    battery_data_.voltage = 0.0f;
    battery_data_.current = 0.0f;
    battery_data_.temperature = 25.0f;
    battery_data_.capacity_remaining = 0.0f;
    battery_data_.capacity_full = 100.0f;
    battery_data_.state_of_health = 100;
    battery_data_.state_of_charge = 0;

    // 初始化节点状态
    memset(&node_status_, 0, sizeof(node_status_));
    node_status_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
}

BatteryNode::~BatteryNode() {
}

bool BatteryNode::begin(uint32_t bitrate, uint8_t node_id) {
    // 加载参数
    loadParameters();

    // 设置实际使用的节点ID
    uint8_t actual_node_id = (settings_.can_node_id > 0) ? (uint8_t)settings_.can_node_id : node_id;
    //settings_.telem_rate = 2;
    // 设置回调函数
    dronecan_.setAcceptCallback(shouldAcceptTransfer);
    dronecan_.setReceiveCallback(onTransferReceived);

    // 初始化DroneCAN
    if (!dronecan_.begin(bitrate, actual_node_id,8192)) {
        Serial.println("Failed to initialize DroneCAN");
        return false;
    }

    Serial.println("BatteryNode initialized successfully");
    Serial.print("Node ID: ");
    Serial.println(actual_node_id);
    Serial.print("Telemetry rate: ");
    Serial.print(settings_.telem_rate);
    Serial.println(" Hz");

    return true;
}

void BatteryNode::update() {
    uint64_t current_time = getMicros64();

    // 更新DroneCAN状态
    dronecan_.update();

    // 每秒发送一次节点状态
    if (current_time - last_status_time_ > 1000000) {
        last_status_time_ = current_time;
        sendNodeStatus();
    }

    // 按配置频率发送电池信息
    uint32_t battery_interval = 1000000 / settings_.telem_rate;
    if (current_time - last_battery_time_ > battery_interval) {
        last_battery_time_ = current_time;
        //Serial.println("Sending BatteryInfo...");
        sendBatteryInfo();
    }
}

void BatteryNode::setBatteryData( BatteryRawData* battery_data) {
    battery_data_.voltage = battery_data->voltage;
    battery_data_.current = battery_data->current;
    battery_data_.temperature = battery_data->temperature;
    battery_data_.capacity_remaining = battery_data->capacity_remaining;
    battery_data_.capacity_full = battery_data->capacity_full;
    battery_data_.state_of_health = battery_data->state_of_health;
    battery_data_.state_of_charge = battery_data->state_of_charge;
    battery_data_.average_power = battery_data->average_power;
}


// 用于判断是否接受某个传输请求,过滤特定的数据类型
bool BatteryNode::shouldAcceptTransfer(const CanardInstance* ins,
                                      uint64_t* out_data_type_signature,
                                      uint16_t data_type_id,
                                      CanardTransferType transfer_type,
                                      uint8_t source_node_id) {
    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
                return true;
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
                return true;
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
                return true;
            case UAVCAN_PROTOCOL_RESTARTNODE_ID:
                *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
                return true;
        }
    }

    return false;
}

void BatteryNode::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
    // 获取BatteryNode实例
    BatteryNode* node = (BatteryNode*)ins->user_reference;

    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                node->handleGetNodeInfo(ins, transfer);
                break;
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
                node->handleParamGetSet(ins, transfer);
                break;
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
                node->handleParamExecuteOpcode(ins, transfer);
                break;
            case UAVCAN_PROTOCOL_RESTARTNODE_ID:
                node->handleRestartNode(ins, transfer);
                break;
        }
    }
}

void BatteryNode::handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer) {
    struct uavcan_protocol_GetNodeInfoResponse response;
    memset(&response, 0, sizeof(response));
    //Serial.println("处理getnodeinfo请求");
    // 节点状态
    node_status_.uptime_sec = getMicros64() / 1000000ULL;
    response.status = node_status_;

    // 软件版本
    response.software_version.major = 1;
    response.software_version.minor = 0;

    // 硬件版本
    response.hardware_version.major = 1;
    response.hardware_version.minor = 0;
    getUniqueID(response.hardware_version.unique_id);

    // 节点名称
    const char* node_name = "Dk.BatteryNode";
    strncpy((char*)response.name.data, node_name, sizeof(response.name.data));
    response.name.len = strlen(node_name);

    // 编码并发送响应
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint16_t size = uavcan_protocol_GetNodeInfoResponse_encode(&response, buffer);

    int8_t getinfo = dronecan_.requestOrRespond(transfer->source_node_id,
                              UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                              UAVCAN_PROTOCOL_GETNODEINFO_ID,
                              CANARD_PRIORITY_LOW ,
                              CanardResponse,
                              buffer,
                              size);
    //  Serial.print("GetNodeInfo response 结果：");
    //  Serial.println(getinfo);
    //  Serial.println("=====================");
}

void BatteryNode::handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer) {
    // 简化的参数处理实现
    struct uavcan_protocol_param_GetSetResponse response;
    memset(&response, 0, sizeof(response));

    // 这里可以实现完整的参数处理逻辑
    const char* param_name = "battery_index";
    strncpy((char*)response.name.data, param_name, sizeof(response.name.data));
    response.name.len = strlen(param_name);
    response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
    response.value.integer_value = settings_.battery_index;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t size = uavcan_protocol_param_GetSetResponse_encode(&response, buffer);

    dronecan_.requestOrRespond(transfer->source_node_id,
                              UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                              UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                              CANARD_PRIORITY_LOW,
                              CanardResponse,
                              buffer,
                              size);
}

void BatteryNode::handleParamExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer) {
    struct uavcan_protocol_param_ExecuteOpcodeResponse response;
    memset(&response, 0, sizeof(response));
    response.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&response, buffer);

    dronecan_.requestOrRespond(transfer->source_node_id,
                              UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                              UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                              CANARD_PRIORITY_LOW,
                              CanardResponse,
                              buffer,
                              size);
}

void BatteryNode::handleRestartNode(CanardInstance* ins, CanardRxTransfer* transfer) {
    Serial.println("Restart command received - resetting D.BatNode...");
    delay(100);
    ESP.restart();
}

void BatteryNode::sendNodeStatus() {
    node_status_.uptime_sec = getMicros64() / 1000000ULL;

    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint16_t size = uavcan_protocol_NodeStatus_encode(&node_status_, buffer);

    dronecan_.broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                       UAVCAN_PROTOCOL_NODESTATUS_ID,
                       CANARD_PRIORITY_LOW,
                       buffer,
                       size);
}

void BatteryNode::sendBatteryInfo() {
    //Serial.println("Entering sendBatteryInfo()");
    struct uavcan_equipment_power_BatteryInfo battery_info;
    memset(&battery_info, 0, sizeof(battery_info));

    // 填充电池信息
    battery_info.voltage = battery_data_.voltage;
    battery_info.average_power_10sec = battery_data_.average_power;
    battery_info.current = battery_data_.current;
    battery_info.temperature = battery_data_.temperature + 273.15f; // 转换为开尔文
    battery_info.state_of_charge_pct = battery_data_.state_of_charge;
    battery_info.state_of_health_pct = battery_data_.state_of_health;
    battery_info.remaining_capacity_wh = battery_data_.capacity_remaining;
    battery_info.full_charge_capacity_wh = battery_data_.capacity_full;
    battery_info.battery_id = (uint8_t)settings_.battery_index;

    // 使用正确的电池状态标志（根据实际的dronecan_msgs.h定义）
    // 如果标志不存在，使用通用状态指示
    if (battery_data_.current > 0) {
        // 放电状态
        battery_info.status_flags |= 0x01;  // 使用位标志
    } else if (battery_data_.current < 0) {
        // 充电状态
        battery_info.status_flags |= 0x02;  // 使用位标志
    }

    if (battery_data_.state_of_charge > 90) {
        // 电量充足
        battery_info.status_flags |= 0x04;  // 使用位标志
    }

    uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    uint16_t size = uavcan_equipment_power_BatteryInfo_encode(&battery_info, buffer);

    //Serial.print("Encoded BatteryInfo size: ");
    //Serial.println(size);

    bool result = dronecan_.broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                       UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                       CANARD_PRIORITY_LOW,
                       buffer,
                       size);
    //Serial.print("Broadcast result: ");
    //Serial.println(result);
}

uint64_t BatteryNode::getMicros64() {
    return (uint64_t)micros();
}

void BatteryNode::getUniqueID(uint8_t id[16]) {
    // 使用ESP32的MAC地址作为唯一ID
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    memset(id, 0, 16);
    memcpy(id, mac, 6);
    // 其余字节填充固定值
    for (int i = 6; i < 16; i++) {
        id[i] = 0xAA + i;
    }
}

// 添加参数保存方法
void BatteryNode::saveParameters() {
    EEPROM.begin(sizeof(Parameters));
    EEPROM.put(0, settings_);
    bool success = EEPROM.commit();  // ESP32需要调用commit来实际写入
    EEPROM.end();

    if (success) {
        Serial.println("参数已保存到EEPROM");
        parameters_modified_ = false;
    } else {
        Serial.println("参数保存失败");
    }
}

// 添加参数设置方法
void BatteryNode::setModbusAddress(uint8_t addr) {
    if (addr >= 1 && addr <= 247) {
        settings_.modbus_address = addr;
        parameters_modified_ = true;
        Serial.printf("Modbus地址设置为: 0x%02X\n", addr);
    } else {
        Serial.println("错误: Modbus地址范围1-247");
    }
}

void BatteryNode::setCANNodeID(uint8_t node_id) {
    if (node_id != (uint8_t)settings_.can_node_id) {
        settings_.can_node_id = node_id;
        parameters_modified_ = true;
        Serial.printf("CAN节点ID设置为: %d (需要重启生效)\n", node_id);
    }
}

void BatteryNode::setCANBaudrate(uint32_t baudrate) {
    if (baudrate != (uint32_t)settings_.can_baudrate) { // 注意：这里应该比较波特率
        if (baudrate == 1000000 || baudrate == 500000 || baudrate == 250000) {
        settings_.can_baudrate = baudrate;
        parameters_modified_ = true;
        Serial.printf("CAN波特率设置为: %d bps (需要重启生效)\n", baudrate);
        } else {
            Serial.println("错误: 支持的波特率: 1000000, 500000, 250000");
        }// 修正波特率设置
    }
}

void BatteryNode::setTelemetryRate(float rate) {
    if (rate != settings_.telem_rate) {
        if (rate >= 0.1f && rate <= 50.0f) {
                settings_.telem_rate = rate;
                parameters_modified_ = true;
                //Serial.printf("遥测速率设置为: %.1f Hz\n", rate);
            } else {
                Serial.println("错误: 遥测速率范围0.1-50.0 Hz");
            }
    }
}

void BatteryNode::setBatteryIndex(uint8_t index) {
    if (index != (uint8_t)settings_.battery_index) {
        settings_.battery_index = index;
        parameters_modified_ = true;
        Serial.printf("电池索引设置为: %d\n", index);
    }
}


// 修改loadParameters方法，添加参数验证和自动保存
void BatteryNode::loadParameters() {
    EEPROM.begin(sizeof(Parameters));
    //Serial.printf("参数大小：%d 字节\n", sizeof(Parameters));
    EEPROM.get(0, settings_);
    EEPROM.end();

    // 参数验证
    bool need_save = false;


    if (settings_.modbus_address < 1 || settings_.modbus_address > 247) {
        settings_.modbus_address = 1;
        need_save = true;
    }
    if (settings_.telem_rate == 0 || settings_.telem_rate > 100) {
        settings_.telem_rate = 10;
        need_save = true;
    }
    if (settings_.battery_index < 0 || settings_.battery_index > 255) {
        settings_.battery_index = 0;
        need_save = true;
    }
    if (settings_.can_node_id < 0 || settings_.can_node_id > 127) {
        settings_.can_node_id = 73; // 默认值
        need_save = true;
    }
    if (settings_.can_baudrate != 1000000 && settings_.can_baudrate != 500000 &&
        settings_.can_baudrate != 250000) {
        settings_.can_baudrate = 1000000;
        need_save = true;
    }


    // 如果参数无效，自动保存修正后的值
    if (need_save) {
        saveParameters();
    }

    Serial.printf("加载参数: CAN_ID=%d, CAN_Baudrate=%d, BatteryIndex=%d, TelemRate=%.1f, ModbusAddr=0x%02X\n",
                  settings_.can_node_id, settings_.can_baudrate, settings_.battery_index, settings_.telem_rate, settings_.modbus_address);
}


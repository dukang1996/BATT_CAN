#ifndef BATTERY_NODE_H
#define BATTERY_NODE_H

#include "DroneCAN_ESP32.h"

class BatteryNode {
public:
    BatteryNode();
    ~BatteryNode();

    // 数据缓存保护板原始数据结构
    struct BatteryRawData_t{
        float voltage;   //mv
        float average_power;  //mw
        float current;   //ma
        float temperature;   // °C
        float capacity_remaining;   //mAH
        float capacity_full;  //mAH
        uint8_t state_of_health; // %
        uint8_t state_of_charge; // %
    } ;
    using BatteryRawData = BatteryRawData_t;


    bool begin(uint32_t bitrate = 1000000, uint8_t node_id = 0);
    void update();
    void setBatteryData(BatteryRawData* data);
    // 添加参数管理方法
    void saveParameters();
    void loadParameters();
    void setModbusAddress(uint8_t addr);
    uint8_t getModbusAddress() const { return settings_.modbus_address; }
    void setCANNodeID(uint8_t node_id);
    uint8_t getCANNodeID() const { return settings_.can_node_id; }
    void setCANBaudrate(uint32_t baudrate);
    uint32_t getCANBaudrate() const { return settings_.can_baudrate; }
    void setTelemetryRate(float rate);
    float getTelemetryRate() const { return settings_.telem_rate; }
    void setBatteryIndex(uint8_t index);
    uint8_t getBatteryIndex() const { return settings_.battery_index; }


private:
    DroneCAN_ESP32 dronecan_;
    uint32_t last_status_time_;
    uint32_t last_battery_time_;
    uint8_t transfer_id_status_;
    uint8_t transfer_id_battery_;

    // 参数结构
    struct Parameters {
        uint8_t modbus_address = 0x01;
        uint8_t can_node_id = 73;
        uint32_t can_baudrate = 1000000;
        float telem_rate = 2.0f;
        uint8_t battery_index = 0;
        uint32_t crc = 0;
    };

    // 电池状态
    struct {
        float voltage;
        float average_power;
        float current;
        float temperature;
        float capacity_remaining;
        float capacity_full;
        uint8_t state_of_health;
        uint8_t state_of_charge;
    } battery_data_;

    Parameters settings_;

    // 节点状态
    struct uavcan_protocol_NodeStatus node_status_;

    // 回调函数
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                    uint64_t* out_data_type_signature,
                                    uint16_t data_type_id,
                                    CanardTransferType transfer_type,
                                    uint8_t source_node_id);

    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);

    // 消息处理函数
    void handleGetNodeInfo(CanardInstance* ins, CanardRxTransfer* transfer);
    void handleParamGetSet(CanardInstance* ins, CanardRxTransfer* transfer);
    void handleParamExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer);
    void handleRestartNode(CanardInstance* ins, CanardRxTransfer* transfer);

    // 消息发送函数
    void sendNodeStatus();
    void sendBatteryInfo();

    // 工具函数
    uint64_t getMicros64();
    void getUniqueID(uint8_t id[16]);
     // 添加参数保存标志
    bool parameters_modified_ = false;
};

#endif

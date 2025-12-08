/**
 * @file crc16.hpp
 * @brief CRC16 校验模块
 * 
 * 参考 OrangeAim-Drone 的 CRC16 算法逻辑重写
 * 用于下位机通信数据包的校验
 * 
 * Requirements: 6.6
 */

#ifndef ARMOR_DETECTOR_CRC16_HPP
#define ARMOR_DETECTOR_CRC16_HPP

#include <cstdint>
#include <vector>

namespace armor_detector {

/**
 * @brief CRC16 校验类
 * 
 * 使用 CRC-16/CCITT-FALSE 多项式的查表法实现
 * 初始值: 0xFFFF
 */
class CRC16 {
public:
    /// CRC 初始值
    static constexpr uint16_t CRC_INIT = 0xFFFF;

    /**
     * @brief 计算数据的 CRC16 校验值
     * @param data 数据指针
     * @param length 数据长度
     * @param init_crc 初始 CRC 值，默认为 CRC_INIT
     * @return CRC16 校验值
     */
    static uint16_t calculate(const uint8_t* data, uint32_t length, uint16_t init_crc = CRC_INIT);

    /**
     * @brief 计算 vector 数据的 CRC16 校验值
     * @param data 数据 vector
     * @param init_crc 初始 CRC 值，默认为 CRC_INIT
     * @return CRC16 校验值
     */
    static uint16_t calculate(const std::vector<uint8_t>& data, uint16_t init_crc = CRC_INIT);

    /**
     * @brief 在数据末尾追加 CRC16 校验值（小端序）
     * @param data 数据指针，需要预留 2 字节空间
     * @param length 总长度（包含 CRC 的 2 字节）
     * @note 如果 data 为空或 length <= 2，则不执行任何操作
     */
    static void append(uint8_t* data, uint32_t length);

    /**
     * @brief 在 vector 末尾追加 CRC16 校验值
     * @param data 数据 vector，会在末尾追加 2 字节
     */
    static void append(std::vector<uint8_t>& data);

    /**
     * @brief 验证数据包的 CRC16 校验值
     * @param data 数据指针（包含末尾的 CRC）
     * @param length 总长度（包含 CRC 的 2 字节）
     * @return true 校验通过，false 校验失败
     * @note 如果 data 为空或 length <= 2，返回 false
     */
    static bool verify(const uint8_t* data, uint32_t length);

    /**
     * @brief 验证 vector 数据包的 CRC16 校验值
     * @param data 数据 vector（包含末尾的 CRC）
     * @return true 校验通过，false 校验失败
     */
    static bool verify(const std::vector<uint8_t>& data);

private:
    /// CRC16 查找表
    static const uint16_t CRC_TABLE[256];
};

} // namespace armor_detector

#endif // ARMOR_DETECTOR_CRC16_HPP

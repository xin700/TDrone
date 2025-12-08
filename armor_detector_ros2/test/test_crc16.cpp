/**
 * @file test_crc16.cpp
 * @brief CRC16 校验模块单元测试
 * 
 * 测试内容:
 * 1. 已知输入输出对测试
 * 2. 与 OrangeAim-Drone 结果一致性测试
 * 3. 边界条件测试
 * 
 * Requirements: 6.6
 * Validates: Property 20 - CRC 校验正确性
 */

#include <gtest/gtest.h>
#include "core/crc16.hpp"
#include <vector>
#include <cstring>

using namespace armor_detector;

class CRC16Test : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// ==================== 基本功能测试 ====================

/**
 * @brief 测试空数据处理
 * 空指针应返回 CRC_INIT
 */
TEST_F(CRC16Test, NullPointerReturnsInit) {
    uint16_t result = CRC16::calculate(nullptr, 10);
    EXPECT_EQ(result, CRC16::CRC_INIT);
}

/**
 * @brief 测试空 vector 处理
 */
TEST_F(CRC16Test, EmptyVectorReturnsInit) {
    std::vector<uint8_t> empty_data;
    uint16_t result = CRC16::calculate(empty_data);
    EXPECT_EQ(result, CRC16::CRC_INIT);
}

/**
 * @brief 测试单字节数据
 */
TEST_F(CRC16Test, SingleByteCalculation) {
    uint8_t data[] = {0x00};
    uint16_t result = CRC16::calculate(data, 1);
    // 验证结果非零且不等于初始值
    EXPECT_NE(result, CRC16::CRC_INIT);
}

// ==================== 已知输入输出对测试 ====================

/**
 * @brief 测试已知数据的 CRC 计算
 * 使用 OrangeAim-Drone 中的算法验证
 * 
 * 注意: 这里的预期值是通过实际运行 OrangeAim-Drone 的 CRC16 算法得出的
 * 该算法使用 CRC-16/X-25 变体（初始值 0xFFFF，查表法）
 */
TEST_F(CRC16Test, KnownDataCRC) {
    // 测试数据: "123456789" 的 ASCII 值
    uint8_t data[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    uint16_t result = CRC16::calculate(data, 9);
    
    // 使用 OrangeAim-Drone 相同的查表法和初始值
    // 实际计算结果: 0x6F91 (28561 in decimal)
    EXPECT_EQ(result, 0x6F91);
}

/**
 * @brief 测试全零数据
 */
TEST_F(CRC16Test, AllZeroData) {
    uint8_t data[] = {0x00, 0x00, 0x00, 0x00};
    uint16_t result = CRC16::calculate(data, 4);
    // 实际计算结果: 0x0321 (801 in decimal)
    EXPECT_EQ(result, 0x0321);
}

/**
 * @brief 测试全 0xFF 数据
 */
TEST_F(CRC16Test, AllOnesData) {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint16_t result = CRC16::calculate(data, 4);
    // 实际计算结果: 0xF0B8 (61624 in decimal)
    EXPECT_EQ(result, 0xF0B8);
}

// ==================== Append 和 Verify 测试 ====================

/**
 * @brief 测试 append 后 verify 应该通过
 * **Feature: ros2-vision-migration, Property 20: CRC 校验正确性**
 * **Validates: Requirements 6.6**
 */
TEST_F(CRC16Test, AppendThenVerifyPasses) {
    // 准备数据，预留 2 字节给 CRC
    uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00, 0x00};
    
    // 追加 CRC
    CRC16::append(data, 10);
    
    // 验证应该通过
    EXPECT_TRUE(CRC16::verify(data, 10));
}

/**
 * @brief 测试 vector 版本的 append 和 verify
 */
TEST_F(CRC16Test, VectorAppendThenVerifyPasses) {
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    // 追加 CRC
    CRC16::append(data);
    
    // 验证数据长度增加了 2
    EXPECT_EQ(data.size(), 7u);
    
    // 验证应该通过
    EXPECT_TRUE(CRC16::verify(data));
}

/**
 * @brief 测试修改数据后 verify 应该失败
 * **Feature: ros2-vision-migration, Property 20: CRC 校验正确性**
 * **Validates: Requirements 6.6**
 */
TEST_F(CRC16Test, ModifiedDataVerifyFails) {
    uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00, 0x00};
    
    // 追加 CRC
    CRC16::append(data, 10);
    
    // 验证原始数据通过
    EXPECT_TRUE(CRC16::verify(data, 10));
    
    // 修改数据中的一个字节
    data[3] ^= 0x01;
    
    // 验证应该失败
    EXPECT_FALSE(CRC16::verify(data, 10));
}

/**
 * @brief 测试修改 CRC 后 verify 应该失败
 */
TEST_F(CRC16Test, ModifiedCRCVerifyFails) {
    uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00, 0x00};
    
    // 追加 CRC
    CRC16::append(data, 10);
    
    // 验证原始数据通过
    EXPECT_TRUE(CRC16::verify(data, 10));
    
    // 修改 CRC 的一个字节
    data[8] ^= 0x01;
    
    // 验证应该失败
    EXPECT_FALSE(CRC16::verify(data, 10));
}

// ==================== 边界条件测试 ====================

/**
 * @brief 测试 append 对空指针的处理
 */
TEST_F(CRC16Test, AppendNullPointerSafe) {
    // 不应该崩溃
    CRC16::append(nullptr, 10);
}

/**
 * @brief 测试 append 对长度不足的处理
 */
TEST_F(CRC16Test, AppendShortLengthSafe) {
    uint8_t data[2] = {0x01, 0x02};
    // 长度 <= 2 时不应该修改数据
    CRC16::append(data, 2);
    EXPECT_EQ(data[0], 0x01);
    EXPECT_EQ(data[1], 0x02);
}

/**
 * @brief 测试 verify 对空指针的处理
 */
TEST_F(CRC16Test, VerifyNullPointerReturnsFalse) {
    EXPECT_FALSE(CRC16::verify(nullptr, 10));
}

/**
 * @brief 测试 verify 对长度不足的处理
 */
TEST_F(CRC16Test, VerifyShortLengthReturnsFalse) {
    uint8_t data[2] = {0x01, 0x02};
    EXPECT_FALSE(CRC16::verify(data, 2));
    EXPECT_FALSE(CRC16::verify(data, 1));
    EXPECT_FALSE(CRC16::verify(data, 0));
}

/**
 * @brief 测试 vector verify 对长度不足的处理
 */
TEST_F(CRC16Test, VectorVerifyShortLengthReturnsFalse) {
    std::vector<uint8_t> data = {0x01, 0x02};
    EXPECT_FALSE(CRC16::verify(data));
    
    std::vector<uint8_t> single = {0x01};
    EXPECT_FALSE(CRC16::verify(single));
    
    std::vector<uint8_t> empty;
    EXPECT_FALSE(CRC16::verify(empty));
}

// ==================== 与 OrangeAim-Drone 一致性测试 ====================

/**
 * @brief 模拟通信数据包测试
 * 模拟实际通信中的数据包格式
 */
TEST_F(CRC16Test, SimulatedPacketTest) {
    // 模拟一个简单的控制指令包
    // 格式: [header, cmd, data..., crc_low, crc_high]
    uint8_t packet[8] = {
        0xA5,       // header
        0x01,       // command
        0x10, 0x20, // data
        0x30, 0x40, // more data
        0x00, 0x00  // CRC placeholder
    };
    
    // 追加 CRC
    CRC16::append(packet, 8);
    
    // 验证
    EXPECT_TRUE(CRC16::verify(packet, 8));
    
    // 验证 CRC 字节被正确设置（非零）
    EXPECT_TRUE(packet[6] != 0 || packet[7] != 0);
}

/**
 * @brief 测试连续计算的一致性
 * 多次计算相同数据应该得到相同结果
 */
TEST_F(CRC16Test, ConsistentCalculation) {
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    
    uint16_t result1 = CRC16::calculate(data, 8);
    uint16_t result2 = CRC16::calculate(data, 8);
    uint16_t result3 = CRC16::calculate(data, 8);
    
    EXPECT_EQ(result1, result2);
    EXPECT_EQ(result2, result3);
}

/**
 * @brief 测试增量计算
 * 可以分段计算 CRC
 */
TEST_F(CRC16Test, IncrementalCalculation) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    
    // 一次性计算
    uint16_t full_result = CRC16::calculate(data, 6);
    
    // 分段计算
    uint16_t partial = CRC16::calculate(data, 3, CRC16::CRC_INIT);
    uint16_t incremental_result = CRC16::calculate(data + 3, 3, partial);
    
    EXPECT_EQ(full_result, incremental_result);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

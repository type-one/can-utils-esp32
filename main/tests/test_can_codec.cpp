#include "can/can_codec.hpp"
#include <cstdint>
#include <gtest/gtest.h>

// https://kvaser.com/developer-blog/an-introduction-j1939-and-dbc-files/

// Test fixture for CAN codec tests
class CANCodecTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Setup code if needed
    }

    void TearDown() override
    {
        // Cleanup code if needed
    }
};

TEST_F(CANCodecTest, TestSigCodecUnsigned)
{
    can::sig_codec codec(0, 8, '1', '+');
    std::uint8_t data[8] = { 0 };
    codec(0xAB, data);
    EXPECT_EQ(data[0], 0xAB);
    std::uint64_t decoded = codec(data);
    EXPECT_EQ(decoded, 0xAB);
}

TEST_F(CANCodecTest, TestSigCalcTypeAddition)
{
    can::sig_calc_type<std::uint64_t> calc1(10);
    can::sig_calc_type<std::uint64_t> calc2(20);
    auto result = calc1 + calc2;
    EXPECT_EQ(result.get_raw(), 30);
}

TEST_F(CANCodecTest, TestSigCalcTypeComparison)
{
    can::sig_calc_type<std::int64_t> calc1(-10);
    can::sig_calc_type<std::int64_t> calc2(20);
    EXPECT_LT(calc1, calc2);
    EXPECT_GT(calc2, calc1);
}

TEST_F(CANCodecTest, TestPhysValueConversion)
{
    can::phys_value phys(0.1, 5.0);
    double result = phys(100, can::val_type_t::u64);
    EXPECT_DOUBLE_EQ(result, 15.0);
}

TEST_F(CANCodecTest, TestPhysValueConversionDifferentFactor)
{
    can::phys_value phys(0.5, -2.0);
    double result = phys(200, can::val_type_t::u64);
    EXPECT_DOUBLE_EQ(result, 98.0);
}

// Add tests with real CAN data values
TEST_F(CANCodecTest, TestRealCANData)
{
    // Example real CAN data values
    std::uint8_t real_data[8] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0 };
    can::sig_codec codec(0, 64, '1', '+');
    std::uint64_t decoded = codec(real_data);
    EXPECT_EQ(decoded, 0xF0DEBC9A78563412);
}

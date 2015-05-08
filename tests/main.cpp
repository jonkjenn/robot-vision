#include "bachelor-line.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <memory>

using namespace std;

namespace {
    void test()
    {
    }


    class MockDrive {
        public:
            MOCK_METHOD2(drive, bool(unsigned int power1, unsigned int power2));
            MOCK_METHOD0(getDistance,uint32_t());
            MOCK_METHOD6(driveDistance,void(unsigned int speed, unsigned long distance, std::function<void()> callback, bool reverse , bool use_ramping ,bool ignore_stop));
            MOCK_METHOD1(stop,void(std::function<void()> callback));
            MOCK_METHOD1(set_distance_sensor_stop,void(bool value));
    };

    //The fixture for testing class Foo.
        class LineFollowerTest : public ::testing::Test {
            protected:
                // You can remove any or all of the following functions if its body
                // is empty.
                LineFollowerTest() {
                    // You can do set-up work for each test here.
                    lf.setup(drive);
                }

                virtual ~LineFollowerTest() {
                    // You can do clean-up work that doesn't throw exceptions here.
                }

                // If the constructor and destructor are not enough for setting up
                // and cleaning up each test, you can define the following methods:

                virtual void SetUp() {
                    // Code here will be called immediately after the constructor (right
                    // before each test).
                }

                virtual void TearDown() {
                    // Code here will be called immediately after each test (right
                    // before the destructor).
                }

                // Objects declared here can be used by all tests in the test case for Foo.
                const uint8_t min_power = 70;
                const uint8_t max_power = 110;
                LineFollower<MockDrive> lf{max_power,min_power};
                std::shared_ptr<MockDrive> drive = std::make_shared<MockDrive>();
        };

    // Tests that the Foo::Bar() method does Abc
    TEST_F(LineFollowerTest, CheckPower) {
        EXPECT_EQ((uint8_t)70, check_power(min_power,max_power,80));
        EXPECT_EQ(min_power, check_power(min_power,max_power,60));
        EXPECT_EQ(min_power, check_power(min_power,max_power,0));
        EXPECT_EQ(max_power, check_power(min_power,max_power,120));
        EXPECT_EQ(max_power, check_power(min_power,max_power,255));
    }

    TEST_F(LineFollowerTest, ScalePower){
        EXPECT_EQ(max_power, scale_power(min_power,max_power,0));
        EXPECT_EQ((uint8_t)93, scale_power(min_power,max_power,110));
        EXPECT_EQ((uint8_t)107, scale_power(min_power,max_power,20));
        EXPECT_EQ((uint8_t)70, scale_power(min_power,max_power,255));
        EXPECT_EQ((uint8_t)82, scale_power(min_power,max_power,179));
    }

    TEST_F(LineFollowerTest, check_line_not_found)
    {
        unsigned int stopcount = 0;
        ASSERT_TRUE(check_line_not_found(1000,stopcount)); 
        ASSERT_TRUE(check_line_not_found(1,stopcount)); 
        ASSERT_TRUE(check_line_not_found(6999,stopcount)); 
        ASSERT_TRUE(check_line_not_found(5000,stopcount)); 
        ASSERT_TRUE(check_line_not_found(0,stopcount)); 
        ASSERT_TRUE(check_line_not_found(7000,stopcount)); 
        ASSERT_TRUE(check_line_not_found(9000,stopcount)); 
        ASSERT_TRUE(check_line_not_found(2500,stopcount)); 

        for(int i=0;i<21;i++)
        {
            check_line_not_found(0,stopcount);
        }
        ASSERT_FALSE(check_line_not_found(0,stopcount));
        ASSERT_FALSE(check_line_not_found(7000,stopcount));
        ASSERT_TRUE(check_line_not_found(2500,stopcount));
        for(int i=0;i<21;i++)
        {
            check_line_not_found(7000,stopcount);
        }
        ASSERT_FALSE(check_line_not_found(7000,stopcount));
        ASSERT_FALSE(check_line_not_found(0,stopcount));
        ASSERT_TRUE(check_line_not_found(3500,stopcount));
    }

    TEST_F(LineFollowerTest, update)
    {
    }

    // Tests that Foo does Xyz.
    TEST_F(LineFollowerTest, DoesXyz) {
        // Exercises the Xyz feature of Foo.
    }


}  // namespace

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

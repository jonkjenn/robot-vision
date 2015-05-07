#include "bachelor-line.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace std;

namespace {

    //The fixture for testing class Foo.
        class LineFollowerTest : public ::testing::Test {
            protected:
                // You can remove any or all of the following functions if its body
                // is empty.
                LineFollowerTest() {
                    // You can do set-up work for each test here.
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
                const uint8_t maxPower = 110;
                const uint8_t minPower = 70;
                LineFollower lf{maxPower,minPower};
        };

    // Tests that the Foo::Bar() method does Abc
    TEST_F(LineFollowerTest, CheckPower) {
        EXPECT_EQ((uint8_t)70, lf.check_power(80));
        EXPECT_EQ(minPower, lf.check_power(60));
        EXPECT_EQ(minPower, lf.check_power(0));
        EXPECT_EQ(maxPower, lf.check_power(120));
        EXPECT_EQ(maxPower, lf.check_power(255));
    }

    TEST_F(LineFollowerTest, ScalePower){
        EXPECT_EQ(maxPower, lf.scale_power(0));
        EXPECT_EQ((uint8_t)93, lf.scale_power(110));
        EXPECT_EQ((uint8_t)107, lf.scale_power(20));
        EXPECT_EQ((uint8_t)70, lf.scale_power(255));
        EXPECT_EQ((uint8_t)82, lf.scale_power(179));
    }

    TEST_F(LineFollowerTest, check_line_not_found)
    {
        ASSERT_TRUE(lf.check_line_not_found(1000)); 
        ASSERT_TRUE(lf.check_line_not_found(1)); 
        ASSERT_TRUE(lf.check_line_not_found(6999)); 
        ASSERT_TRUE(lf.check_line_not_found(5000)); 
        ASSERT_TRUE(lf.check_line_not_found(0)); 
        ASSERT_TRUE(lf.check_line_not_found(7000)); 
        ASSERT_TRUE(lf.check_line_not_found(9000)); 
        ASSERT_TRUE(lf.check_line_not_found(2500)); 

        for(int i=0;i<21;i++)
        {
            lf.check_line_not_found(0);
        }
        ASSERT_FALSE(lf.check_line_not_found(0));
        ASSERT_FALSE(lf.check_line_not_found(7000));
        ASSERT_TRUE(lf.check_line_not_found(2500));
        for(int i=0;i<21;i++)
        {
            lf.check_line_not_found(7000);
        }
        ASSERT_FALSE(lf.check_line_not_found(7000));
        ASSERT_FALSE(lf.check_line_not_found(0));
        ASSERT_TRUE(lf.check_line_not_found(3500));
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

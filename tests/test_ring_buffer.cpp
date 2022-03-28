#include <gtest/gtest.h>
#include <pam_mujoco/ring_buffer.hpp>

using namespace pam_mujoco;

TEST(RingBuffer, test_basics)
{
    RingBuffer<int, 3> rb;

    rb.current() = 1;
    rb.next();
    rb.current() = 2;
    rb.next();
    rb.current() = 3;
    rb.next();
    rb.current() = 4;

    ASSERT_EQ(rb.current(), 4);
    rb.next();
    ASSERT_EQ(rb.current(), 2);
    rb.next();
    ASSERT_EQ(rb.current(), 3);
    rb.next();
    ASSERT_EQ(rb.current(), 4);
}

TEST(RingBuffer, test_get_all)
{
    RingBuffer<int, 3> rb;

    rb.current() = 1;
    rb.next();
    rb.current() = 2;
    rb.next();
    rb.current() = 3;

    {
        const auto &data = rb.get_all();
        std::array<int, 3> expected = {1, 2, 3};
        ASSERT_EQ(data, expected);
    }

    rb.next();
    rb.current() = 4;

    {
        const auto &data = rb.get_all();
        std::array<int, 3> expected = {2, 3, 4};
        ASSERT_EQ(data, expected);
    }

    rb.next();
    rb.current() = 5;

    {
        const auto &data = rb.get_all();
        std::array<int, 3> expected = {3, 4, 5};
        ASSERT_EQ(data, expected);
    }
}

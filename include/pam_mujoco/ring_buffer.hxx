#pragma once

#include "ring_buffer.hpp"

namespace pam_mujoco
{
template <typename T, size_t buffer_size>
size_t RingBuffer<T, buffer_size>::size() const
{
    return buffer_size;
}

template <typename T, size_t buffer_size>
void RingBuffer<T, buffer_size>::next()
{
    index_ = (index_ + 1) % buffer_size;
}

template <typename T, size_t buffer_size>
T& RingBuffer<T, buffer_size>::current()
{
    return buffer_[index_];
}

template <typename T, size_t buffer_size>
T& RingBuffer<T, buffer_size>::operator[](std::size_t i)
{
    std::size_t actual_i = (index_ + 1 + i) % buffer_size;
    return buffer_[actual_i];
}
}  // namespace pam_mujoco

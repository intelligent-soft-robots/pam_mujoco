#pragma once

#include <algorithm>
#include <array>

namespace pam_mujoco
{
/**
 * @brief Simple ring buffer using static memory.
 *
 * @tparam T Type of the buffer elements.
 * @tparam size Size of the buffer
 */
template <typename T, size_t size>
class RingBuffer
{
public:

    /**
     * @brief Move to the next element.
     *
     * Important: This invalidates the pointer of a previous call to get().
     */
    void next()
    {
        index_ = (index_ + 1) % size;
    }

    //! @brief Get pointer on the currently active element.
    T* get()
    {
        return &buffer_[index_];
    }

    /**
     * @brief Get content of the buffer.
     *
     * The content of the buffer is reordered so that the oldest element is
     * first.
     *
     * Important: This invalidates the pointer of a previous call to get().
     */
    const std::array<T, size>& get_all()
    {
        // rotate the buffer such that the elements are ordered by age (oldest
        // first)
        // start for the rotated buffer is index + 1 (= oldest element)
        size_t start = (index_ + 1) % size;
        std::rotate(buffer_.begin(), buffer_.begin() + start, buffer_.end());

        // update index_ accordingly (active element is now at the end of the
        // buffer)
        index_ = size - 1;

        return buffer_;
    }

private:
    std::array<T, size> buffer_;
    size_t index_ = 0;
};
}  // namespace pam_mujoco

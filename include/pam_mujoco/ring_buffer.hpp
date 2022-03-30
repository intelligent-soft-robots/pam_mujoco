#pragma once

#include <algorithm>
#include <array>

namespace pam_mujoco
{
/**
 * @brief Simple ring buffer using static memory.
 *
 * @tparam T Type of the buffer elements.
 * @tparam buffer_size Size of the buffer
 */
template <typename T, size_t buffer_size>
class RingBuffer
{
public:
    //! @brief Get size of the buffer.
    size_t size() const
    {
        return buffer_size;
    }

    /**
     * @brief Move to the next element.
     *
     * Important: This invalidates the reference of a previous call to
     * current().
     */
    void next()
    {
        index_ = (index_ + 1) % buffer_size;
    }

    //! @brief Get writeable reference to the currently active element.
    T& current()
    {
        return buffer_[index_];
    }

    /**
     * @brief Get writeable reference to i-th element of the buffer.
     *
     * The index 0 always refers to the oldest element in the buffer, 1 to the
     * second oldest, and so on.
     *
     * No range check is performed, if ``i >= size()`` the behaviour is
     * undefined.
     */
    T& operator[](std::size_t i)
    {
        std::size_t actual_i = (index_ + 1 + i) % buffer_size;
        return buffer_[actual_i];
    }

    /**
     * @brief Get content of the buffer.
     *
     * The content of the buffer is reordered so that the oldest element is
     * first.
     *
     * Important: This invalidates the reference of a previous call to
     * current().
     */
    std::array<T, buffer_size>& get_all()
    {
        // rotate the buffer such that the elements are ordered by age (oldest
        // first)
        // start for the rotated buffer is index + 1 (= oldest element)
        size_t start = (index_ + 1) % buffer_size;
        std::rotate(buffer_.begin(), buffer_.begin() + start, buffer_.end());

        // update index_ accordingly (active element is now at the end of the
        // buffer)
        index_ = buffer_size - 1;

        return buffer_;
    }

private:
    std::array<T, buffer_size> buffer_;
    size_t index_ = 0;
};
}  // namespace pam_mujoco

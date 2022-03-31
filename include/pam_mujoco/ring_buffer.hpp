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
    size_t size() const;

    /**
     * @brief Move to the next element.
     *
     * Important: This invalidates the reference of a previous call to
     * current().
     */
    void next();

    //! @brief Get writeable reference to the currently active element.
    T& current();

    /**
     * @brief Get writeable reference to i-th element of the buffer.
     *
     * The index 0 always refers to the oldest element in the buffer, 1 to the
     * second oldest, and so on.
     *
     * No range check is performed, if ``i >= size()`` the behaviour is
     * undefined.
     */
    T& operator[](std::size_t i);

private:
    std::array<T, buffer_size> buffer_;
    size_t index_ = 0;
};

}  // namespace pam_mujoco

#include "ring_buffer.hxx"

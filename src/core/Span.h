#pragma once

#include <cstddef>
#include <type_traits>
#include <vector>

namespace fsim {

/// Minimal non-owning view over contiguous memory (C++17 stand-in for
/// std::span). Used at every hot-path boundary so no consumer receives copies.
template <typename T>
class Span {
public:
    using element_type = T;
    using value_type = std::remove_cv_t<T>;

    constexpr Span() noexcept = default;
    constexpr Span(T* data, std::size_t size) noexcept : data_(data), size_(size) {}

    template <std::size_t N>
    constexpr Span(T (&arr)[N]) noexcept : data_(arr), size_(N) {}

    template <typename U, typename = std::enable_if_t<std::is_convertible_v<U (*)[], T (*)[]>>>
    Span(std::vector<U>& v) noexcept : data_(v.data()), size_(v.size()) {}

    template <typename U, typename = std::enable_if_t<std::is_convertible_v<const U (*)[], T (*)[]>>>
    Span(const std::vector<U>& v) noexcept : data_(v.data()), size_(v.size()) {}

    constexpr T* data() const noexcept { return data_; }
    constexpr std::size_t size() const noexcept { return size_; }
    constexpr bool empty() const noexcept { return size_ == 0; }

    constexpr T& operator[](std::size_t i) const noexcept { return data_[i]; }
    constexpr T* begin() const noexcept { return data_; }
    constexpr T* end() const noexcept { return data_ + size_; }

    constexpr Span subspan(std::size_t offset, std::size_t count) const noexcept {
        return Span(data_ + offset, count);
    }

private:
    T* data_ = nullptr;
    std::size_t size_ = 0;
};

} // namespace fsim

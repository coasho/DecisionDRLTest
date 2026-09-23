#pragma once

#include "platform/Memory.h"

#include <algorithm>
#include <cstddef>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace fsim::core {

/// A growable array whose elements never move. Contiguous like std::vector,
/// but its address space is reserved once, up front, and backed with memory
/// only as the array grows, so a pointer or reference to an element stays
/// valid for as long as the array lives. For storage whose addresses are
/// handed out - the per-vehicle state snapshots the SDK promises are stable
/// while their vehicle lives - where a std::vector would move every element
/// the moment it outgrew its capacity, and everything handed out would dangle.
///
/// The reservation costs address space, not memory: maxSize elements' worth
/// is reserved, and only what has been used is committed.
template <typename T>
class StableArray {
    static_assert(std::is_nothrow_destructible_v<T>, "StableArray elements must not throw on destruction");

public:
    explicit StableArray(std::size_t maxSize) : max_(maxSize), page_(platform::pageSize()) {
        reserved_ = roundUp(std::max<std::size_t>(max_, 1) * sizeof(T));
        data_ = static_cast<T*>(platform::reserveAddressSpace(reserved_));
        if (!data_) throw std::bad_alloc();
    }
    ~StableArray() {
        for (std::size_t i = size_; i > 0; --i) data_[i - 1].~T();
        platform::releaseAddressSpace(data_, reserved_);
    }
    StableArray(const StableArray&) = delete;
    StableArray& operator=(const StableArray&) = delete;

    template <typename... Args>
    T& emplace_back(Args&&... args) {
        if (size_ == max_) throw std::length_error("StableArray: all " + std::to_string(max_) + " elements are in use");
        const std::size_t need = (size_ + 1) * sizeof(T);
        if (need > committed_) {
            // Grow geometrically, like a vector would, but in place.
            const std::size_t target = std::min(reserved_, roundUp(std::max(need, 2 * committed_)));
            if (!platform::commitMemory(reinterpret_cast<unsigned char*>(data_) + committed_, target - committed_))
                throw std::bad_alloc();
            committed_ = target;
        }
        T* p = ::new (static_cast<void*>(data_ + size_)) T(std::forward<Args>(args)...);
        ++size_;
        return *p;
    }

    T& operator[](std::size_t i) noexcept { return data_[i]; }
    const T& operator[](std::size_t i) const noexcept { return data_[i]; }
    T* data() noexcept { return data_; }
    const T* data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    std::size_t maxSize() const noexcept { return max_; }

private:
    std::size_t roundUp(std::size_t bytes) const noexcept { return (bytes + page_ - 1) / page_ * page_; }

    T* data_ = nullptr;
    std::size_t size_ = 0, max_ = 0, page_ = 4096, reserved_ = 0, committed_ = 0;
};

} // namespace fsim::core

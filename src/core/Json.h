#pragma once

// Minimal JSON document: parse, typed access with defaults, and a compact
// writer. Enough for scenario files and message payloads; no external
// dependency (design 9.13 "Scenario files").

#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace fsim::core {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Json>;
    using Object = std::vector<std::pair<std::string, Json>>; ///< insertion order kept

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool b) : value_(b) {}
    Json(double d) : value_(d) {}
    Json(int i) : value_(static_cast<double>(i)) {}
    Json(std::string s) : value_(std::move(s)) {}
    Json(const char* s) : value_(std::string(s)) {}
    Json(Array a) : value_(std::move(a)) {}
    Json(Object o) : value_(std::move(o)) {}

    /// Parse a document. Throws std::runtime_error with "<source>:<line>:<col>: ..." on error.
    static Json parse(std::string_view text, std::string_view source = "json");

    Type type() const noexcept { return static_cast<Type>(value_.index()); }
    bool isNull() const noexcept { return type() == Type::Null; }
    bool isBool() const noexcept { return type() == Type::Bool; }
    bool isNumber() const noexcept { return type() == Type::Number; }
    bool isString() const noexcept { return type() == Type::String; }
    bool isArray() const noexcept { return type() == Type::Array; }
    bool isObject() const noexcept { return type() == Type::Object; }

    bool asBool() const { return get<bool>("bool"); }
    double asNumber() const { return get<double>("number"); }
    const std::string& asString() const { return get<std::string>("string"); }
    const Array& asArray() const { return get<Array>("array"); }
    const Object& asObject() const { return get<Object>("object"); }

    /// Object member (null when absent or not an object).
    const Json* find(std::string_view key) const noexcept;
    bool has(std::string_view key) const noexcept { return find(key) != nullptr; }

    /// Typed member access with a fallback; a member of the wrong type throws.
    double number(std::string_view key, double fallback) const;
    bool boolean(std::string_view key, bool fallback) const;
    std::string string(std::string_view key, std::string_view fallback) const;
    const Json& child(std::string_view key) const; ///< a static null when absent

    Json& set(std::string key, Json value); ///< object insert-or-replace (turns null into an object)
    Json& push(Json value);                  ///< array append (turns null into an array)

    std::string dump() const; ///< compact text

private:
    template <typename T>
    const T& get(const char* what) const {
        if (const T* p = std::get_if<T>(&value_)) return *p;
        throw std::runtime_error(std::string("json: expected ") + what);
    }
    std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value_;
};

} // namespace fsim::core

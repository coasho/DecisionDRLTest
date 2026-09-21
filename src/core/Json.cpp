#include "core/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fsim::core {

namespace {

class Parser {
public:
    Parser(std::string_view text, std::string_view source) : text_(text), source_(source) {}

    Json document() {
        skipSpace();
        Json v = value();
        skipSpace();
        if (pos_ != text_.size()) fail("trailing characters after the document");
        return v;
    }

private:
    std::string_view text_, source_;
    std::size_t pos_ = 0;

    [[noreturn]] void fail(const std::string& what) const {
        std::size_t line = 1, col = 1;
        for (std::size_t i = 0; i < pos_ && i < text_.size(); ++i) {
            if (text_[i] == '\n') { ++line; col = 1; } else ++col;
        }
        throw std::runtime_error(std::string(source_) + ":" + std::to_string(line) + ":" + std::to_string(col) + ": " + what);
    }
    bool eof() const noexcept { return pos_ >= text_.size(); }
    char peek() const noexcept { return eof() ? '\0' : text_[pos_]; }
    void skipSpace() {
        for (;;) {
            while (!eof() && (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r')) ++pos_;
            // Comments are tolerated in scenario files (// ... and /* ... */).
            if (!eof() && peek() == '/' && pos_ + 1 < text_.size() && text_[pos_ + 1] == '/') {
                while (!eof() && peek() != '\n') ++pos_;
            } else if (!eof() && peek() == '/' && pos_ + 1 < text_.size() && text_[pos_ + 1] == '*') {
                pos_ += 2;
                while (!eof() && !(peek() == '*' && pos_ + 1 < text_.size() && text_[pos_ + 1] == '/')) ++pos_;
                if (eof()) fail("unterminated comment");
                pos_ += 2;
            } else {
                return;
            }
        }
    }
    void expect(char c) {
        if (peek() != c) fail(std::string("expected '") + c + "'");
        ++pos_;
    }
    bool consume(std::string_view word) {
        if (text_.substr(pos_, word.size()) != word) return false;
        pos_ += word.size();
        return true;
    }

    Json value() {
        switch (peek()) {
        case '{': return object();
        case '[': return array();
        case '"': return Json(string());
        case 't': if (consume("true")) return Json(true); break;
        case 'f': if (consume("false")) return Json(false); break;
        case 'n': if (consume("null")) return Json(nullptr); break;
        default: break;
        }
        if (peek() == '-' || (peek() >= '0' && peek() <= '9')) return number();
        fail("unexpected character");
    }

    Json number() {
        const std::size_t start = pos_;
        if (peek() == '-') ++pos_;
        while (!eof() && ((peek() >= '0' && peek() <= '9') || peek() == '.' || peek() == 'e' || peek() == 'E' || peek() == '+' || peek() == '-')) ++pos_;
        const std::string s(text_.substr(start, pos_ - start));
        char* end = nullptr;
        const double d = std::strtod(s.c_str(), &end);
        if (!end || *end != '\0' || s.empty()) fail("bad number '" + s + "'");
        return Json(d);
    }

    static void appendUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) out += static_cast<char>(cp);
        else if (cp < 0x800) { out += static_cast<char>(0xC0 | (cp >> 6)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
        else if (cp < 0x10000) { out += static_cast<char>(0xE0 | (cp >> 12)); out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
        else { out += static_cast<char>(0xF0 | (cp >> 18)); out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F)); out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
    }
    unsigned hex4() {
        if (pos_ + 4 > text_.size()) fail("bad \\u escape");
        unsigned v = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[pos_++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<unsigned>(c - 'A' + 10);
            else fail("bad \\u escape");
        }
        return v;
    }
    std::string string() {
        expect('"');
        std::string out;
        for (;;) {
            if (eof()) fail("unterminated string");
            char c = text_[pos_++];
            if (c == '"') return out;
            if (c != '\\') { out += c; continue; }
            if (eof()) fail("unterminated escape");
            c = text_[pos_++];
            switch (c) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp = hex4();
                if (cp >= 0xD800 && cp <= 0xDBFF && consume("\\u")) {
                    const unsigned lo = hex4();
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                appendUtf8(out, cp);
                break;
            }
            default: fail("bad escape");
            }
        }
    }

    Json array() {
        expect('[');
        Json::Array items;
        skipSpace();
        if (peek() == ']') { ++pos_; return Json(std::move(items)); }
        for (;;) {
            skipSpace();
            items.push_back(value());
            skipSpace();
            if (peek() == ',') { ++pos_; skipSpace(); if (peek() == ']') { ++pos_; break; } continue; } // trailing comma tolerated
            expect(']');
            break;
        }
        return Json(std::move(items));
    }

    Json object() {
        expect('{');
        Json::Object members;
        skipSpace();
        if (peek() == '}') { ++pos_; return Json(std::move(members)); }
        for (;;) {
            skipSpace();
            if (peek() != '"') fail("expected a member name");
            std::string key = string();
            skipSpace();
            expect(':');
            skipSpace();
            members.emplace_back(std::move(key), value());
            skipSpace();
            if (peek() == ',') { ++pos_; skipSpace(); if (peek() == '}') { ++pos_; break; } continue; }
            expect('}');
            break;
        }
        return Json(std::move(members));
    }
};

void dumpString(std::string& out, const std::string& s) {
    out += '"';
    for (const char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                out += buf;
            } else {
                out += c;
            }
        }
    }
    out += '"';
}

void dumpTo(std::string& out, const Json& j) {
    switch (j.type()) {
    case Json::Type::Null: out += "null"; break;
    case Json::Type::Bool: out += j.asBool() ? "true" : "false"; break;
    case Json::Type::Number: {
        const double d = j.asNumber();
        char buf[32];
        if (std::isfinite(d) && d == std::floor(d) && std::fabs(d) < 1e15) std::snprintf(buf, sizeof buf, "%.0f", d);
        else std::snprintf(buf, sizeof buf, "%.17g", d);
        out += buf;
        break;
    }
    case Json::Type::String: dumpString(out, j.asString()); break;
    case Json::Type::Array: {
        out += '[';
        bool first = true;
        for (const auto& v : j.asArray()) { if (!first) out += ','; first = false; dumpTo(out, v); }
        out += ']';
        break;
    }
    case Json::Type::Object: {
        out += '{';
        bool first = true;
        for (const auto& [k, v] : j.asObject()) { if (!first) out += ','; first = false; dumpString(out, k); out += ':'; dumpTo(out, v); }
        out += '}';
        break;
    }
    }
}

} // namespace

Json Json::parse(std::string_view text, std::string_view source) { return Parser(text, source).document(); }

const Json* Json::find(std::string_view key) const noexcept {
    const Object* o = std::get_if<Object>(&value_);
    if (!o) return nullptr;
    for (const auto& [k, v] : *o)
        if (k == key) return &v;
    return nullptr;
}

double Json::number(std::string_view key, double fallback) const {
    const Json* v = find(key);
    if (!v || v->isNull()) return fallback;
    if (!v->isNumber()) throw std::runtime_error("json: '" + std::string(key) + "' must be a number");
    return v->asNumber();
}

bool Json::boolean(std::string_view key, bool fallback) const {
    const Json* v = find(key);
    if (!v || v->isNull()) return fallback;
    if (v->isNumber()) return v->asNumber() != 0.0;
    if (!v->isBool()) throw std::runtime_error("json: '" + std::string(key) + "' must be true or false");
    return v->asBool();
}

std::string Json::string(std::string_view key, std::string_view fallback) const {
    const Json* v = find(key);
    if (!v || v->isNull()) return std::string(fallback);
    if (!v->isString()) throw std::runtime_error("json: '" + std::string(key) + "' must be a string");
    return v->asString();
}

const Json& Json::child(std::string_view key) const {
    static const Json null;
    const Json* v = find(key);
    return v ? *v : null;
}

Json& Json::set(std::string key, Json value) {
    if (!isObject()) value_ = Object{};
    auto& o = std::get<Object>(value_);
    for (auto& [k, v] : o)
        if (k == key) { v = std::move(value); return *this; }
    o.emplace_back(std::move(key), std::move(value));
    return *this;
}

Json& Json::push(Json value) {
    if (!isArray()) value_ = Array{};
    std::get<Array>(value_).push_back(std::move(value));
    return *this;
}

std::string Json::dump() const {
    std::string out;
    dumpTo(out, *this);
    return out;
}

} // namespace fsim::core

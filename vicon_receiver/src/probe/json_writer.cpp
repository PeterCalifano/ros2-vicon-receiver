/**
 * @file json_writer.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Implementation of the minimal streaming JSON writer.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/json_writer.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace vicon_probe
{
    JsonWriter::JsonWriter(std::ostream &stream, int indent_width)
        : stream_(stream), indent_width_(indent_width)
    {
    }

    void JsonWriter::begin_object()
    {
        open_container(ContainerType::Object, '{');
    }

    void JsonWriter::end_object()
    {
        close_container(ContainerType::Object, '}');
    }

    void JsonWriter::begin_array()
    {
        open_container(ContainerType::Array, '[');
    }

    void JsonWriter::end_array()
    {
        close_container(ContainerType::Array, ']');
    }

    void JsonWriter::key(const std::string &name)
    {
        if (levels_.empty() || levels_.back().type != ContainerType::Object)
        {
            throw std::logic_error("JsonWriter: key '" + name + "' outside an object");
        }

        Level &level = levels_.back();
        if (level.key_pending)
        {
            throw std::logic_error("JsonWriter: key '" + name + "' after a key without value");
        }

        if (level.has_items)
        {
            stream_ << ',';
        }
        write_newline_and_indent();
        stream_ << '"' << escape(name) << "\": ";

        level.has_items = true;
        level.key_pending = true;
    }

    void JsonWriter::value(const std::string &text)
    {
        write_raw_value('"' + escape(text) + '"');
    }

    void JsonWriter::value(const char *text)
    {
        value(std::string(text));
    }

    void JsonWriter::value(double number)
    {
        write_raw_value(format_number(number));
    }

    void JsonWriter::value(bool flag)
    {
        write_raw_value(flag ? "true" : "false");
    }

    void JsonWriter::null()
    {
        write_raw_value("null");
    }

    bool JsonWriter::is_complete() const
    {
        return top_level_written_ && levels_.empty();
    }

    std::string JsonWriter::escape(const std::string &text)
    {
        std::string escaped;
        escaped.reserve(text.size());

        for (const char character : text)
        {
            switch (character)
            {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            case '\b':
                escaped += "\\b";
                break;
            case '\f':
                escaped += "\\f";
                break;
            default:
                // Other control characters must be written as \u00XX; bytes >= 0x80 (UTF-8)
                // are valid in JSON strings and copied unchanged
                if (static_cast<unsigned char>(character) < 0x20)
                {
                    std::array<char, 8> buffer{};
                    std::snprintf(buffer.data(),
                                  buffer.size(),
                                  "\\u%04x",
                                  static_cast<unsigned int>(static_cast<unsigned char>(character)));
                    escaped += buffer.data();
                }
                else
                {
                    escaped += character;
                }
                break;
            }
        }
        return escaped;
    }

    std::string JsonWriter::format_number(double number)
    {
        if (!std::isfinite(number))
        {
            return "null";
        }

        // std::to_chars without a precision gives the shortest text that reads back to the same
        // double, e.g. 0.1 -> "0.1" rather than "0.10000000000000001"
        std::array<char, 64> buffer{};
        const std::to_chars_result result =
            std::to_chars(buffer.data(), buffer.data() + buffer.size(), number);
        return std::string(buffer.data(), result.ptr);
    }

    void JsonWriter::begin_value()
    {
        if (levels_.empty())
        {
            if (top_level_written_)
            {
                throw std::logic_error("JsonWriter: a document has a single top-level value");
            }
            top_level_written_ = true;
            return;
        }

        Level &level = levels_.back();
        if (level.type == ContainerType::Object)
        {
            // In an object the comma and indentation were written by key()
            if (!level.key_pending)
            {
                throw std::logic_error("JsonWriter: value in an object without a key");
            }
            level.key_pending = false;
            return;
        }

        if (level.has_items)
        {
            stream_ << ',';
        }
        write_newline_and_indent();
        level.has_items = true;
    }

    void JsonWriter::write_raw_value(const std::string &json_text)
    {
        begin_value();
        stream_ << json_text;
    }

    void JsonWriter::open_container(ContainerType type, char opening)
    {
        begin_value();
        stream_ << opening;
        levels_.push_back(Level{type});
    }

    void JsonWriter::close_container(ContainerType type, char closing)
    {
        if (levels_.empty() || levels_.back().type != type)
        {
            throw std::logic_error(std::string("JsonWriter: unexpected '") + closing + "'");
        }
        if (levels_.back().key_pending)
        {
            throw std::logic_error("JsonWriter: object closed after a key without value");
        }

        const bool had_items = levels_.back().has_items;
        levels_.pop_back();

        // Empty containers stay on one line: {} and []
        if (had_items)
        {
            write_newline_and_indent();
        }
        stream_ << closing;

        if (levels_.empty())
        {
            stream_ << '\n';
        }
    }

    void JsonWriter::write_newline_and_indent()
    {
        stream_ << '\n';
        const std::size_t spaces = levels_.size() * static_cast<std::size_t>(indent_width_);
        stream_ << std::string(spaces, ' ');
    }
} // namespace vicon_probe

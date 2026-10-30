/**
 * @file json_writer.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Minimal streaming JSON writer used by the Vicon probe (no external dependency).
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_PROBE_JSON_WRITER_HPP
#define VICON_RECEIVER_PROBE_JSON_WRITER_HPP

#include <ostream>
#include <string>
#include <type_traits>
#include <vector>

namespace vicon_probe
{
    /**
     * @brief Writes a pretty-printed JSON document to a stream, one element at a time.
     *
     * The writer keeps track of the open objects and arrays, inserts commas and indentation, and
     * escapes strings. Misuse (e.g. a value in an object without a key, or closing a container that
     * is not open) throws std::logic_error, so a bug cannot silently produce invalid JSON.
     *
     * Non-finite numbers (NaN, infinity) have no JSON representation and are written as null.
     *
     * Example:
     * @code
     *   JsonWriter writer(std::cout);
     *   writer.begin_object();
     *   writer.member("frame", 42);
     *   writer.key("position_mm");
     *   writer.begin_array();
     *   writer.value(1.5);
     *   writer.end_array();
     *   writer.end_object();
     *   // {
     *   //   "frame": 42,
     *   //   "position_mm": [
     *   //     1.5
     *   //   ]
     *   // }
     * @endcode
     */
    class JsonWriter
    {
      public:
        /**
         * @param stream        Output stream; a stream without buffer discards the output.
         * @param indent_width  Number of spaces per nesting level.
         */
        explicit JsonWriter(std::ostream &stream, int indent_width = 2);

        /// Open an object (as a value: after key() in an object, in an array, or at top level).
        void begin_object();

        /// Close the innermost object.
        void end_object();

        /// Open an array (as a value).
        void begin_array();

        /// Close the innermost array.
        void end_array();

        /// Write the name of the next member of the innermost object.
        void key(const std::string &name);

        /// Write a string value.
        void value(const std::string &text);

        /// Write a string value (overload so that literals are not converted to bool).
        void value(const char *text);

        /// Write a floating-point value (shortest representation that round-trips).
        void value(double number);

        /// Write a boolean value.
        void value(bool flag);

        /// Write an integer value of any integral type except bool.
        template <typename Integer,
                  std::enable_if_t<std::is_integral_v<Integer> && !std::is_same_v<Integer, bool>,
                                   int> = 0>
        void value(Integer number)
        {
            write_raw_value(std::to_string(number));
        }

        /// Write a null value.
        void null();

        /// Write a member (key and value) of the innermost object.
        template <typename Value> void member(const std::string &name, const Value &member_value)
        {
            key(name);
            value(member_value);
        }

        /// Write a member whose value is an array of numbers.
        template <typename Number>
        void member_array(const std::string &name, const Number *numbers, std::size_t count)
        {
            key(name);
            begin_array();
            for (std::size_t index = 0; index < count; ++index)
            {
                value(numbers[index]);
            }
            end_array();
        }

        /// Whether exactly one top-level value has been written and every container is closed.
        bool is_complete() const;

        /// Escape a string for inclusion between JSON double quotes.
        static std::string escape(const std::string &text);

        /// Format a number as JSON ("null" if it is not finite).
        static std::string format_number(double number);

      private:
        /// Kind of an open container.
        enum class ContainerType
        {
            Object,
            Array
        };

        /// State of one open container.
        struct Level
        {
            ContainerType type;       ///< Object or array
            bool has_items = false;   ///< At least one member/element written
            bool key_pending = false; ///< (objects) key written, waiting for its value
        };

        /// Prepare the stream for a value (comma, newline, indentation) and validate the state.
        void begin_value();

        /// Write a value given as already formatted JSON text.
        void write_raw_value(const std::string &json_text);

        /// Open a container after begin_value().
        void open_container(ContainerType type, char opening);

        /// Close the innermost container, checking its type.
        void close_container(ContainerType type, char closing);

        /// Write a newline and the indentation of the current nesting level.
        void write_newline_and_indent();

        std::ostream &stream_;
        int indent_width_;
        std::vector<Level> levels_;
        bool top_level_written_ = false;
    };
} // namespace vicon_probe

#endif // VICON_RECEIVER_PROBE_JSON_WRITER_HPP

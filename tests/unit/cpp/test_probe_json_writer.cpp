/**
 * @file test_probe_json_writer.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of vicon_probe::JsonWriter: layout, escaping, numbers and misuse detection.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/json_writer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using vicon_probe::JsonWriter;

// -------------------------------------------------------------------------------------------------
// Layout
// -------------------------------------------------------------------------------------------------

TEST(JsonWriter, WritesNestedDocumentWithIndentation)
{
    std::ostringstream output;
    JsonWriter writer(output);

    writer.begin_object();
    writer.member("frame", 42);
    writer.member("name", "cf0");
    writer.key("position_mm");
    writer.begin_array();
    writer.value(1.5);
    writer.value(-2.0);
    writer.end_array();
    writer.member("occluded", false);
    writer.end_object();

    const std::string expected = "{\n"
                                 "  \"frame\": 42,\n"
                                 "  \"name\": \"cf0\",\n"
                                 "  \"position_mm\": [\n"
                                 "    1.5,\n"
                                 "    -2\n"
                                 "  ],\n"
                                 "  \"occluded\": false\n"
                                 "}\n";
    EXPECT_EQ(output.str(), expected);
    EXPECT_TRUE(writer.is_complete());
}

TEST(JsonWriter, EmptyContainersStayOnOneLine)
{
    std::ostringstream output;
    JsonWriter writer(output);

    writer.begin_object();
    writer.key("subjects");
    writer.begin_array();
    writer.end_array();
    writer.key("errors");
    writer.begin_object();
    writer.end_object();
    writer.end_object();

    EXPECT_EQ(output.str(), "{\n  \"subjects\": [],\n  \"errors\": {}\n}\n");
}

TEST(JsonWriter, ArrayOfObjects)
{
    std::ostringstream output;
    JsonWriter writer(output);

    writer.begin_array();
    writer.begin_object();
    writer.member("id", 1);
    writer.end_object();
    writer.begin_object();
    writer.member("id", 2);
    writer.end_object();
    writer.end_array();

    EXPECT_EQ(output.str(), "[\n  {\n    \"id\": 1\n  },\n  {\n    \"id\": 2\n  }\n]\n");
}

TEST(JsonWriter, MemberArrayWritesAllElements)
{
    std::ostringstream output;
    JsonWriter writer(output, 0);
    const double quaternion[4] = {0.0, 0.0, 0.5, 1.0};

    writer.begin_object();
    writer.member_array("rotation_xyzw", quaternion, 4);
    writer.end_object();

    // Indentation 0: one element per line, no leading spaces
    EXPECT_EQ(output.str(), "{\n\"rotation_xyzw\": [\n0,\n0,\n0.5,\n1\n]\n}\n");
}

TEST(JsonWriter, IntegerTypesAreWrittenExactly)
{
    std::ostringstream output;
    JsonWriter writer(output, 0);

    writer.begin_array();
    writer.value(std::numeric_limits<std::uint64_t>::max());
    writer.value(std::numeric_limits<std::int64_t>::min());
    writer.value(7u);
    writer.end_array();

    EXPECT_EQ(output.str(), "[\n18446744073709551615,\n-9223372036854775808,\n7\n]\n");
}

TEST(JsonWriter, IsCompleteOnlyAfterTopLevelValueIsClosed)
{
    std::ostringstream output;
    JsonWriter writer(output);

    EXPECT_FALSE(writer.is_complete());
    writer.begin_object();
    EXPECT_FALSE(writer.is_complete());
    writer.end_object();
    EXPECT_TRUE(writer.is_complete());
}

TEST(JsonWriter, StreamWithoutBufferDiscardsOutput)
{
    // Used by the probe when no --json file is given
    std::ostream discarded_output(nullptr);
    JsonWriter writer(discarded_output);

    writer.begin_object();
    writer.member("frame", 1);
    writer.end_object();

    EXPECT_TRUE(writer.is_complete());
}

// -------------------------------------------------------------------------------------------------
// Strings
// -------------------------------------------------------------------------------------------------

TEST(JsonWriter, EscapesSpecialCharacters)
{
    EXPECT_EQ(JsonWriter::escape("plain"), "plain");
    EXPECT_EQ(JsonWriter::escape("say \"hi\""), "say \\\"hi\\\"");
    EXPECT_EQ(JsonWriter::escape("C:\\vicon"), "C:\\\\vicon");
    EXPECT_EQ(JsonWriter::escape("a\nb\tc\rd"), "a\\nb\\tc\\rd");
    EXPECT_EQ(JsonWriter::escape("\b\f"), "\\b\\f");
}

TEST(JsonWriter, EscapesOtherControlCharactersAsUnicode)
{
    EXPECT_EQ(JsonWriter::escape(std::string("\x01", 1)), "\\u0001");
    EXPECT_EQ(JsonWriter::escape(std::string("\x1f", 1)), "\\u001f");
}

TEST(JsonWriter, KeepsUtf8AndSubjectNamesUnchanged)
{
    // Vicon names can contain '-' and other characters that are fine in JSON
    EXPECT_EQ(JsonWriter::escape("g1-unitree-torso-cam"), "g1-unitree-torso-cam");
    EXPECT_EQ(JsonWriter::escape("Kamera \xc3\xa4"), "Kamera \xc3\xa4");
}

TEST(JsonWriter, EscapesKeys)
{
    std::ostringstream output;
    JsonWriter writer(output, 0);

    writer.begin_object();
    writer.member("a\"b", 1);
    writer.end_object();

    EXPECT_EQ(output.str(), "{\n\"a\\\"b\": 1\n}\n");
}

// -------------------------------------------------------------------------------------------------
// Numbers
// -------------------------------------------------------------------------------------------------

TEST(JsonWriter, FormatsShortestRoundTripNumbers)
{
    EXPECT_EQ(JsonWriter::format_number(0.1), "0.1");
    EXPECT_EQ(JsonWriter::format_number(-1234.5), "-1234.5");
    EXPECT_EQ(JsonWriter::format_number(3.0), "3");

    // The text reads back to exactly the same double
    const double value = 0.12345678901234568;
    EXPECT_EQ(std::stod(JsonWriter::format_number(value)), value);
}

TEST(JsonWriter, NonFiniteNumbersBecomeNull)
{
    EXPECT_EQ(JsonWriter::format_number(std::nan("")), "null");
    EXPECT_EQ(JsonWriter::format_number(std::numeric_limits<double>::infinity()), "null");
    EXPECT_EQ(JsonWriter::format_number(-std::numeric_limits<double>::infinity()), "null");
}

// -------------------------------------------------------------------------------------------------
// Misuse is detected
// -------------------------------------------------------------------------------------------------

TEST(JsonWriter, ValueInObjectWithoutKeyThrows)
{
    std::ostringstream output;
    JsonWriter writer(output);
    writer.begin_object();
    EXPECT_THROW(writer.value(1), std::logic_error);
}

TEST(JsonWriter, KeyOutsideObjectThrows)
{
    std::ostringstream output;
    JsonWriter writer(output);
    EXPECT_THROW(writer.key("top"), std::logic_error);

    writer.begin_array();
    EXPECT_THROW(writer.key("in_array"), std::logic_error);
}

TEST(JsonWriter, TwoKeysInARowThrow)
{
    std::ostringstream output;
    JsonWriter writer(output);
    writer.begin_object();
    writer.key("first");
    EXPECT_THROW(writer.key("second"), std::logic_error);
}

TEST(JsonWriter, MismatchedClosingThrows)
{
    std::ostringstream output;
    JsonWriter writer(output);
    writer.begin_object();
    EXPECT_THROW(writer.end_array(), std::logic_error);
}

TEST(JsonWriter, ClosingObjectAfterDanglingKeyThrows)
{
    std::ostringstream output;
    JsonWriter writer(output);
    writer.begin_object();
    writer.key("dangling");
    EXPECT_THROW(writer.end_object(), std::logic_error);
}

TEST(JsonWriter, SecondTopLevelValueThrows)
{
    std::ostringstream output;
    JsonWriter writer(output);
    writer.begin_object();
    writer.end_object();
    EXPECT_THROW(writer.begin_object(), std::logic_error);
}

/**
 * @file vicon_probe.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Probe that records everything a Vicon DataStream server sends.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/vicon_probe.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

#include "vicon_receiver/probe/sdk_names.hpp"

namespace vicon_probe
{
    namespace sdk = ViconDataStreamSDK::CPP;

    namespace
    {
        /// Pause between attempts while the server has not sent a first frame.
        constexpr std::chrono::milliseconds kNoFrameRetryDelay{10};

        /// Seconds since the Unix epoch, from the system clock.
        double unix_time_seconds()
        {
            const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
            return std::chrono::duration<double>(since_epoch).count();
        }

        /// Current UTC time as ISO 8601 text, e.g. "2026-10-30T12:00:00Z".
        std::string utc_timestamp()
        {
            const std::time_t now = std::time(nullptr);
            std::tm utc_time{};
            gmtime_r(&now, &utc_time);

            std::ostringstream text;
            text << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
            return text.str();
        }

        /// Format a number with a fixed number of decimals for the console ("n/a" if NaN).
        std::string fixed(double number, int decimals)
        {
            if (std::isnan(number))
            {
                return "n/a";
            }
            std::ostringstream text;
            text << std::fixed << std::setprecision(decimals) << number;
            return text.str();
        }

        /// Write a {"min", "mean", "max"} object for running statistics.
        void
        write_statistics(JsonWriter &json, const std::string &name, const RunningStatistics &values)
        {
            json.key(name);
            json.begin_object();
            json.member("count", values.count());
            json.member("min", values.minimum());
            json.member("mean", values.mean());
            json.member("max", values.maximum());
            json.end_object();
        }

        /// Write the identification and calibration of one camera as a JSON object.
        void write_camera(const sdk::Client &client, JsonWriter &json, const std::string &camera)
        {
            CallErrors errors;
            json.begin_object();
            json.member("name", camera);

            const sdk::Output_GetCameraId id = client.GetCameraId(camera);
            if (errors.check("GetCameraId", id.Result))
            {
                json.member("id", id.CameraId);
            }

            const sdk::Output_GetCameraUserId user_id = client.GetCameraUserId(camera);
            if (errors.check("GetCameraUserId", user_id.Result))
            {
                json.member("user_id", user_id.CameraUserId);
            }

            const sdk::Output_GetCameraType type = client.GetCameraType(camera);
            if (errors.check("GetCameraType", type.Result))
            {
                json.member("type", std::string(type.CameraType));
            }

            const sdk::Output_GetCameraDisplayName display_name =
                client.GetCameraDisplayName(camera);
            if (errors.check("GetCameraDisplayName", display_name.Result))
            {
                json.member("display_name", std::string(display_name.CameraDisplayName));
            }

            const sdk::Output_GetIsVideoCamera is_video = client.GetIsVideoCamera(camera);
            if (errors.check("GetIsVideoCamera", is_video.Result))
            {
                json.member("is_video", is_video.IsVideoCamera);
            }

            const sdk::Output_GetCameraSensorMode sensor_mode = client.GetCameraSensorMode(camera);
            if (errors.check("GetCameraSensorMode", sensor_mode.Result))
            {
                json.member("sensor_mode", std::string(sensor_mode.SensorMode));
            }

            const sdk::Output_GetCameraResolution resolution = client.GetCameraResolution(camera);
            if (errors.check("GetCameraResolution", resolution.Result))
            {
                const unsigned int pixels[2] = {resolution.ResolutionX, resolution.ResolutionY};
                json.member_array("resolution_px", pixels, 2);
            }

            const sdk::Output_GetCameraWindowSize window = client.GetCameraWindowSize(camera);
            if (errors.check("GetCameraWindowSize", window.Result))
            {
                json.key("window_px");
                json.begin_object();
                json.member("start_x", window.WindowStartX);
                json.member("start_y", window.WindowStartY);
                json.member("width", window.WindowWidth);
                json.member("height", window.WindowHeight);
                json.end_object();
            }

            const sdk::Output_GetCameraGlobalTranslation position =
                client.GetCameraGlobalTranslation(camera);
            if (errors.check("GetCameraGlobalTranslation", position.Result))
            {
                json.member_array("position_mm", position.Translation, 3);
            }

            const sdk::Output_GetCameraGlobalRotationQuaternion rotation =
                client.GetCameraGlobalRotationQuaternion(camera);
            if (errors.check("GetCameraGlobalRotationQuaternion", rotation.Result))
            {
                json.member_array("rotation_xyzw", rotation.Rotation, 4);
            }

            const sdk::Output_GetCameraPrincipalPoint principal_point =
                client.GetCameraPrincipalPoint(camera);
            if (errors.check("GetCameraPrincipalPoint", principal_point.Result))
            {
                const double point[2] = {principal_point.PrincipalPointX,
                                         principal_point.PrincipalPointY};
                json.member_array("principal_point", point, 2);
            }

            const sdk::Output_GetCameraFocalLength focal_length =
                client.GetCameraFocalLength(camera);
            if (errors.check("GetCameraFocalLength", focal_length.Result))
            {
                json.member("focal_length", focal_length.FocalLength);
            }

            const sdk::Output_GetCameraLensParameters lens = client.GetCameraLensParameters(camera);
            if (errors.check("GetCameraLensParameters", lens.Result))
            {
                json.member_array("lens_parameters", lens.LensParameters, 3);
            }

            errors.write(json);
            json.end_object();
        }

        /// Write the static description of one segment (tree and static transform).
        void write_segment_structure(const sdk::Client &client,
                                     JsonWriter &json,
                                     const std::string &subject,
                                     const std::string &segment)
        {
            CallErrors errors;
            json.begin_object();
            json.member("name", segment);

            const sdk::Output_GetSegmentParentName parent =
                client.GetSegmentParentName(subject, segment);
            if (errors.check("GetSegmentParentName", parent.Result))
            {
                json.member("parent", std::string(parent.SegmentName));
            }

            json.key("children");
            json.begin_array();
            const sdk::Output_GetSegmentChildCount child_count =
                client.GetSegmentChildCount(subject, segment);
            if (errors.check("GetSegmentChildCount", child_count.Result))
            {
                for (unsigned int index = 0; index < child_count.SegmentCount; ++index)
                {
                    const sdk::Output_GetSegmentChildName child =
                        client.GetSegmentChildName(subject, segment, index);
                    if (errors.check("GetSegmentChildName", child.Result))
                    {
                        json.value(std::string(child.SegmentName));
                    }
                }
            }
            json.end_array();

            const sdk::Output_GetSegmentStaticTranslation translation =
                client.GetSegmentStaticTranslation(subject, segment);
            if (errors.check("GetSegmentStaticTranslation", translation.Result))
            {
                json.member_array("static_translation_mm", translation.Translation, 3);
            }

            const sdk::Output_GetSegmentStaticRotationQuaternion rotation =
                client.GetSegmentStaticRotationQuaternion(subject, segment);
            if (errors.check("GetSegmentStaticRotationQuaternion", rotation.Result))
            {
                json.member_array("static_rotation_xyzw", rotation.Rotation, 4);
            }

            errors.write(json);
            json.end_object();
        }

        /// Write the static description of one subject: root, segment tree and marker names.
        void write_subject_structure_entry(const sdk::Client &client,
                                           JsonWriter &json,
                                           const std::string &subject)
        {
            CallErrors errors;
            json.begin_object();
            json.member("name", subject);

            const sdk::Output_GetSubjectRootSegmentName root =
                client.GetSubjectRootSegmentName(subject);
            if (errors.check("GetSubjectRootSegmentName", root.Result))
            {
                json.member("root_segment", std::string(root.SegmentName));
            }

            json.key("segments");
            json.begin_array();
            const sdk::Output_GetSegmentCount segment_count = client.GetSegmentCount(subject);
            if (errors.check("GetSegmentCount", segment_count.Result))
            {
                for (unsigned int index = 0; index < segment_count.SegmentCount; ++index)
                {
                    const sdk::Output_GetSegmentName segment =
                        client.GetSegmentName(subject, index);
                    if (errors.check("GetSegmentName", segment.Result))
                    {
                        write_segment_structure(client, json, subject, segment.SegmentName);
                    }
                }
            }
            json.end_array();

            json.key("markers");
            json.begin_array();
            const sdk::Output_GetMarkerCount marker_count = client.GetMarkerCount(subject);
            if (errors.check("GetMarkerCount", marker_count.Result))
            {
                for (unsigned int index = 0; index < marker_count.MarkerCount; ++index)
                {
                    const sdk::Output_GetMarkerName marker = client.GetMarkerName(subject, index);
                    if (!errors.check("GetMarkerName", marker.Result))
                    {
                        continue;
                    }

                    const sdk::Output_GetMarkerParentName parent =
                        client.GetMarkerParentName(subject, marker.MarkerName);
                    json.begin_object();
                    json.member("name", std::string(marker.MarkerName));
                    if (errors.check("GetMarkerParentName", parent.Result))
                    {
                        json.member("parent_segment", std::string(parent.SegmentName));
                    }
                    json.end_object();
                }
            }
            json.end_array();

            errors.write(json);
            json.end_object();
        }

        /// Write the global pose of one segment in the current frame.
        /// @return true if the translation or the rotation is occluded.
        bool write_segment_pose(const sdk::Client &client,
                                JsonWriter &json,
                                const std::string &subject,
                                const std::string &segment)
        {
            CallErrors errors;
            bool occluded = false;
            json.begin_object();
            json.member("name", segment);

            const sdk::Output_GetSegmentGlobalTranslation translation =
                client.GetSegmentGlobalTranslation(subject, segment);
            if (errors.check("GetSegmentGlobalTranslation", translation.Result))
            {
                json.member_array("translation_mm", translation.Translation, 3);
                json.member("translation_occluded", translation.Occluded);
                occluded = occluded || translation.Occluded;
            }

            const sdk::Output_GetSegmentGlobalRotationQuaternion rotation =
                client.GetSegmentGlobalRotationQuaternion(subject, segment);
            if (errors.check("GetSegmentGlobalRotationQuaternion", rotation.Result))
            {
                json.member_array("rotation_xyzw", rotation.Rotation, 4);
                json.member("rotation_occluded", rotation.Occluded);
                occluded = occluded || rotation.Occluded;
            }

            errors.write(json);
            json.end_object();
            return occluded;
        }

        /// Write one labeled marker of a subject in the current frame, with the cameras that saw it.
        void write_marker_observation(const sdk::Client &client,
                                      JsonWriter &json,
                                      const std::string &subject,
                                      const std::string &marker)
        {
            CallErrors errors;
            json.begin_object();
            json.member("name", marker);

            const sdk::Output_GetMarkerGlobalTranslation position =
                client.GetMarkerGlobalTranslation(subject, marker);
            if (errors.check("GetMarkerGlobalTranslation", position.Result))
            {
                json.member_array("position_mm", position.Translation, 3);
                json.member("occluded", position.Occluded);
            }

            // Each ray contribution is one camera that saw the marker in this frame
            const sdk::Output_GetMarkerRayContributionCount ray_count =
                client.GetMarkerRayContributionCount(subject, marker);
            if (errors.check("GetMarkerRayContributionCount", ray_count.Result))
            {
                json.key("camera_ids");
                json.begin_array();
                for (unsigned int index = 0; index < ray_count.RayContributionsCount; ++index)
                {
                    const sdk::Output_GetMarkerRayContribution ray =
                        client.GetMarkerRayContribution(subject, marker, index);
                    if (errors.check("GetMarkerRayContribution", ray.Result))
                    {
                        json.value(ray.CameraID);
                    }
                }
                json.end_array();
            }

            errors.write(json);
            json.end_object();
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // CallErrors
    // ---------------------------------------------------------------------------------------------

    bool CallErrors::check(const std::string &call, sdk::Result::Enum result)
    {
        if (result == sdk::Result::Success)
        {
            return true;
        }
        failures_.emplace_back(call, result_name(result));
        return false;
    }

    bool CallErrors::empty() const
    {
        return failures_.empty();
    }

    void CallErrors::write(JsonWriter &writer) const
    {
        if (failures_.empty())
        {
            return;
        }

        // The same call can fail several times (e.g. in a loop): each failure is one element
        writer.key("errors");
        writer.begin_array();
        for (const auto &[call, result] : failures_)
        {
            writer.begin_object();
            writer.member("call", call);
            writer.member("result", result);
            writer.end_object();
        }
        writer.end_array();
    }

    // ---------------------------------------------------------------------------------------------
    // RunningStatistics
    // ---------------------------------------------------------------------------------------------

    void RunningStatistics::add(double sample)
    {
        count_ += 1;
        sum_ += sample;
        minimum_ = std::min(minimum_, sample);
        maximum_ = std::max(maximum_, sample);
    }

    std::uint64_t RunningStatistics::count() const
    {
        return count_;
    }

    double RunningStatistics::minimum() const
    {
        return count_ == 0 ? std::nan("") : minimum_;
    }

    double RunningStatistics::mean() const
    {
        return count_ == 0 ? std::nan("") : sum_ / static_cast<double>(count_);
    }

    double RunningStatistics::maximum() const
    {
        return count_ == 0 ? std::nan("") : maximum_;
    }

    // ---------------------------------------------------------------------------------------------
    // ViconProbe
    // ---------------------------------------------------------------------------------------------

    ViconProbe::ViconProbe(const ProbeOptions &options,
                           std::ostream &console,
                           std::ostream &json_stream)
        : options_(options), console_(console), json_(json_stream)
    {
    }

    ExitCode ViconProbe::run(const std::atomic<bool> &stop_requested)
    {
        json_.begin_object();
        write_probe_information();

        if (!connect())
        {
            json_.end_object();
            return ExitCode::ConnectionFailed;
        }

        enable_streams();

        if (!wait_for_first_frame(stop_requested))
        {
            // Either interrupted by the user or the connection dropped before any frame
            const bool interrupted = stop_requested.load();
            const bool stream_lost = !interrupted;
            write_summary(interrupted, stream_lost);
            json_.end_object();
            client_.Disconnect();
            print_final_summary(interrupted, stream_lost);
            return stream_lost ? ExitCode::StreamLost : ExitCode::Success;
        }

        // The session description comes from the first frame (subjects, cameras, ...)
        write_server_information();
        write_cameras();
        write_devices();
        write_subject_structure();
        print_session_summary();

        // The first frame is already fetched: record it, then fetch the following ones
        json_.key("frames");
        json_.begin_array();
        write_frame();

        bool stream_lost = false;
        while (frames_recorded_ < options_.frame_count && !stop_requested.load())
        {
            const sdk::Output_GetFrame frame = client_.GetFrame();
            if (frame.Result != sdk::Result::Success)
            {
                console_ << "Stream lost: GetFrame returned " << result_name(frame.Result) << "\n";
                stream_lost = true;
                break;
            }
            write_frame();
        }
        json_.end_array();

        const bool interrupted = !stream_lost && frames_recorded_ < options_.frame_count;
        write_summary(interrupted, stream_lost);
        json_.end_object();

        client_.Disconnect();
        print_final_summary(interrupted, stream_lost);
        return stream_lost ? ExitCode::StreamLost : ExitCode::Success;
    }

    void ViconProbe::write_probe_information()
    {
        json_.key("probe");
        json_.begin_object();
        json_.member("tool", "vicon_probe");
        json_.member("started_utc", utc_timestamp());
        json_.member("sdk_version", sdk_version(client_));
        json_.member("host", options_.host);
        json_.member("requested_frames", options_.frame_count);
        json_.member("include_centroids", options_.include_centroids);
        json_.key("units");
        json_.begin_object();
        json_.member("translation", "mm (as streamed by the SDK)");
        json_.member("rotation", "quaternion x, y, z, w (SDK order)");
        json_.member("latency", "s");
        json_.end_object();
        json_.end_object();
    }

    bool ViconProbe::connect()
    {
        console_ << "Connecting to " << options_.host << " ..." << std::endl;

        CallErrors errors;
        json_.key("connection");
        json_.begin_object();
        json_.member("host", options_.host);

        if (options_.connection_timeout_ms.has_value())
        {
            const unsigned int timeout_ms = options_.connection_timeout_ms.value();
            json_.member("timeout_ms", timeout_ms);
            errors.check("SetConnectionTimeout", client_.SetConnectionTimeout(timeout_ms).Result);
        }

        const auto start_time = std::chrono::steady_clock::now();
        const sdk::Output_Connect output = client_.Connect(options_.host);
        const std::chrono::duration<double> duration =
            std::chrono::steady_clock::now() - start_time;

        json_.member("result", result_name(output.Result));
        json_.member("duration_s", duration.count());
        errors.write(json_);
        json_.end_object();

        if (output.Result != sdk::Result::Success)
        {
            console_ << "Connection to " << options_.host
                     << " failed: " << result_name(output.Result) << " (after "
                     << fixed(duration.count(), 2) << " s)" << std::endl;
            return false;
        }

        console_ << "Connected in " << fixed(duration.count(), 2) << " s (SDK "
                 << sdk_version(client_) << ")" << std::endl;
        return true;
    }

    void ViconProbe::enable_streams()
    {
        CallErrors errors;
        json_.key("stream");
        json_.begin_object();

        // Same mode as the vicon_client node, so the probe sees what the node sees
        json_.member("mode", "ClientPull");
        errors.check("SetStreamMode", client_.SetStreamMode(sdk::StreamMode::ClientPull).Result);

        json_.key("enabled");
        json_.begin_object();
        json_.member("segment_data", result_name(client_.EnableSegmentData().Result));
        json_.member("marker_data", result_name(client_.EnableMarkerData().Result));
        json_.member("unlabeled_marker_data",
                     result_name(client_.EnableUnlabeledMarkerData().Result));
        json_.member("marker_ray_data", result_name(client_.EnableMarkerRayData().Result));
        json_.member("device_data", result_name(client_.EnableDeviceData().Result));
        json_.member("camera_calibration_data",
                     result_name(client_.EnableCameraCalibrationData().Result));
        if (options_.include_centroids)
        {
            json_.member("centroid_data", result_name(client_.EnableCentroidData().Result));
        }
        json_.end_object();

        errors.write(json_);
        json_.end_object();
    }

    bool ViconProbe::wait_for_first_frame(const std::atomic<bool> &stop_requested)
    {
        console_ << "Waiting for the first frame ..." << std::endl;

        while (!stop_requested.load())
        {
            const sdk::Output_GetFrame frame = client_.GetFrame();
            if (frame.Result == sdk::Result::Success)
            {
                return true;
            }
            if (frame.Result != sdk::Result::NoFrame)
            {
                console_ << "No frame received: GetFrame returned " << result_name(frame.Result)
                         << std::endl;
                return false;
            }

            // NoFrame: the server has not sent a frame yet
            std::this_thread::sleep_for(kNoFrameRetryDelay);
        }
        return false;
    }

    void ViconProbe::write_server_information()
    {
        CallErrors errors;
        json_.key("server");
        json_.begin_object();

        const sdk::Output_GetAxisMapping axes = client_.GetAxisMapping();
        json_.key("axis_mapping");
        json_.begin_object();
        json_.member("x", direction_name(axes.XAxis));
        json_.member("y", direction_name(axes.YAxis));
        json_.member("z", direction_name(axes.ZAxis));
        json_.end_object();

        const sdk::Output_GetServerOrientation orientation = client_.GetServerOrientation();
        if (errors.check("GetServerOrientation", orientation.Result))
        {
            json_.member("orientation", server_orientation_name(orientation.Orientation));
        }

        const sdk::Output_GetFrameRate frame_rate = client_.GetFrameRate();
        if (errors.check("GetFrameRate", frame_rate.Result))
        {
            json_.member("frame_rate_hz", frame_rate.FrameRateHz);
        }

        // Named frame rates of the system (e.g. of the cameras and of other devices)
        json_.key("frame_rates_hz");
        json_.begin_object();
        const sdk::Output_GetFrameRateCount rate_count = client_.GetFrameRateCount();
        if (errors.check("GetFrameRateCount", rate_count.Result))
        {
            for (unsigned int index = 0; index < rate_count.Count; ++index)
            {
                const sdk::Output_GetFrameRateName name = client_.GetFrameRateName(index);
                if (!errors.check("GetFrameRateName", name.Result))
                {
                    continue;
                }
                const sdk::Output_GetFrameRateValue rate = client_.GetFrameRateValue(name.Name);
                if (errors.check("GetFrameRateValue", rate.Result))
                {
                    json_.member(name.Name, rate.Value);
                }
            }
        }
        json_.end_object();

        errors.write(json_);
        json_.end_object();
    }

    void ViconProbe::write_cameras()
    {
        json_.key("cameras");
        json_.begin_array();

        const sdk::Output_GetCameraCount camera_count = client_.GetCameraCount();
        if (session_errors_.check("GetCameraCount", camera_count.Result))
        {
            for (unsigned int index = 0; index < camera_count.CameraCount; ++index)
            {
                const sdk::Output_GetCameraName camera = client_.GetCameraName(index);
                if (session_errors_.check("GetCameraName", camera.Result))
                {
                    write_camera(client_, json_, camera.CameraName);
                }
            }
        }
        json_.end_array();
    }

    void ViconProbe::write_devices()
    {
        json_.key("devices");
        json_.begin_array();

        const sdk::Output_GetDeviceCount device_count = client_.GetDeviceCount();
        if (session_errors_.check("GetDeviceCount", device_count.Result))
        {
            for (unsigned int index = 0; index < device_count.DeviceCount; ++index)
            {
                const sdk::Output_GetDeviceName device = client_.GetDeviceName(index);
                if (!session_errors_.check("GetDeviceName", device.Result))
                {
                    continue;
                }
                json_.begin_object();
                json_.member("name", std::string(device.DeviceName));
                json_.member("type", device_type_name(device.DeviceType));
                json_.end_object();
            }
        }
        json_.end_array();
    }

    void ViconProbe::write_subject_structure()
    {
        json_.key("subjects");
        json_.begin_array();

        const sdk::Output_GetSubjectCount subject_count = client_.GetSubjectCount();
        if (session_errors_.check("GetSubjectCount", subject_count.Result))
        {
            for (unsigned int index = 0; index < subject_count.SubjectCount; ++index)
            {
                const sdk::Output_GetSubjectName subject = client_.GetSubjectName(index);
                if (session_errors_.check("GetSubjectName", subject.Result))
                {
                    write_subject_structure_entry(client_, json_, subject.SubjectName);
                }
            }
        }
        json_.end_array();
    }

    void ViconProbe::write_frame()
    {
        // Taken first, as close as possible to the arrival of the frame
        const double receive_time_s = unix_time_seconds();
        frames_recorded_ += 1;

        CallErrors errors;
        std::string console_line;
        json_.begin_object();

        const sdk::Output_GetFrameNumber frame_number = client_.GetFrameNumber();
        if (errors.check("GetFrameNumber", frame_number.Result))
        {
            const std::uint64_t missed = frame_gaps_.record(frame_number.FrameNumber);
            json_.member("frame_number", frame_number.FrameNumber);
            json_.member("frames_missed_before", missed);
            console_line = "frame " + std::to_string(frame_number.FrameNumber);
            if (missed > 0)
            {
                console_line += " (" + std::to_string(missed) + " missed)";
            }
        }
        else
        {
            console_line = "frame ?";
        }

        const sdk::Output_GetHardwareFrameNumber hardware_frame = client_.GetHardwareFrameNumber();
        if (errors.check("GetHardwareFrameNumber", hardware_frame.Result))
        {
            json_.member("hardware_frame_number", hardware_frame.HardwareFrameNumber);
        }

        json_.member("receive_time_unix_s", receive_time_s);

        const sdk::Output_GetFrameRate frame_rate = client_.GetFrameRate();
        if (errors.check("GetFrameRate", frame_rate.Result))
        {
            json_.member("frame_rate_hz", frame_rate.FrameRateHz);
        }

        // Total latency: time between the camera exposure and the frame being available here
        const sdk::Output_GetLatencyTotal latency = client_.GetLatencyTotal();
        if (errors.check("GetLatencyTotal", latency.Result))
        {
            json_.member("latency_total_s", latency.Total);
            latency_s_.add(latency.Total);
            console_line += "  latency " + fixed(latency.Total * 1000.0, 1) + " ms";
        }

        // Latency of each stage of the processing pipeline
        json_.key("latency_samples_s");
        json_.begin_object();
        const sdk::Output_GetLatencySampleCount sample_count = client_.GetLatencySampleCount();
        if (errors.check("GetLatencySampleCount", sample_count.Result))
        {
            for (unsigned int index = 0; index < sample_count.Count; ++index)
            {
                const sdk::Output_GetLatencySampleName name = client_.GetLatencySampleName(index);
                if (!errors.check("GetLatencySampleName", name.Result))
                {
                    continue;
                }
                const sdk::Output_GetLatencySampleValue sample =
                    client_.GetLatencySampleValue(name.Name);
                if (errors.check("GetLatencySampleValue", sample.Result))
                {
                    json_.member(name.Name, sample.Value);
                }
            }
        }
        json_.end_object();

        const sdk::Output_GetTimecode timecode = client_.GetTimecode();
        if (errors.check("GetTimecode", timecode.Result))
        {
            json_.key("timecode");
            json_.begin_object();
            json_.member("standard", timecode_standard_name(timecode.Standard));
            json_.member("hours", timecode.Hours);
            json_.member("minutes", timecode.Minutes);
            json_.member("seconds", timecode.Seconds);
            json_.member("frames", timecode.Frames);
            json_.member("sub_frame", timecode.SubFrame);
            json_.member("sub_frames_per_frame", timecode.SubFramesPerFrame);
            json_.member("field_flag", timecode.FieldFlag);
            json_.member("user_bits", timecode.UserBits);
            json_.end_object();
        }

        json_.key("subjects");
        json_.begin_array();
        const sdk::Output_GetSubjectCount subject_count = client_.GetSubjectCount();
        if (errors.check("GetSubjectCount", subject_count.Result))
        {
            for (unsigned int index = 0; index < subject_count.SubjectCount; ++index)
            {
                const sdk::Output_GetSubjectName subject = client_.GetSubjectName(index);
                if (errors.check("GetSubjectName", subject.Result))
                {
                    write_subject_frame(subject.SubjectName, console_line);
                }
            }
        }
        json_.end_array();

        write_frame_markers(errors);
        if (options_.include_centroids)
        {
            write_frame_centroids(errors);
        }

        errors.write(json_);
        json_.end_object();

        console_ << console_line << std::endl;
    }

    void ViconProbe::write_subject_frame(const std::string &subject_name, std::string &console_line)
    {
        SubjectStatistics &statistics = subject_statistics_[subject_name];
        statistics.frames_seen += 1;

        CallErrors errors;
        json_.begin_object();
        json_.member("name", subject_name);
        console_line += "  | " + subject_name;

        const sdk::Output_GetObjectQuality quality = client_.GetObjectQuality(subject_name);
        if (errors.check("GetObjectQuality", quality.Result))
        {
            json_.member("quality", quality.Quality);
            statistics.quality.add(quality.Quality);
            console_line += " q=" + fixed(quality.Quality, 3);
        }
        else
        {
            console_line += " q=n/a";
        }

        bool any_occluded = false;
        json_.key("segments");
        json_.begin_array();
        const sdk::Output_GetSegmentCount segment_count = client_.GetSegmentCount(subject_name);
        if (errors.check("GetSegmentCount", segment_count.Result))
        {
            for (unsigned int index = 0; index < segment_count.SegmentCount; ++index)
            {
                const sdk::Output_GetSegmentName segment =
                    client_.GetSegmentName(subject_name, index);
                if (!errors.check("GetSegmentName", segment.Result))
                {
                    continue;
                }
                const bool occluded =
                    write_segment_pose(client_, json_, subject_name, segment.SegmentName);
                any_occluded = any_occluded || occluded;
            }
        }
        json_.end_array();

        json_.key("markers");
        json_.begin_array();
        const sdk::Output_GetMarkerCount marker_count = client_.GetMarkerCount(subject_name);
        if (errors.check("GetMarkerCount", marker_count.Result))
        {
            for (unsigned int index = 0; index < marker_count.MarkerCount; ++index)
            {
                const sdk::Output_GetMarkerName marker = client_.GetMarkerName(subject_name, index);
                if (errors.check("GetMarkerName", marker.Result))
                {
                    write_marker_observation(client_, json_, subject_name, marker.MarkerName);
                }
            }
        }
        json_.end_array();

        errors.write(json_);
        json_.end_object();

        if (any_occluded)
        {
            statistics.frames_with_occlusion += 1;
            console_line += " OCCLUDED";
        }
    }

    void ViconProbe::write_frame_markers(CallErrors &frame_errors)
    {
        // Markers assigned to a subject, as reconstructed points
        json_.key("labeled_markers");
        json_.begin_array();
        const sdk::Output_GetLabeledMarkerCount labeled_count = client_.GetLabeledMarkerCount();
        if (frame_errors.check("GetLabeledMarkerCount", labeled_count.Result))
        {
            for (unsigned int index = 0; index < labeled_count.MarkerCount; ++index)
            {
                const sdk::Output_GetLabeledMarkerGlobalTranslation marker =
                    client_.GetLabeledMarkerGlobalTranslation(index);
                if (!frame_errors.check("GetLabeledMarkerGlobalTranslation", marker.Result))
                {
                    continue;
                }
                json_.begin_object();
                json_.member("id", marker.MarkerID);
                json_.member_array("position_mm", marker.Translation, 3);
                json_.end_object();
            }
        }
        json_.end_array();

        // Reconstructed points not assigned to any subject (stray reflections, ghosts, ...)
        json_.key("unlabeled_markers");
        json_.begin_array();
        const sdk::Output_GetUnlabeledMarkerCount unlabeled_count =
            client_.GetUnlabeledMarkerCount();
        if (frame_errors.check("GetUnlabeledMarkerCount", unlabeled_count.Result))
        {
            for (unsigned int index = 0; index < unlabeled_count.MarkerCount; ++index)
            {
                const sdk::Output_GetUnlabeledMarkerGlobalTranslation marker =
                    client_.GetUnlabeledMarkerGlobalTranslation(index);
                if (!frame_errors.check("GetUnlabeledMarkerGlobalTranslation", marker.Result))
                {
                    continue;
                }
                json_.begin_object();
                json_.member("id", marker.MarkerID);
                json_.member_array("position_mm", marker.Translation, 3);
                json_.end_object();
            }
        }
        json_.end_array();
    }

    void ViconProbe::write_frame_centroids(CallErrors &frame_errors)
    {
        json_.key("centroids");
        json_.begin_array();

        const sdk::Output_GetCameraCount camera_count = client_.GetCameraCount();
        if (!frame_errors.check("GetCameraCount", camera_count.Result))
        {
            json_.end_array();
            return;
        }

        for (unsigned int camera_index = 0; camera_index < camera_count.CameraCount; ++camera_index)
        {
            const sdk::Output_GetCameraName camera_output = client_.GetCameraName(camera_index);
            if (!frame_errors.check("GetCameraName", camera_output.Result))
            {
                continue;
            }
            const std::string camera = camera_output.CameraName;

            CallErrors errors;
            json_.begin_object();
            json_.member("camera", camera);
            json_.key("centroids");
            json_.begin_array();

            const sdk::Output_GetCentroidCount centroid_count = client_.GetCentroidCount(camera);
            if (errors.check("GetCentroidCount", centroid_count.Result))
            {
                for (unsigned int index = 0; index < centroid_count.CentroidCount; ++index)
                {
                    const sdk::Output_GetCentroidPosition position =
                        client_.GetCentroidPosition(camera, index);
                    const sdk::Output_GetCentroidWeight weight =
                        client_.GetCentroidWeight(camera, index);
                    if (!errors.check("GetCentroidPosition", position.Result))
                    {
                        continue;
                    }
                    json_.begin_object();
                    json_.member_array("position_px", position.CentroidPosition, 2);
                    json_.member("radius_px", position.Radius);
                    if (errors.check("GetCentroidWeight", weight.Result))
                    {
                        json_.member("weight", weight.Weight);
                    }
                    json_.end_object();
                }
            }
            json_.end_array();
            errors.write(json_);
            json_.end_object();
        }
        json_.end_array();
    }

    void ViconProbe::write_summary(bool interrupted, bool stream_lost)
    {
        json_.key("summary");
        json_.begin_object();
        json_.member("frames_recorded", frames_recorded_);
        json_.member("frames_missed", frame_gaps_.missed());
        json_.member("frame_number_discontinuities", frame_gaps_.discontinuities());
        json_.member("interrupted", interrupted);
        json_.member("stream_lost", stream_lost);
        write_statistics(json_, "latency_total_s", latency_s_);

        json_.key("subjects");
        json_.begin_object();
        for (const auto &[name, statistics] : subject_statistics_)
        {
            json_.key(name);
            json_.begin_object();
            json_.member("frames_seen", statistics.frames_seen);
            json_.member("frames_with_occlusion", statistics.frames_with_occlusion);
            write_statistics(json_, "quality", statistics.quality);
            json_.end_object();
        }
        json_.end_object();

        session_errors_.write(json_);
        json_.end_object();
    }

    void ViconProbe::print_session_summary()
    {
        console_ << "\nSession\n";

        const sdk::Output_GetFrameRate frame_rate = client_.GetFrameRate();
        console_ << "  frame rate:    "
                 << (frame_rate.Result == sdk::Result::Success
                         ? fixed(frame_rate.FrameRateHz, 1) + " Hz"
                         : result_name(frame_rate.Result))
                 << "\n";

        const sdk::Output_GetAxisMapping axes = client_.GetAxisMapping();
        console_ << "  axis mapping:  x=" << direction_name(axes.XAxis)
                 << " y=" << direction_name(axes.YAxis) << " z=" << direction_name(axes.ZAxis)
                 << "\n";

        const sdk::Output_GetCameraCount camera_count = client_.GetCameraCount();
        console_ << "  cameras:       "
                 << (camera_count.Result == sdk::Result::Success
                         ? std::to_string(camera_count.CameraCount)
                         : result_name(camera_count.Result))
                 << "\n";

        const sdk::Output_GetSubjectCount subject_count = client_.GetSubjectCount();
        if (subject_count.Result != sdk::Result::Success)
        {
            console_ << "  subjects:      " << result_name(subject_count.Result) << "\n\n";
            return;
        }

        console_ << "  subjects:      " << subject_count.SubjectCount << "\n";
        for (unsigned int index = 0; index < subject_count.SubjectCount; ++index)
        {
            const std::string subject = client_.GetSubjectName(index).SubjectName;
            const sdk::Output_GetSegmentCount segments = client_.GetSegmentCount(subject);
            const sdk::Output_GetMarkerCount markers = client_.GetMarkerCount(subject);
            console_ << "    " << subject << ": " << segments.SegmentCount << " segment(s), "
                     << markers.MarkerCount << " marker(s)\n";
        }
        console_ << std::endl;
    }

    void ViconProbe::print_final_summary(bool interrupted, bool stream_lost)
    {
        console_ << "\nRecorded " << frames_recorded_ << " frame(s): " << frame_gaps_.missed()
                 << " missed, " << frame_gaps_.discontinuities() << " frame-number discontinuities";
        if (interrupted)
        {
            console_ << " (interrupted)";
        }
        if (stream_lost)
        {
            console_ << " (stream lost)";
        }
        console_ << "\n";

        console_ << "Latency total: min " << fixed(latency_s_.minimum() * 1000.0, 1) << " ms, mean "
                 << fixed(latency_s_.mean() * 1000.0, 1) << " ms, max "
                 << fixed(latency_s_.maximum() * 1000.0, 1) << " ms\n";

        for (const auto &[name, statistics] : subject_statistics_)
        {
            console_ << "  " << name << ": seen in " << statistics.frames_seen
                     << " frame(s), occluded in " << statistics.frames_with_occlusion
                     << ", quality min " << fixed(statistics.quality.minimum(), 3) << " mean "
                     << fixed(statistics.quality.mean(), 3) << "\n";
        }
        console_ << std::flush;
    }

    std::string sdk_version(const sdk::Client &client)
    {
        const sdk::Output_GetVersion version = client.GetVersion();
        return std::to_string(version.Major) + "." + std::to_string(version.Minor) + "." +
               std::to_string(version.Point) + "." + std::to_string(version.Revision);
    }
} // namespace vicon_probe

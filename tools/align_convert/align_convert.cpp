// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#include <iostream>

#include "librealsense2/rs.hpp"

#include "tclap/CmdLine.h"

#include "converters/converter-csv.hpp"
#include "converters/converter-png.hpp"
#include "converters/converter-raw.hpp"
#include "converters/converter-ply.hpp"
#include "converters/converter-bin.hpp"


using namespace std;
using namespace TCLAP;


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

// void show_stream_intrinsics(rs2::device device, rs2::sensor sensor)
// {
//     // Each stream has its own intrinsic information, first choose a stream, then display its intrinsics
//     rs2::stream_profile selected_profile = how_to::choose_a_streaming_profile(sensor);
//     how_to::get_field_of_view(selected_profile);
// }

int main(int argc, char** argv) try
{
    rs2::log_to_file(RS2_LOG_SEVERITY_WARN);

    // Parse command line arguments
    CmdLine cmd("librealsense rs-convert tool", ' ');
    ValueArg<string> inputFilename("i", "input", "ROS-bag filename", true, "", "ros-bag-file");
    ValueArg<string> outputFilenamePng("p", "output-png", "output PNG file(s) path", false, "", "png-path");
    ValueArg<string> outputFilenameCsv("v", "output-csv", "output CSV (depth matrix) file(s) path", false, "", "csv-path");
    ValueArg<string> outputFilenameRaw("r", "output-raw", "output RAW file(s) path", false, "", "raw-path");
    ValueArg<string> outputFilenamePly("l", "output-ply", "output PLY file(s) path", false, "", "ply-path");
    ValueArg<string> outputFilenameBin("b", "output-bin", "output BIN (depth matrix) file(s) path", false, "", "bin-path");
    SwitchArg switchDepth("d", "depth", "convert depth frames (default - all supported)", false);
    SwitchArg switchColor("c", "color", "convert color frames (default - all supported)", false);

    cmd.add(inputFilename);
    cmd.add(outputFilenamePng);
    cmd.add(outputFilenameCsv);
    cmd.add(outputFilenameRaw);
    cmd.add(outputFilenamePly);
    cmd.add(outputFilenameBin);
    cmd.add(switchDepth);
    cmd.add(switchColor);
    cmd.parse(argc, argv);

    vector<shared_ptr<rs2::tools::converter::converter_base>> converters;

    rs2_stream streamType = switchDepth.isSet() ? rs2_stream::RS2_STREAM_DEPTH
        : switchColor.isSet() ? rs2_stream::RS2_STREAM_COLOR
        : rs2_stream::RS2_STREAM_ANY;

    if (outputFilenameCsv.isSet()) {
        converters.push_back(
            make_shared<rs2::tools::converter::converter_csv>(
                outputFilenameCsv.getValue()
                , streamType));
    }

    if (outputFilenamePng.isSet()) {
        converters.push_back(
            make_shared<rs2::tools::converter::converter_png>(
                outputFilenamePng.getValue()
                , streamType));
    }

    if (outputFilenameRaw.isSet()) {
        converters.push_back(
            make_shared<rs2::tools::converter::converter_raw>(
                outputFilenameRaw.getValue()
                , streamType));
    }

    if (outputFilenamePly.isSet()) {
        converters.push_back(
            make_shared<rs2::tools::converter::converter_ply>(
                outputFilenamePly.getValue()));
    }

    if (outputFilenameBin.isSet()) {
        converters.push_back(
            make_shared<rs2::tools::converter::converter_bin>(
                outputFilenameBin.getValue()));
    }

    if (converters.empty()) {
        throw runtime_error("output not defined");
    }

    auto pipe = make_shared<rs2::pipeline>();
    rs2::config cfg;
    cfg.enable_device_from_file(inputFilename.getValue());
    rs2::pipeline_profile profile = pipe->start(cfg);

    auto device = pipe->get_active_profile().get_device();
    rs2::playback playback = device.as<rs2::playback>();
    playback.set_real_time(false);

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(profile.get_device());

    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);

    // auto sync = pipe.create_syncer();

    auto duration = playback.get_duration();
    int progress = 0;
    auto frameNumber = 0ULL;

    while (true) {
        auto frameset = pipe->wait_for_frames();

        int posP = static_cast<int>(playback.get_position() * 100. / duration.count());

        if (posP > progress) {
            progress = posP;
            cout << posP << "%" << "\r" << flush;
        }

        if (frameset[0].get_frame_number() < frameNumber) {
            break;
        }

        frameNumber = frameset[0].get_frame_number();


        // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
        // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
        //  after the call to wait_for_frames();
        if (profile_changed(pipe->get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe->get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        // save images
        for_each(converters.begin(), converters.end(),
            [&processed] (shared_ptr<rs2::tools::converter::converter_base>& converter) {
                converter->convert(processed);
            });

        for_each(converters.begin(), converters.end(),
            [] (shared_ptr<rs2::tools::converter::converter_base>& converter) {
                converter->wait();
            });

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }

    }

    auto const i = pipe->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    // cout << endl;
    //
    // for_each(converters.begin(), converters.end(),
    //     [] (shared_ptr<rs2::tools::converter::converter_base>& converter) {
    //         cout << converter->get_statistics() << endl;
    //     });

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function()
        << "(" << e.get_failed_args() << "):\n    " << e.what() << endl;

    return EXIT_FAILURE;
}
catch (const exception & e)
{
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
catch (...)
{
    cerr << "some error" << endl;
    return EXIT_FAILURE;
}

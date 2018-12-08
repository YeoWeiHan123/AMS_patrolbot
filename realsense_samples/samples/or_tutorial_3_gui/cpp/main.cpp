// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <thread>
#include <iostream>

#include "or_utils.hpp"
#include "or_console_display.hpp"
#include "or_gui_display.hpp"
#include "version.h"
using namespace std;
using namespace rs::core;
using namespace rs::object_recognition;

// Version number of the samples
extern constexpr auto rs_sample_version = concat("VERSION: ",RS_SAMPLE_VERSION_STR);

std::vector<std::string> objects_names;
image_info colorInfo, depthInfo;
bool is_localize=true;
bool is_tracking=false;
// Doing the OR processing for a frame can take longer than the frame interval, so we
// keep track of whether or not we are still processing the last frame.
bool is_or_processing_frame = false;
bool is_exit = false;

unique_ptr<console_display::or_console_display>    console_view;
unique_ptr<gui_display::or_gui_display> gui_view;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        rs::object_recognition::or_configuration_interface* ptr = reinterpret_cast<rs::object_recognition::or_configuration_interface*>(param);
        is_localize= !is_localize;
        is_tracking = !is_tracking;
        if (!is_tracking && is_localize)
        {
            //change mode to localization
            ptr->set_recognition_mode(rs::object_recognition::recognition_mode::LOCALIZATION);
            //set the localization mechnizm to use CNN
            ptr->set_localization_mechanism(rs::object_recognition::localization_mechanism::CNN);
            //ignore all objects under 0.7 probabilty (confidence)
            ptr->set_recognition_confidence(0.7);
            //enable GPU computing
            ptr->set_compute_engine(rs::object_recognition::compute_engine::GPU);
            objects_names.clear();

            rs::core::status st = ptr->apply_changes();
            if (st != rs::core::status_no_error)
                exit(1);
        }
    }
}

// After the localization has done we track the found objects
void setTracking(const image_info& colorInfo, or_configuration_interface* or_configuration,
                 const localization_data* localization_data, const int array_size)
{
    // Change mode to tracking
    or_configuration->set_recognition_mode(rs::object_recognition::recognition_mode::TRACKING);

    // Prepare rois for tracking
    rs::core::rect* trackingRois = new rs::core::rect[array_size];
    for (int i = 0; i< array_size; i++)
    {

        rs::core::rect roi= {localization_data[i].roi.x,localization_data[i].roi.y,
                             localization_data[i].roi.width,localization_data[i].roi.height
                            };
        trackingRois[i] =roi;
    }

    // Set the rois
    or_configuration->set_tracking_rois(trackingRois, array_size);
    delete[] trackingRois;
    status st = or_configuration->apply_changes();
    if (st != rs::core::status_no_error)
        exit(1);

    // Set the state accordingly
    is_localize = false;
    is_tracking = true;
}

// Run object tracking and sending result to view
void run_object_tracking(rs::core::correlated_sample_set* or_sample_set, or_video_module_impl* impl, or_data_interface* or_data,
                         or_configuration_interface* or_configuration)
{
    rs::core::status st;

    // Declare data structure and size for results
    localization_data* localization_data = nullptr;
    rs::object_recognition::tracking_data* tracking_data = nullptr;
    int array_size=0;


    gui_view->set_color_image((*or_sample_set)[rs::core::stream_type::color]);

    if (is_localize || is_tracking)
    {

        // Run object localization or tracking processing
        st = impl->process_sample_set(*or_sample_set);
        // Recycle sample set after processing complete
        if((*or_sample_set)[stream_type::color])
            (*or_sample_set)[stream_type::color]->release();
        if((*or_sample_set)[stream_type::depth])
            (*or_sample_set)[stream_type::depth]->release();

        (*or_sample_set)[stream_type::color] = nullptr;
        (*or_sample_set)[stream_type::depth] = nullptr;

        if (st != rs::core::status_no_error)
        {
            is_or_processing_frame = false;
            return;
        }

        if (is_localize)
        {
            // Retrieve localization data from the or_data object
            st = or_data->query_localization_result(&localization_data, array_size);
            if (st != rs::core::status_no_error)
            {
                is_or_processing_frame = false;
                return;
            }
        }
        else
        {
            // Retrieve tracking data from the or_data object
            st = or_data->query_tracking_result(&tracking_data, array_size);
            if (st != rs::core::status_no_error)
            {
                is_or_processing_frame = false;
                return;
            }
        }

        // display the top bounding boxes with object name and probablity

        gui_view->draw_results(localization_data, tracking_data, or_configuration, array_size, is_localize,
                               objects_names);

        if (is_localize)
        {
            if (localization_data && array_size != 0)
            {
                console_view->on_object_tracking_data(localization_data, tracking_data,
                                                      or_configuration, array_size, is_localize, objects_names);
                // After localization has done we want to track the found objects.
                setTracking(colorInfo, or_configuration, localization_data, array_size);

            }
        }
        else
        {
            // Display the top bounding boxes with object name and probability
            if (tracking_data)
            {
                console_view->on_object_tracking_data(localization_data, tracking_data,
                                                      or_configuration, array_size, is_localize, objects_names);
            }
        }

    }

    is_or_processing_frame = false;
}

int main(int argc,char* argv[])
{
    rs::core::status st;
    or_utils or_utils;

    cout  << "Initializing, one moment please ..." << endl << endl;

    console_view = move(console_display::make_console_or_display());
    gui_view = move(gui_display::make_gui_or_display());

    or_video_module_impl impl;
    or_data_interface* or_data = nullptr;
    or_configuration_interface* or_configuration = nullptr;

    // Start the camera
    or_utils.init_camera(colorInfo, depthInfo, impl, &or_data, &or_configuration, true);

    // Change mode to localization
    or_configuration->set_recognition_mode(recognition_mode::LOCALIZATION);
    // Set the localization mechanism to use CNN
    or_configuration->set_localization_mechanism(localization_mechanism::CNN);
    // Ignore all objects under 0.7 probability (confidence)
    or_configuration->set_recognition_confidence(0.7);
    // Enable GPU computing
    or_configuration->set_compute_engine(rs::object_recognition::compute_engine::GPU);
    //Enable 3d location
    or_configuration->enable_object_center_estimation(true);


    st = or_configuration->apply_changes();
    if (st != rs::core::status_no_error)
        return 1;

    gui_view->initialize(colorInfo, "Localization + tracking");
    cv::setMouseCallback(gui_view->get_win_name(),mouseHandler,or_configuration);

    cout << endl << "-------- Press Esc key to exit --------" << endl << endl;


    while (!(is_exit = or_utils.user_request_exit()))
    {
        // Get the sample set from the camera
        rs::core::correlated_sample_set* sample_set = or_utils.get_sample_set(colorInfo,depthInfo);

        if(!is_or_processing_frame)
        {
            is_or_processing_frame = true;

            // Increase image reference to hold for library processing
            (*sample_set)[rs::core::stream_type::color]->add_ref();
            (*sample_set)[rs::core::stream_type::depth]->add_ref();         

            run_object_tracking(sample_set, &impl, or_data, or_configuration);

        }

        gui_view->show_results();

    }

    // Close the camera
    or_utils.stop_camera();
    cout << "-------- Stopping --------" << endl;

    return 0;
}

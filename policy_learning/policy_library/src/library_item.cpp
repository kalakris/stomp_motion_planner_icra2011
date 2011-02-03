/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Peter Pastor */

// system includes
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <errno.h>

// ros includes
#include <ros/ros.h>

// local includes
#include <policy_library/library_item.h>
#include <policy_library/library_io.h>

namespace library
{

LibraryItem::LibraryItem() :
    initialized_(false), item_id_(-1)
{
}

bool LibraryItem::initializeBase(const std::string library_directory_name, const int item_id, const std::string item_name, const bool is_new_dmp)
{

    if (library_directory_name.empty())
    {
        ROS_ERROR("Library directory name is empty.");
        initialized_ = false;
        return initialized_;
    }

    if (item_id < 0)
    {
        ROS_ERROR("Item id is %i and therefore not valid.", item_id);
        initialized_ = false;
        return initialized_;
    }

    initial_item_name_.assign(item_name);

    library_directory_name_.assign(library_directory_name);
    item_id_ = item_id;

    std::stringstream ss;
    ss << item_id_;
    item_name_.assign(library_directory_name_ + item_name + ss.str());

    initialized_ = true;
    return initialized_;
}

}

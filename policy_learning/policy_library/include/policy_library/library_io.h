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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

 \file    dmp_library_io.h

 \author  Peter Pastor
 \date    Jun 9, 2010

 **********************************************************************/

#ifndef DMP_LIBRARY_IO_H_
#define DMP_LIBRARY_IO_H_

// system includes
#include <string>
#include <map>

// ros includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// local includes
#include <policy_library/LibraryIndex.h>

namespace library
{

static const char* library_index_file_name = "library_index.bag";
static const char* library_index_topic_name = "library_index";

struct LibraryIndexItem
{
    std::string item_description_;
    std::string class_name_;
};

class LibraryIO
{

public:

    LibraryIO();
    ~LibraryIO();

    /*! Adds the item_id and description to the library_index_file will remove the previous item_directory (is it existed),
     *  create a new one, and replace the item in the library.
     *
     * @param library_directory_name
     * @param item_id
     * @param item_name
     * @param is_new_dmp
     * @param description
     * @return
     */
    // TODO: this function needs to be fixed if ever needed
    //static bool addToLibraryIndex(const std::string library_directory_name, const int item_id, const std::string item_description =
    //        std::string("no description added yet"), const bool force_replacing_flag = false);

    /*! Creates the necessary directory. If the directory is already existent, it will remove it and re-recreate it.
     *
     * @param library_directory_name
     * @param item_id
     * @return
     */
    static bool createLibraryEntry(const std::string library_directory_name, const std::string item_name);

    /*! This function will load the library_index_file and return all (item_id, description) touple in the parameter item_id_description map.
     *
     * @param library_directory_name
     * @param item_id_description_map
     * @return
     */
    static bool loadLibraryIndex(const std::string library_directory_name, std::map<int, LibraryIndexItem> &item_id_description_map);

    /**
     * Saves the library_index map into the library index file (overwriting the existing file, if any)
     * @param library_directory_name
     * @param item_id_description_map
     * @return
     */
    static bool saveLibraryIndex(const std::string library_directory_name, const std::map<int, LibraryIndexItem> &library_index);

    /*! Adds the item_ids to the library index. If force_replacing_flag is set, the item will be replaced, else it will be only added if the item is not
     *  already contained.
     *
     * @param library_directory_name
     * @param ids
     * @return
     */
    static bool updateLibraryIndex(const std::string library_directory_name, const std::map<int, LibraryIndexItem> &item_id_description_map, const bool force_replacing_flag = false);

private:

    static void libraryIndexItemStructToMsg(const LibraryIndexItem& li_struct, int item_id, policy_library::Item& li_msg);
    static void libraryIndexItemMsgToStruct(const policy_library::Item& li_msg, LibraryIndexItem& li_struct);

};

inline void LibraryIO::libraryIndexItemStructToMsg(const LibraryIndexItem& li_struct, int item_id, policy_library::Item& li_msg)
{
    li_msg.item_description = li_struct.item_description_;
    li_msg.class_name = li_struct.class_name_;
    li_msg.item_id = item_id;
}

inline void LibraryIO::libraryIndexItemMsgToStruct(const policy_library::Item& li_msg, LibraryIndexItem& li_struct)
{
    li_struct.item_description_ = li_msg.item_description;
    li_struct.class_name_ = li_msg.class_name;
}

/*inline bool LibraryIO::addToLibraryIndex(const std::string library_directory_name, const int item_id, const std::string item_description,
                                     const bool force_replacing_flag)
{
    std::map<int, std::string> item_id_description_map;
    item_id_description_map.insert(std::pair<int, std::string>(item_id, item_description));
    return updateLibraryIndex(library_directory_name, item_id_description_map, force_replacing_flag);
}*/

inline bool LibraryIO::createLibraryEntry(const std::string library_directory_name, const std::string item_name)
{
    ROS_INFO("Checking whether %s exists.", item_name.c_str());
    if (boost::filesystem::exists(item_name))
    {
        ROS_WARN("Directory %s already exists, deleting it...", item_name.c_str());
        if (item_name.find("library") == std::string::npos)
        {
            ROS_ERROR("Directory %s does not contain \"item_\" and therefore to risky to delete.", item_name.c_str());
            return false;
        }

        // ROS_WARN("Deleting %s...", item_name.c_str());
        if (!boost::filesystem::remove_all(item_name.c_str()))
        {
            ROS_ERROR("Could not delete directory %s.", item_name.c_str());
            return false;
        }
    }

    ROS_INFO("Creating directory %s...", item_name.c_str());
    if (!boost::filesystem::create_directories(item_name.c_str()))
    {
        ROS_ERROR_STREAM("Could not create directory " << item_name << " :" << std::strerror(errno));
        return false;
    }

    return true;
}

inline bool LibraryIO::loadLibraryIndex(const std::string library_directory_name, std::map<int, LibraryIndexItem> &item_id_description_map)
{
    item_id_description_map.clear();
    std::string abs_bagfile_name = library_directory_name + library_index_file_name;
    if (!boost::filesystem::exists(abs_bagfile_name))
    {
        ROS_INFO("Library is empty.");
        return true;
    }

    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(library_index_topic_name));
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            policy_library::LibraryIndex::ConstPtr library_index = msg.instantiate<policy_library::LibraryIndex> ();
            if (library_index != NULL)
            {
                for (int i = 0; i < static_cast<int> (library_index->items.size()); i++)
                {
                    LibraryIndexItem li_item;
                    libraryIndexItemMsgToStruct(library_index->items[i], li_item);
                    item_id_description_map.insert(std::pair<int, LibraryIndexItem>(library_index->items[i].item_id, li_item));
                }
            }
            else
            {
                ROS_ERROR("Could not read bag file.");
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

inline bool LibraryIO::saveLibraryIndex(const std::string library_directory_name, const std::map<int, LibraryIndexItem> &library_index)
{
    std::string abs_bagfile_name = library_directory_name + library_index_file_name;

    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Write);
        policy_library::LibraryIndex library_index_msg;
        policy_library::Item library_item;
        for (std::map<int, LibraryIndexItem>::const_iterator mi = library_index.begin(); mi != library_index.end(); mi++)
        {
            libraryIndexItemStructToMsg(mi->second, mi->first, library_item);
            library_index_msg.items.push_back(library_item);
        }
        bag.write(library_index_topic_name, ros::Time::now(), library_index_msg);
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }

    return true;
}

inline bool LibraryIO::updateLibraryIndex(const std::string library_directory_name, const std::map<int, LibraryIndexItem> &item_id_description_map, const bool force_replacing_flag)
{

    if (item_id_description_map.empty())
    {
        ROS_INFO("Nothing to be updated in the library.");
        return false;
    }

    std::map<int, LibraryIndexItem> input_map;
    input_map = item_id_description_map;

    std::string abs_bagfile_name = library_directory_name + library_index_file_name;
    if (!boost::filesystem::exists(abs_bagfile_name))
    {
        return saveLibraryIndex(library_directory_name, input_map);
    }

    std::map<int, LibraryIndexItem> contained_items;
    if (!loadLibraryIndex(library_directory_name, contained_items))
    {
        ROS_ERROR("Could not read library index bag file.");
        return false;
    }

    // merge the maps:

    std::map<int, LibraryIndexItem> output_map;
    if(force_replacing_flag)
    {
        for (std::map<int, LibraryIndexItem>::iterator mi = input_map.begin(); mi != input_map.end(); mi++)
        {
            contained_items.erase(contained_items.find(mi->first));
            output_map.insert(std::pair<int, LibraryIndexItem>(mi->first, mi->second));
        }
        for (std::map<int, LibraryIndexItem>::iterator mi = contained_items.begin(); mi != contained_items.end(); mi++)
        {
            output_map.insert(std::pair<int, LibraryIndexItem>(mi->first, mi->second));
        }
    }
    else
    {
        for (std::map<int, LibraryIndexItem>::iterator mi = contained_items.begin(); mi != contained_items.end(); mi++)
        {
            if(input_map.find(mi->first) != input_map.end())
            {
                input_map.erase (input_map.find(mi->first));
            }
            output_map.insert(std::pair<int, LibraryIndexItem>(mi->first, mi->second));
        }
        for (std::map<int, LibraryIndexItem>::iterator mi = input_map.begin(); mi != input_map.end(); mi++)
        {
            output_map.insert(std::pair<int, LibraryIndexItem>(mi->first, mi->second));
        }
    }

    return saveLibraryIndex(library_directory_name, output_map);
}

}

#endif /* DMP_LIBRARY_IO_H_ */

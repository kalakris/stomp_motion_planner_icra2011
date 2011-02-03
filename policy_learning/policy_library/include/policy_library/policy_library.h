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

#ifndef POLICY_LIBRARY_H_
#define POLICY_LIBRARY_H_

// system includes
#include <vector>
#include <string>
#include <iostream>
#include <map>

#include <sstream>
#include <errno.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>

// local includes
#include <policy_library/library_item.h>
#include <policy_library/library_io.h>
#include <policy_library/policy_factory.h>

// defines

namespace library
{

// TODO: rename T into ItemTyp

template<class T>
    class PolicyLibrary
    {

    public:

        typedef boost::shared_ptr<T> item_ptr;
        typedef typename std::map<int, item_ptr> MapOfItemPtr;
        typedef typename std::pair<int, item_ptr> PairOfItemPtr;

        /*! constructor
         */
        PolicyLibrary(ros::NodeHandle& node_handle);

        /*! destructor
         *
         */
        ~PolicyLibrary();

        /*!
         *
         * @param library_directory_name
         * @return
         */
        bool initialize(const std::string library_directory_name);

        /*!
         */
        void print();

        /*!
         * @return
         */
        std::string getInfoString();

        /*!
         * @param item
         * @param force_replacing
         * @return
         */
        bool add(item_ptr item);

        /*!
         */
        void clear();

        /*!
         * @param item_id
         * @param item_ptr
         * @return
         */
        bool getItem(const int item_id, item_ptr &item);

        /*!
         * @return
         */
        int getFreeItemID();

        /*!
         * @return
         */
        bool updateFromDisc();

    protected:

        /*!
         */
        bool initialized_;

        /*!
         */
        MapOfItemPtr items_;

        /*!
         */
        std::map<int, LibraryIndexItem> library_index_;

        /*!
         */
        // std::map<int, std::vector<int> > trial_ids_;

        /*!
         */
        std::string library_directory_name_;

    private:

        bool readFromDisc();
        bool writeToDisc();

        bool updateLibraryIndexFile(const std::map<int, LibraryIndexItem> &item_id_description_map);
        bool updateLibraryIndexFile(const int item_id, const LibraryIndexItem& item_description);
        bool loadLibraryIndexFile();

        bool isSynchronized();

        // TODO replace by bool + reference...
        std::vector<int> getMissingIDsInMemory();
        std::vector<int> getMissingIDsOnDisc();

        PolicyFactory policy_factory_;

    };

// g++ requires template header in the same file as implementation.

template<class T>
    PolicyLibrary<T>::PolicyLibrary(ros::NodeHandle& node_handle) :
        initialized_(false), library_directory_name_(std::string("")), policy_factory_(node_handle)
    {
    }

template<class T>
    PolicyLibrary<T>::~PolicyLibrary()
    {
    }

template<class T>
    bool PolicyLibrary<T>::initialize(const std::string library_directory_name)
    {

        if (initialized_)
        {
            ROS_WARN("Library already initialized. Reinizializing from direcotory %s.",library_directory_name.c_str());
        }

        items_.clear();
        library_index_.clear();

        library_directory_name_.assign(library_directory_name);
        if (library_directory_name_.compare(library_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
        {
            library_directory_name_.append("/");
        }

        initialized_ = readFromDisc();
        return initialized_;
    }

template<class T>
    int PolicyLibrary<T>::getFreeItemID()
    {
        int item_id = 1;
        for (typename MapOfItemPtr::iterator mi = items_.begin(); mi != items_.end(); mi++)
        {
            if (item_id != mi->first)
            {
                return item_id;
            }
            item_id++;
        }
        return item_id;
    }

template<class T>
    bool PolicyLibrary<T>::getItem(const int item_id, item_ptr &item)
    {
        for (typename MapOfItemPtr::iterator mi = items_.begin(); mi != items_.end(); mi++)
        {
            if (mi->first == item_id)
            {
                item = mi->second;
                return true;
            }
        }
        ROS_ERROR("Item with id >%i< is not contained.", item_id);
        return false;
    }

template<class T>
    bool PolicyLibrary<T>::isSynchronized()
    {
        if (items_.size() != library_index_.size())
        {
            ROS_INFO_COND(items_.size()>library_index_.size(),"There are more items in memory than on disc.");
            ROS_INFO_COND(items_.size()<library_index_.size(),"There are more items on disc than in memory.");
            return false;
        }

        bool synchronized = true;

        std::vector<int> item_ids;
        item_ids = getMissingIDsInMemory();
        if (!item_ids.empty())
        {
            synchronized = false;
        }
        item_ids = getMissingIDsOnDisc();
        if (!item_ids.empty())
        {
            synchronized = false;
        }

        return synchronized;
    }

template<class T>
    std::vector<int> PolicyLibrary<T>::getMissingIDsInMemory()
    {
        std::vector<int> item_ids;
        for (std::map<int, LibraryIndexItem>::iterator mis = library_index_.begin(); mis != library_index_.end(); ++mis)
        {
            if (items_.find(mis->first) == items_.end())
            {
                // ROS_INFO("Item %i is missing in memory.", mis->first);
                item_ids.push_back(mis->first);
            }
        }
        return item_ids;
    }

template<class T>
    std::vector<int> PolicyLibrary<T>::getMissingIDsOnDisc()
    {
        std::vector<int> item_ids;
        for (typename MapOfItemPtr::iterator mi = items_.begin(); mi != items_.end(); mi++)
        {
            if (library_index_.find(mi->first) == library_index_.end())
            {
                ROS_INFO("Item %i is missing on disc.", mi->first);
                item_ids.push_back(mi->first);
            }
        }
        return item_ids;
    }

template<class T>
    bool PolicyLibrary<T>::add(item_ptr item)
    {
        int item_id = item->getID();

        ROS_INFO("Adding item with id %i.", item_id);

        if (!LibraryIO::createLibraryEntry(library_directory_name_, item->getName()))
        {
            ROS_ERROR("Could not create directory in library.");
            return false;
        }

        if (!item->writeToDisc())
        {
            ROS_ERROR("Could not write item to file.");
            return false;
        }

        LibraryIndexItem li_item;
        li_item.item_description_ = item->getDescription();
        li_item.class_name_ = item->getClassName();
        library_index_.insert(std::pair<int, LibraryIndexItem>(item_id, li_item));

        if (!updateLibraryIndexFile(item_id, li_item))
        {
            ROS_ERROR("Could not update index file.");
            return false;
        }

        items_.insert(PairOfItemPtr(item_id, item));
        return true;
    }

template<class T>
    bool PolicyLibrary<T>::loadLibraryIndexFile()
    {
        library_index_.clear();
        if (!LibraryIO::loadLibraryIndex(library_directory_name_, library_index_))
        {
            ROS_ERROR("Could not load library.");
            return false;
        }
        return true;
    }

template<class T>
    bool PolicyLibrary<T>::updateLibraryIndexFile(const int item_id, const LibraryIndexItem& item_description)
    {
        std::map<int, LibraryIndexItem> item_id_description_map;
        item_id_description_map.insert(std::pair<int, LibraryIndexItem>(item_id, item_description));
        return updateLibraryIndexFile(item_id_description_map);
    }

template<class T>
    bool PolicyLibrary<T>::updateLibraryIndexFile(const std::map<int, LibraryIndexItem> &item_id_description_map)
    {
        if (item_id_description_map.empty())
        {
            return false;
        }
        return LibraryIO::updateLibraryIndex(library_directory_name_, item_id_description_map);
    }

template<class T>
    void PolicyLibrary<T>::clear()
    {
        items_.clear();
        // TODO: check whether this makes sense
        library_index_.clear();
    }

template<class T>
    void PolicyLibrary<T>::print()
    {
        std::string info;
        info.assign(getInfoString());
        if (info.empty())
        {
            info.assign("Library is empty.");
        }
        ROS_INFO_STREAM(info);
    }

template<class T>
    std::string PolicyLibrary<T>::getInfoString()
    {
        std::string info;
        if (items_.empty())
        {
            return info;
        }

        for (typename MapOfItemPtr::iterator mi = items_.begin(); mi != items_.end(); mi++)
        {
            std::stringstream ss;
            ss << mi->second->getID();
            // info.append(std::string("\nid:") + ss.str() + std::string(" description:") + mi->second->getDescription());
            info.append(mi->second->getInfoString());
            info.append(std::string("\n\n"));
        }
        return info;
    }

template<class T>
    bool PolicyLibrary<T>::readFromDisc()
    {

        if (!loadLibraryIndexFile())
        {
            ROS_ERROR("Could not read library index file.");
            return false;
        }

        // read only those from file which are not already contained
        std::vector<int> ids;
        ids = getMissingIDsInMemory();
        for (std::vector<int>::iterator vi = ids.begin(); vi != ids.end(); vi++)
        {
            item_ptr item;

            std::string class_name = library_index_.find(*vi)->second.class_name_;

            if (!policy_factory_.createPolicyInstanceByName(class_name, item))
            {
                ROS_ERROR("Could not create policy by name: %s", class_name.c_str());
                return false;
            }
            ROS_INFO("Reading %s.", library_directory_name_.c_str());
            if (!item->readFromDisc(library_directory_name_, *vi))
            {
                ROS_ERROR("Could not read item with id %i from %s.", *vi, library_directory_name_.c_str());
                return false;
            }
            items_.insert(PairOfItemPtr(*vi, item));
        }

        return isSynchronized();
    }

template<class T>
    bool PolicyLibrary<T>::writeToDisc()
    {

        if (!loadLibraryIndexFile())
        {
            ROS_ERROR("Could not load library.");
            return false;
        }

        if (isSynchronized())
        {
            ROS_WARN("Library already synchronized. Not writing anything.");
            return true;
        }

        // write only those which are not already contained
        std::vector<int> item_ids;
        std::vector<std::string> item_names;
        item_ids = getMissingIDsOnDisc();
        for (std::vector<int>::iterator vi = item_ids.begin(); vi != item_ids.end(); vi++)
        {
            if (!items_[*vi]->writeToDisc())
            {
                ROS_ERROR("Could not write item with id %i to disc.", *vi);
                return false;
            }
            item_names.push_back(items_[*vi]->getName());
        }
        return updateLibraryIndexFile(item_ids, item_names);
    }

template<class T>
    bool PolicyLibrary<T>::updateFromDisc()
    {
        return readFromDisc();
    }

}

#endif /* POLICY_LIBRARY_H_ */

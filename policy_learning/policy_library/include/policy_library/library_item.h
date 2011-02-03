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

#ifndef DMP_LIBRARY_ITEM_H_
#define DMP_LIBRARY_ITEM_H_

// system includes
#include <string>

// ros includes
#include <ros/ros.h>

// local includes

namespace library
{

template<class T>
    class PolicyLibrary;

class LibraryItem
{

public:

    template<class T>
        friend class PolicyLibrary;

    /*! constructor
     */
    LibraryItem();

    /*!
     * @param directory_name
     * @param item_id
     */
    bool initializeBase(const std::string directory_name, const int item_id, const std::string item_name, const bool is_new_dmp = true);

    /*!
     */
    int getID() const;
    bool setID(const int id);

    /*!
     */
    std::string getName() const;
    virtual std::string getFileName(const int trial_id) = 0;

    /*!
     * @return
     */
    virtual std::string getInfoString() = 0;

    /*!
     *
     * @param description
     */
    void setDescription(const std::string& description);
    std::string getDescription() const;

    /**
     * @return the class name
     */
    virtual std::string getClassName() = 0;

    /*!
     */
    virtual bool readFromDisc(const std::string directory_name, const int item_id, const int trial_id = 0) = 0;
    virtual bool writeToDisc(const int trial_id = 0) = 0;

    virtual bool readFromDisc(const std::string abs_file_name) = 0;
    virtual bool writeToDisc(const std::string abs_file_name) = 0;

protected:

    /*!
     */
    bool initialized_;

    /*!
     */
    int item_id_;

    /*!
     */
    std::string item_name_;

    /*!
     */
    std::string library_directory_name_;

    /*!
     */
    std::string description_;

private:

    std::string initial_item_name_;

};

// inline functions follow
inline int LibraryItem::getID() const
{
    return item_id_;
}
inline bool LibraryItem::setID(const int item_id)
{
    item_id_ = item_id;
    return true;
    // return initializeBase(library_directory_name_, item_id_, initial_item_name_, false);
}
inline std::string LibraryItem::getName() const
{
    return item_name_;
}

inline void LibraryItem::setDescription(const std::string& description)
{
    description_.assign(description);
}
inline std::string LibraryItem::getDescription() const
{
    return description_;
}

}

#endif /* DMP_LIBRARY_ITEM_H_ */

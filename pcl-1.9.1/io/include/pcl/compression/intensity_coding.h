/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef INTENSITY_COMPRESSION_H
#define INTENSITY_COMPRESSION_H

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

namespace pcl
{
  namespace octree
  {
    using namespace std;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b IntensityCoding class
     *  \note This class encodes 32-bit (4 * 8-bit) intensity information for octree-based point cloud compression.
     *  \note Modifed from "color_coding.h" in PCL library
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Anlan Zhang (alzhang1506@gmail.com)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT>
      class IntensityCoding
      {

      // public typedefs
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      public:

        /** \brief Constructor.
         *
         * */
        IntensityCoding () :
          output_ (), pointAvgIntensityDataVector_ (), pointAvgIntensityDataVector_Iterator_ (),
          pointDiffIntensityDataVector_ (), pointDiffIntensityDataVector_Iterator_ (), intensityBitReduction_(0)
        {
        }

        /** \brief Empty class constructor. */
        virtual
        ~IntensityCoding ()
        {
        }

        /** \brief Set amount of voxels containing point intensity information and reserve memory
          * \param voxelCount_arg: amounts of voxels
          */
        inline void
        setVoxelCount (unsigned int voxelCount_arg)
        {
          pointAvgIntensityDataVector_.reserve (voxelCount_arg * 4);
        }

        /** \brief Set amount of points within point intensity to be encoded and reserve memory
         *  \param pointCount_arg: amounts of points within point cloud
         * */
        inline
        void
        setPointCount (unsigned int pointCount_arg)
        {
          pointDiffIntensityDataVector_.reserve (pointCount_arg * 4);
        }

        /** \brief Initialize encoding of intensity information
         * */
        void
        initializeEncoding ()
        {
          pointAvgIntensityDataVector_.clear ();

          pointDiffIntensityDataVector_.clear ();
        }

        /** \brief Initialize decoding of intensity information
         * */
        void
        initializeDecoding ()
        {
          pointAvgIntensityDataVector_Iterator_ = pointAvgIntensityDataVector_.begin ();

          pointDiffIntensityDataVector_Iterator_ = pointDiffIntensityDataVector_.begin ();
        }

        /** \brief Get reference to vector containing averaged intensity data
         * */
        std::vector<char>&
        getAverageDataVector ()
        {
          return pointAvgIntensityDataVector_;
        }

        /** \brief Get reference to vector containing differential intensity data
         * */
        std::vector<char>&
        getDifferentialDataVector ()
        {
          return pointDiffIntensityDataVector_;
        }

        /** \brief Encode averaged intensity information for a subset of points from point cloud
         * \param indexVector_arg indices defining a subset of points from points cloud
         * \param intensity_offset_arg offset to intensity information
         * \param inputCloud_arg input point cloud
         * */
        void
        encodeAverageOfPoints (const typename std::vector<int>& indexVector_arg, unsigned char intensity_offset_arg, PointCloudConstPtr inputCloud_arg)
        {
          std::size_t i, len;

          unsigned int avg0to7;
          unsigned int avg8to15;
          unsigned int avg16to23;
          unsigned int avg24to31;

          // initialize
          avg0to7 = avg8to15 = avg16to23 = avg24to31 = 0;

          // iterate over points
          len = indexVector_arg.size ();
          for (i = 0; i < len; i++)
          {
            // get intensity information from points
            const int& idx = indexVector_arg[i];
            const char* idxPointPtr = reinterpret_cast<const char*> (&inputCloud_arg->points[idx]);
            const int& intensityInt = *reinterpret_cast<const int*> (idxPointPtr+intensity_offset_arg);

            // add intensity information
            avg0to7 += (intensityInt >> 0) & 0xFF;
            avg8to15 += (intensityInt >> 8) & 0xFF;
            avg16to23 += (intensityInt >> 16) & 0xFF;
            avg24to31 += (intensityInt >> 24) & 0xFF;

          }

          // calculated average intensity information
          if (len > 1)
          {
            avg0to7   /= static_cast<unsigned int> (len);
            avg8to15  /= static_cast<unsigned int> (len);
            avg16to23  /= static_cast<unsigned int> (len);
            avg24to31  /= static_cast<unsigned int> (len);
          }

          // remove least significant bits
          avg0to7 >>= intensityBitReduction_;
          avg8to15 >>= intensityBitReduction_;
          avg16to23 >>= intensityBitReduction_;
          avg24to31 >>= intensityBitReduction_;

          // add to average intensity vector
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg0to7));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg8to15));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg16to23));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg24to31));
        }

        /** \brief Encode intensity information of a subset of points from point cloud
         * \param indexVector_arg indices defining a subset of points from points cloud
         * \param intensity_offset_arg offset to intensity information
         * \param inputCloud_arg input point cloud
         * */
        void
        encodePoints (const typename std::vector<int>& indexVector_arg, unsigned char intensity_offset_arg, PointCloudConstPtr inputCloud_arg)
        {
          std::size_t i, len;

          unsigned int avg0to7;
          unsigned int avg8to15;
          unsigned int avg16to23;
          unsigned int avg24to31;

          // initialize
          avg0to7 = avg8to15 = avg16to23 = avg24to31 = 0;

          // iterate over points
          len = indexVector_arg.size ();
          for (i = 0; i < len; i++)
          {
            // get intensity information from point
            const int& idx = indexVector_arg[i];
            const char* idxPointPtr = reinterpret_cast<const char*> (&inputCloud_arg->points[idx]);
            const int& intensityInt = *reinterpret_cast<const int*> (idxPointPtr+intensity_offset_arg);

            // add intensity information
            avg0to7 += (intensityInt >> 0) & 0xFF;
            avg8to15 += (intensityInt >> 8) & 0xFF;
            avg16to23 += (intensityInt >> 16) & 0xFF;
            avg24to31 += (intensityInt >> 24) & 0xFF;
          }

          if (len > 1)
          {
            unsigned char diff0to7;
            unsigned char diff8to15;
            unsigned char diff16to23;
            unsigned char diff24to31;

            // calculated average intensity information
            avg0to7   /= static_cast<unsigned int> (len);
            avg8to15  /= static_cast<unsigned int> (len);
            avg16to23  /= static_cast<unsigned int> (len);
            avg24to31  /= static_cast<unsigned int> (len);

            // iterate over points for differential encoding
            for (i = 0; i < len; i++)
            {
              const int& idx = indexVector_arg[i];
              const char* idxPointPtr = reinterpret_cast<const char*> (&inputCloud_arg->points[idx]);
              const int& intensityInt = *reinterpret_cast<const int*> (idxPointPtr+intensity_offset_arg);

              // extract intensity components and do XOR encoding with predicted average intensity
              diff0to7 = (static_cast<unsigned char> (avg0to7)) ^ static_cast<unsigned char> (((intensityInt >> 0) & 0xFF));
              diff8to15 = (static_cast<unsigned char> (avg8to15)) ^ static_cast<unsigned char> (((intensityInt >> 8) & 0xFF));
              diff16to23 = (static_cast<unsigned char> (avg16to23)) ^ static_cast<unsigned char> (((intensityInt >> 16) & 0xFF));
              diff24to31 = (static_cast<unsigned char> (avg24to31)) ^ static_cast<unsigned char> (((intensityInt >> 24) & 0xFF));

              // remove least significant bits
              diff0to7 = static_cast<unsigned char> (diff0to7 >> intensityBitReduction_);
              diff8to15 = static_cast<unsigned char> (diff8to15 >> intensityBitReduction_);
              diff16to23 = static_cast<unsigned char> (diff16to23 >> intensityBitReduction_);
              diff24to31 = static_cast<unsigned char> (diff24to31 >> intensityBitReduction_);

              // add to differential intensity vector
              pointDiffIntensityDataVector_.push_back (static_cast<char> (diff0to7));
              pointDiffIntensityDataVector_.push_back (static_cast<char> (diff8to15));
              pointDiffIntensityDataVector_.push_back (static_cast<char> (diff16to23));
              pointDiffIntensityDataVector_.push_back (static_cast<char> (diff24to31));
            }
          }

          // remove least significant bits from average intensity information
          avg0to7   >>= intensityBitReduction_;
          avg8to15 >>= intensityBitReduction_;
          avg16to23  >>= intensityBitReduction_;
          avg24to31  >>= intensityBitReduction_;

          // add to differential intensity vector
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg0to7));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg8to15));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg16to23));
          pointAvgIntensityDataVector_.push_back (static_cast<char> (avg24to31));
        }

        /** \brief Decode intensity information
          * \param outputCloud_arg output point cloud
          * \param beginIdx_arg index indicating first point to be assigned with intensity information
          * \param endIdx_arg index indicating last point to be assigned with intensity information
          * \param intensity_offset_arg offset to intensity information
          */
        void
        decodePoints (PointCloudPtr outputCloud_arg, std::size_t beginIdx_arg, std::size_t endIdx_arg, unsigned char intensity_offset_arg)
        {
          std::size_t i;
          unsigned int pointCount;
          unsigned int intensityInt;

          assert (beginIdx_arg <= endIdx_arg);

          // amount of points to be decoded
          pointCount = static_cast<unsigned int> (endIdx_arg - beginIdx_arg);

          // get averaged intensity information for current voxel
          unsigned char avg0to7 = *(pointAvgIntensityDataVector_Iterator_++);
          unsigned char avg8to15 = *(pointAvgIntensityDataVector_Iterator_++);
          unsigned char avg16to23 = *(pointAvgIntensityDataVector_Iterator_++);
          unsigned char avg24to31 = *(pointAvgIntensityDataVector_Iterator_++);

          // invert bit shifts during encoding
          avg0to7 = static_cast<unsigned char> (avg0to7 << intensityBitReduction_);
          avg8to15 = static_cast<unsigned char> (avg8to15 << intensityBitReduction_);
          avg16to23 = static_cast<unsigned char> (avg16to23 << intensityBitReduction_);
          avg24to31 = static_cast<unsigned char> (avg24to31 << intensityBitReduction_);

          // iterate over points
          for (i = 0; i < pointCount; i++)
          {
            if (pointCount > 1)
            {
              // get differential intensity information from input vector
              unsigned char diff0to7   = static_cast<unsigned char> (*(pointDiffIntensityDataVector_Iterator_++));
              unsigned char diff8to15 = static_cast<unsigned char> (*(pointDiffIntensityDataVector_Iterator_++));
              unsigned char diff16to23  = static_cast<unsigned char> (*(pointDiffIntensityDataVector_Iterator_++));
              unsigned char diff24to31  = static_cast<unsigned char> (*(pointDiffIntensityDataVector_Iterator_++));

              // invert bit shifts during encoding
              diff0to7 = static_cast<unsigned char> (diff0to7 << intensityBitReduction_);
              diff8to15 = static_cast<unsigned char> (diff8to15 << intensityBitReduction_);
              diff16to23 = static_cast<unsigned char> (diff16to23 << intensityBitReduction_);
              diff24to31 = static_cast<unsigned char> (diff24to31 << intensityBitReduction_);

              // decode intensity information
              intensityInt = ((avg0to7 ^ diff0to7) << 0) |
                           ((avg8to15 ^ diff8to15) << 8) |
                         ((avg16to23 ^ diff16to23) << 16)|
                         ((avg24to31 ^ diff24to31) << 24);
            }
            else
            {
              // decode intensity information
              intensityInt = (avg0to7 << 0) | (avg8to15 << 8) | (avg16to23 << 16) | (avg24to31 << 24);
            }

            char* idxPointPtr = reinterpret_cast<char*> (&outputCloud_arg->points[beginIdx_arg + i]);
            int& pointIntensity = *reinterpret_cast<int*> (idxPointPtr+intensity_offset_arg);
            // assign intensity to point from point cloud
            pointIntensity=intensityInt;
          }
        }

        /** \brief Set default intensity to points
         * \param outputCloud_arg output point cloud
         * \param beginIdx_arg index indicating first point to be assigned with intensity information
         * \param endIdx_arg index indicating last point to be assigned with intensity information
         * \param intensity_offset_arg offset to intensity information
         * */
        void
        setDefaultIntensity (PointCloudPtr outputCloud_arg, std::size_t beginIdx_arg, std::size_t endIdx_arg, unsigned char intensity_offset_arg)
        {
          std::size_t i;
          unsigned int pointCount;

          assert (beginIdx_arg <= endIdx_arg);

          // amount of points to be decoded
          pointCount = static_cast<unsigned int> (endIdx_arg - beginIdx_arg);

          // iterate over points
          for (i = 0; i < pointCount; i++)
          {
            char* idxPointPtr = reinterpret_cast<char*> (&outputCloud_arg->points[beginIdx_arg + i]);
            int& pointIntensity = *reinterpret_cast<int*> (idxPointPtr+intensity_offset_arg);
            // assign intensity to point from point cloud
            pointIntensity = defaultIntensity_;
          }
        }


      protected:

        /** \brief Pointer to output point cloud dataset. */
        PointCloudPtr output_;

        /** \brief Vector for storing average intensity information  */
        std::vector<char> pointAvgIntensityDataVector_;

        /** \brief Iterator on average intensity information vector */
        std::vector<char>::const_iterator pointAvgIntensityDataVector_Iterator_;

        /** \brief Vector for storing differential intensity information  */
        std::vector<char> pointDiffIntensityDataVector_;

        /** \brief Iterator on differential intensity information vector */
        std::vector<char>::const_iterator pointDiffIntensityDataVector_Iterator_;

        /** \brief Amount of bits to be removed from intensity components before encoding */
        unsigned char intensityBitReduction_;

        // frame header identifier
        static const int defaultIntensity_;

      };

    // define default intensity
    template<typename PointT>
    const int IntensityCoding<PointT>::defaultIntensity_ = 0;

  }
}

#define PCL_INSTANTIATE_IntensityCoding(T) template class PCL_EXPORTS pcl::octree::IntensityCoding<T>;

#endif

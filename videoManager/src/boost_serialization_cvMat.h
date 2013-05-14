/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: SENEKA
 * \note
 *   ROS stack name: SENEKA
 * \note
 *   ROS package name: VideoManager
 *
 * \author
 *   Author: Johannes Goth (cmm-jg)
 * \author
 *   Supervised by: Christophe Maufroy (cmm)
 *
 * \date Date of creation: 19.03.2013
 *
 * \brief
 *   boost_serialization_cvMat.h
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef BOOST_SERIALIZATION_CVMAT_H_
#define BOOST_SERIALIZATION_CVMAT_H_

#include <iostream>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

// The macro BOOST_SERIALIZATION_SPLIT_MEMBER() generates code which invokes the save or load depending on whether the archive is used for saving or loading.
BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

/* Serialization of cv:Mat
 * Define serialization of cv:Mat with specific save and load functions, which are used by boot::archive to store the cv:Mat into a archive file (e.g. text or binary format) */
namespace boost {
	namespace serialization {

		template<class Archive>
		void save(Archive & ar, const cv::Mat& m, const unsigned int version)
		{
			size_t matSize = m.elemSize();
			size_t matType = m.type();

			// cv::Mat header information
			ar & m.cols;
			ar & m.rows;
			ar & matSize;
			ar & matType;	// size of the stored element type e.g. integer
			size_t dataSize = m.cols * m.rows * m.elemSize();

			// save cv::Mat image data
			for (size_t dc = 0; dc < dataSize; ++dc) {
				ar & m.data[dc];
			}
		}

		template<class Archive>
		void load(Archive & ar, cv::Mat& m, const unsigned int version)
		{
			int cols, rows;
			size_t matSize, matType;

			ar & cols;
			ar & rows;
			ar & matSize;
			ar & matType;

			m.create(rows, cols, matType);
			size_t dataSize = m.cols * m.rows * matSize;

			//cout << "reading matrix data rows, cols, elemSize, type, datasize: (" << m.rows << "," << m.cols << "," << m.elemSize() << "," << m.type() << "," << dataSize << ")" << endl;

			// load cv::Mat image data
			for (size_t dc = 0; dc < dataSize; ++dc) {
				ar & m.data[dc];
			}
		}


		// Contains the input file another cv::Mat
		template<class Archive, class Stream, class Obj>
		bool try_stream_next(Archive &ar, const Stream &s, Obj &o)
		{
			bool success = false;

			// try to read data from archive file
			try {
				ar >> o;
				success = true;
			}
			// if there occurs an archive_exception::input_stream_error, throw it
			catch (const boost::archive::archive_exception &e) {
				if (e.code != boost::archive::archive_exception::input_stream_error) {
					throw;
				}
			}

			// otherwise return false, so the file is empty
			return success;
		}
	}
}



#endif /* BOOST_SERIALIZATION_CVMAT_H_ */

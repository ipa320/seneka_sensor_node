/*
 * boost_serialization_cvMat.h
 *
 *  Created on: 19.03.2013
 *      Author: cmm-jg
 */

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

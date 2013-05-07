/*
 * boost_serialization_sensor_msgs_image.h
 *
 *  Created on: 18.04.2013
 *      Author: Author: Johannes Goth (cmm-jg)
 */

#ifndef BOOST_SERIALIZATION_CVMAT_H_
#define BOOST_SERIALIZATION_CVMAT_H_

#include <ros/common.h>
#include <iostream>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

// The macro BOOST_SERIALIZATION_SPLIT_MEMBER() generates code which invokes the save or load depending on whether the archive is used for saving or loading.
BOOST_SERIALIZATION_SPLIT_FREE(sensor_msgs::Image)

/* Serialization of sensor_msgs::Image
 * Define serialization of sensor_msgs::Image with specific save and load functions, which are used by boot::archive to store the sensor_msgs::Image into a archive file (e.g. text or binary format) */
namespace boost {
	namespace serialization {

		template<class Archive>
		void save(Archive & ar, const sensor_msgs::Image& m, const unsigned int version)
		{
			ar & m.header.seq;
			ar & m.height;
			ar & m.width;
			ar & m.encoding;
			ar & m.is_bigendian;
			ar & m.step;

			size_t dataSize = m.data.size();

			for (size_t dc = 0; dc < dataSize; ++dc) {
				ar & m.data[dc];
			}
		}

		template<class Archive>
		void load(Archive & ar, sensor_msgs::Image& m, const unsigned int version)
		{
			ar & m.header.seq;
			ar & m.height;
			ar & m.width;
			ar & m.encoding;
			ar & m.is_bigendian;
			ar & m.step;

			//std::cout << "reading matrix data rows, cols, elemSize, type, datasize: (" << m.height << "," << m.width << "," << m.encoding << "," << ")" << std::endl;

			size_t dataSize = m.height * m.width *2;

			unsigned char tmp;
			for (size_t dc = 0; dc < dataSize; ++dc) {
				ar & tmp;
				m.data.push_back(tmp);
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

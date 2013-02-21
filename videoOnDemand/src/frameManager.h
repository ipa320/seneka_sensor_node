/*
 * frameManager.h
 *
 *  Created on: 21.02.2013
 *      Author: Johannes Goth (cmm-jg)
 */

#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_

class frameManager {
public:
	frameManager();
	virtual ~frameManager();
	void startCaching();
	int getVideo();

private:

	// methods
	void cacheFrame(cv::Mat frame);
	void verifyCacheSize();
	void storeCache(std::vector<cv::Mat>* cache);
	int createVideo();
	std::vector<cv::Mat>* getCurrentCache();


	// variables
	u_int maxCacheSize;
	bool useCacheA;
	bool useCacheB;
	std::vector<cv::Mat>* cacheA;
	std::vector<cv::Mat>* cacheB;
	std::vector<cv::FileStorage*> storageFiles;


};

#endif /* FRAMEMANAGERH_ */

#pragma once

#include <unordered_map>
#include <mutex>

#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/Switch>
#include <osg/Program>
#include <osgDB/ReadFile>
#include <osg/LineWidth>
#include <osgViewer/Renderer>
#include <osg/Texture2D>
#include <osg/PolygonMode>

#include "messageStruct.h"

#include "commonOsg/osgManagerBase.h"

#include <osg/MatrixTransform>

class OsgManager : public OsgManagerBase
{
public:
	static OsgManager* getInstance() {
		if (m_pInstance == nullptr) {
			m_pInstance = new OsgManager();
		}
		return m_pInstance;
	}

	virtual ~OsgManager();

	void run();
	void getPosition();
	void test();

	void getPoints(const sensor_msgs::PointCloud2::ConstPtr& msg_in);

	void updatePosition(double x, double y, double z, double w, double tx, double ty, double tz);

protected:
	static OsgManager* m_pInstance;

protected:
	OsgManager();
	
private:
	osg::ref_ptr<osg::Group> root = nullptr;
	osg::ref_ptr<osg::Group> rootWireTerrain = nullptr;
	osg::ref_ptr<osg::Geometry> pathGeom = nullptr;
	osg::ref_ptr<osg::Group> sunLight = nullptr;
	osg::ref_ptr<osgViewer::Viewer> pviewer = nullptr;
	osg::ref_ptr<osg::Switch> sceneSwitch;

	osg::ref_ptr<osg::MatrixTransform> localAxisNode = nullptr;
	osg::Matrix localAxisMatrix;
	std::mutex notifyMutex;
};


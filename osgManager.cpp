#include "osgManager.h"

#include "airsimManager.h"

OsgManager* OsgManager::m_pInstance = nullptr;

OsgManager::OsgManager() : OsgManagerBase() {}

OsgManager::~OsgManager() {
	pviewer.release();
}

void OsgManager::getPoints(const sensor_msgs::PointCloud2::ConstPtr& msg_in) {
	std::cout << "get points " << msg_in->pcl_pc.size() << std::endl;
	// do something here
	m_pSceneSwitcher->removeChildren(0, m_pSceneSwitcher->getNumChildren());

	osg::ref_ptr<osg::Geometry> pGeomPoints = new osg::Geometry();
	osg::Vec3Array* pVertex = new osg::Vec3Array();

	for (int i = 0; i < msg_in->pcl_pc.size(); ++i) {
		pVertex->push_back(osg::Vec3(msg_in->pcl_pc[i].x, msg_in->pcl_pc[i].y, msg_in->pcl_pc[i].z));
	}

	pGeomPoints->setVertexArray(pVertex);
	pGeomPoints->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pVertex->size()));

	m_pSceneSwitcher->addChild(pGeomPoints);
}

void OsgManager::run() {
	std::thread thread([]() {
		AirSimManager::getInstance()->run();
		});
	thread.detach();
}

void OsgManager::getPosition() {
	AirSimManager::getInstance()->getPosition();
}

void OsgManager::test() {
	AirSimManager::getInstance()->test();
}
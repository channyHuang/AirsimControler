#include "osgManager.h"

#include "airsimManager.h"
#include "commonOsg/commonOsg.h"

OsgManager* OsgManager::m_pInstance = nullptr;

OsgManager::OsgManager() : OsgManagerBase() {
	localAxisNode = new osg::MatrixTransform;
	localAxisMatrix.makeIdentity();
	localAxisNode->setMatrix(localAxisMatrix);
	localAxisNode->addChild(createAxis());
	m_pSceneSwitcher->addChild(localAxisNode);
}

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

void OsgManager::updatePosition(double x, double y, double z, double w, double tx, double ty, double tz) {
	localAxisMatrix.setRotate(osg::Quat(x, y, z, w));
	localAxisMatrix.setTrans(osg::Vec3d(tx, ty, tz));
	localAxisNode->setMatrix(localAxisMatrix);

	if (pathGeom == nullptr) {
		osg::Vec3Array* varray = new osg::Vec3Array();
		osg::Vec3Array* carray = new osg::Vec3Array();

		varray->push_back(osg::Vec3(tx, ty, tz));
		carray->push_back(osg::Vec3(1, 1, 0));

		pathGeom = new osg::Geometry;
		pathGeom->setVertexArray(varray);
		pathGeom->setColorArray(carray, osg::Array::Binding::BIND_OVERALL);
		pathGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, varray->size()));

		m_pSceneSwitcher->addChild(pathGeom);
	}
	else {
		osg::Vec3Array* varray = (osg::Vec3Array*)pathGeom->getVertexArray();
		varray->push_back(osg::Vec3(tx, ty, tz));

		pathGeom->setVertexArray(varray);
		pathGeom->removePrimitiveSet(0, pathGeom->getNumPrimitiveSets());
		
		pathGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, varray->size()));
		
		pathGeom->dirtyBound();
		pathGeom->dirtyGLObjects();
	}
}
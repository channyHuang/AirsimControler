#include "osgManager.h"

#include "osgPickHandler.h"

#include "airsimManager.h"

osg::ref_ptr<osg::LineWidth> getNewLineWidth(float width) {
	osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
	linewidth->setWidth(width);
	return linewidth;
}

osg::ref_ptr<osg::Geode> createAxis(float len)
{
	osg::Geode* geode(new osg::Geode());
	osg::Geometry* geometry(new osg::Geometry());

	osg::Vec3Array* vertices(new osg::Vec3Array());
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(len, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, len, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, len));
	geometry->setVertexArray(vertices);

	osg::Vec4Array* colors(new osg::Vec4Array());
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::LineWidth> linewidth = getNewLineWidth(2.f);
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

	return geode;
}

OsgManager* OsgManager::instance = nullptr;

OsgManager::OsgManager() {
	root = new osg::Group;
	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	sceneSwitch = new osg::Switch;
	sceneSwitch->setAllChildrenOn();
	root->addChild(sceneSwitch);
	root->addChild(createAxis(0.1f));
}

OsgManager::~OsgManager() {
	pviewer.release();
}

void OsgManager::setViewer(osgViewer::Viewer& viewer) {
	pviewer = &viewer;

	pviewer->addEventHandler(new PickHandler());
	pviewer->setSceneData(root);

	AirSimManager::getInstance()->notifyPoints.connect(this, &OsgManager::getPoints);
}

void OsgManager::switchScene() {
	sceneMaxIdx = sceneSwitch->getNumChildren();
	if (sceneIdx >= sceneMaxIdx) {
		sceneSwitch->setAllChildrenOn();
		sceneIdx = 0;
	}
	else {
		sceneSwitch->setSingleChildOn(sceneIdx);
		sceneIdx++;
	}
}

void OsgManager::getPoints(const sensor_msgs::PointCloud2::ConstPtr& msg_in) {
	std::cout << "get points " << msg_in->pcl_pc.size() << std::endl;
	// do something here
	sceneSwitch->removeChildren(0, sceneSwitch->getNumChildren());

	osg::ref_ptr<osg::Geometry> geomPoints = new osg::Geometry();
	osg::Vec3Array* varray = new osg::Vec3Array();

	for (int i = 0; i < msg_in->pcl_pc.size(); ++i) {
		varray->push_back(osg::Vec3(msg_in->pcl_pc[i].x, msg_in->pcl_pc[i].y, msg_in->pcl_pc[i].z));
	}

	geomPoints->setVertexArray(varray);
	geomPoints->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, varray->size()));

	sceneSwitch->addChild(geomPoints);

	root->dirtyBound();
}

void OsgManager::run() {
	
	std::thread thread([]() {
		AirSimManager::getInstance()->run();
		});
thread.detach();
}

void OsgManager::getPosition() {
	// AirSimManager::getInstance()->getPosition();
}

void OsgManager::test() {
	// AirSimManager::getInstance()->test();
}
#include "ImguiMainPage.h"

#include "osgManager.h"
#include <thread>
GLuint textureID;

#include "commonOsg/commonOsg.h"

#include "nativefiledialog/nfd.h"

void pickCbFunc(const osg::Vec3& vPos, void* pUser) {
	// do someing here
    OsgManager::getInstance()->showPick(vPos);
}

ImguiMainPage::ImguiMainPage() {
    
}

ImguiMainPage::ImguiMainPage(osgViewer::Viewer& viewer, osg::ref_ptr< CameraHandler> pCameraHandler) {
    pviewer = &viewer;
    m_pCameraHandler = pCameraHandler;
    OsgManager::getInstance()->setViewer(viewer);
    cFileName = new char[nMaxFileNameLength];
    memset(cFileName, 0, nMaxFileNameLength);
    cTexturePath = new char[nMaxFileNameLength];
    memset(cTexturePath, 0, nMaxFileNameLength);

    m_pPicker = new PickHandler();
    m_pPicker->setCallback(pickCbFunc, nullptr);
    viewer.addEventHandler(m_pPicker);
}

ImguiMainPage::~ImguiMainPage() {
    pviewer = nullptr;
    if (cFileName != nullptr) {
        delete[]cFileName;
    }
    if (cTexturePath != nullptr) {
        delete[]cTexturePath;
    }
}

void ImguiMainPage::drawUi() {
    ImGui::Begin("Airsim Control");
    if (ImGui::Button("Switch Scene")) {
        OsgManager::getInstance()->switchScene();
    }
    if (ImGui::InputTextWithHint("file", "<.obj .ply .xyz>", cFileName, nMaxFileNameLength, ImGuiInputTextFlags_EnterReturnsTrue)) {
    }
    ImGui::SameLine();
    if (ImGui::Button("Open File")) {
        nfdresult_t result = NFD_OpenDialog(""/*"obj,ply,xyz,csv"*/, nullptr, &cFileName);
        if (result == NFD_OKAY) {

        }
    }
    if (ImGui::Button("Reset Scene")) {
        m_pCameraHandler->reset();
    }

    if (ImGui::BeginTabBar("Control", ImGuiTabBarFlags_None))
    {
        if (ImGui::BeginTabItem("Drone"))
        {
            if (ImGui::Button("move")) {
                std::thread runThread([]() { OsgManager::getInstance()->run(); });
                runThread.detach();
            }
            if (ImGui::Button("get position")) {
                OsgManager::getInstance()->getPosition();
            }
            if (ImGui::Button("test")) {
                OsgManager::getInstance()->test();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::End();
}

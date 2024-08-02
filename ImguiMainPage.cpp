#include "ImguiMainPage.h"

#include "osgManager.h"
#include <thread>
GLuint textureID;

ImguiMainPage::ImguiMainPage() {
    
}

ImguiMainPage::ImguiMainPage(osgViewer::Viewer& viewer) {
    pviewer = &viewer;

    OsgManager::getInstance()->setViewer(viewer);

    cFileName = new char[nMaxFileNameLength];
    memset(cFileName, 0, nMaxFileNameLength);
}

ImguiMainPage::~ImguiMainPage() {
    pviewer = nullptr;
}

void ImguiMainPage::drawUi() {
    ImGui::Begin("imgui osg");
    if (ImGui::Button("Switch Scene")) {
        OsgManager::getInstance()->switchScene();
    }
    if (ImGui::InputTextWithHint("file", "<.obj .ply .xyz>", cFileName, nMaxFileNameLength, ImGuiInputTextFlags_EnterReturnsTrue)) {
    }
    ImGui::SameLine();
    if (ImGui::Button("Open File")) {
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

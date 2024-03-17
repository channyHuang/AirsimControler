#include "ImguiMainPage.h"

#include "osgManager.h"
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
                OsgManager::getInstance()->run();
            }

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::End();
}

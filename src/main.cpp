#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "vec2.hpp"
#include "arm.hpp"
#include <vector>
#include <string>

static const int SCREEN_W = 1150;
static const int SCREEN_H = 720;
static const int PANEL_W = 300;
static const float OBSTACLE_RADIUS = 26.0f;

static const Color PAL_CREAM  = { 0xFE, 0xF5, 0xF0, 255 };
static const Color PAL_YELLOW = { 0xFF, 0xDA, 0x44, 255 };
static const Color PAL_ORANGE = { 0xFC, 0x5E, 0x3B, 255 };
static const Color PAL_PINK   = { 0xDD, 0x7A, 0xB3, 255 };
static const Color PAL_INDIGO = { 0x45, 0x45, 0xE5, 255 };

static int ColorToStyleInt(Color c) {
    return (c.r << 24) | (c.g << 16) | (c.b << 8) | c.a;
}

const char* methodName(IKMethod m) {
    if (m == IKMethod::Transpose) return "Jacobian Transpose";
    if (m == IKMethod::PseudoInverse) return "Pseudo-Inverse";
    return "Damped Least Squares";
}

int main() {
    InitWindow(SCREEN_W, SCREEN_H, "Reach Me If You Can");
    SetTargetFPS(60);

    Font fontRegular = LoadFontEx("assets/fonts/Poppins-Regular.ttf", 64, nullptr, 0);
    Font fontSemiBold = LoadFontEx("assets/fonts/Poppins-SemiBold.ttf", 64, nullptr, 0);
    SetTextureFilter(fontRegular.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(fontSemiBold.texture, TEXTURE_FILTER_BILINEAR);
    bool haveCustomFont = fontRegular.texture.id != 0 && fontSemiBold.texture.id != 0;
    if (haveCustomFont) GuiSetFont(fontRegular);

    GuiSetStyle(DEFAULT, BACKGROUND_COLOR, ColorToStyleInt(PAL_CREAM));
    GuiSetStyle(DEFAULT, LINE_COLOR, ColorToStyleInt(Fade(PAL_INDIGO, 0.35f)));
    GuiSetStyle(DEFAULT, TEXT_SIZE, 14);
    GuiSetStyle(DEFAULT, TEXT_SPACING, 0);

    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(DEFAULT, TEXT_COLOR_FOCUSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(DEFAULT, TEXT_COLOR_PRESSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(DEFAULT, BORDER_COLOR_NORMAL, ColorToStyleInt(Fade(PAL_INDIGO, 0.45f)));
    GuiSetStyle(DEFAULT, BASE_COLOR_NORMAL, ColorToStyleInt(RAYWHITE));

    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToStyleInt(PAL_YELLOW));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToStyleInt(PAL_PINK));
    GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToStyleInt(PAL_ORANGE));
    GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToStyleInt(RAYWHITE));

    GuiSetStyle(SLIDER, BASE_COLOR_NORMAL, ColorToStyleInt(PAL_YELLOW));
    GuiSetStyle(SLIDER, BASE_COLOR_FOCUSED, ColorToStyleInt(PAL_ORANGE));
    GuiSetStyle(SLIDER, BASE_COLOR_PRESSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(SLIDER, BORDER_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));

    GuiSetStyle(TOGGLE, BASE_COLOR_NORMAL, ColorToStyleInt(RAYWHITE));
    GuiSetStyle(TOGGLE, BASE_COLOR_FOCUSED, ColorToStyleInt(PAL_PINK));
    GuiSetStyle(TOGGLE, BASE_COLOR_PRESSED, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(TOGGLE, BORDER_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(TOGGLE, TEXT_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(TOGGLE, TEXT_COLOR_PRESSED, ColorToStyleInt(RAYWHITE));

    GuiSetStyle(CHECKBOX, BASE_COLOR_NORMAL, ColorToStyleInt(PAL_CREAM));
    GuiSetStyle(CHECKBOX, BASE_COLOR_PRESSED, ColorToStyleInt(PAL_ORANGE));
    GuiSetStyle(CHECKBOX, BORDER_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));

    GuiSetStyle(VALUEBOX, BASE_COLOR_NORMAL, ColorToStyleInt(RAYWHITE));
    GuiSetStyle(VALUEBOX, BORDER_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));
    GuiSetStyle(VALUEBOX, TEXT_COLOR_NORMAL, ColorToStyleInt(PAL_INDIGO));

    int nJoints = 3;
    float segLen[5] = { 90, 90, 90, 90, 90 };
    float angleLimit = 165.0f;
    int methodIdx = 1;
    bool avoidObstacles = false;

    Vec2 base(PANEL_W + (SCREEN_W - PANEL_W) / 2.0f, SCREEN_H - 90.0f);
    std::vector<double> lens(segLen, segLen + nJoints);
    RobotArm arm(nJoints, lens, base);
    arm.angleLimitDeg = angleLimit;

    bool hasTarget = false;
    Vec2 target;
    StepResult lastResult;
    std::vector<Obstacle> obstacles;
    int iterationCount = 0;

    while (!WindowShouldClose()) {
        Vector2 mp = GetMousePosition();
        bool overPanel = mp.x < PANEL_W;

        if (!overPanel) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                target = Vec2(mp.x, mp.y);
                hasTarget = true;
                lastResult = StepResult();
                iterationCount = 0;
            }
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && avoidObstacles) {
                Vec2 click(mp.x, mp.y);
                int hitIdx = -1;
                for (size_t i = 0; i < obstacles.size(); i++) if ((obstacles[i].pos - click).length() < OBSTACLE_RADIUS) hitIdx = (int)i;
                if (hitIdx >= 0) obstacles.erase(obstacles.begin() + hitIdx);
                else {
                    Obstacle cand(click, OBSTACLE_RADIUS);
                    auto joints = arm.jointPositions();
                    bool overlap = false;
                    for (size_t k = 0; k + 1 < joints.size(); k++) if (segmentIntersectsObstacle(joints[k], joints[k + 1], cand)) overlap = true;
                    if (!overlap) obstacles.push_back(cand);
                }
            }
        }

        if (hasTarget && !lastResult.converged && !lastResult.unreachable) {
            lastResult = arm.step(target, (IKMethod)methodIdx, obstacles, avoidObstacles);
            iterationCount++;
        }

        BeginDrawing();
        ClearBackground(PAL_CREAM);

        for (const auto& o : obstacles) DrawCircleV({ (float)o.pos.x, (float)o.pos.y }, (float)o.radius, Fade(PAL_INDIGO, 0.35f));
        for (const auto& o : obstacles) DrawCircleLines((int)o.pos.x, (int)o.pos.y, (float)o.radius, PAL_INDIGO);

        auto joints = arm.jointPositions();
        for (size_t i = 0; i + 1 < joints.size(); i++) {
            Color linkColor = lastResult.singular ? PAL_ORANGE : PAL_INDIGO;
            DrawLineEx({ (float)joints[i].x, (float)joints[i].y }, { (float)joints[i + 1].x, (float)joints[i + 1].y }, 9.0f, linkColor);
        }
        for (size_t i = 0; i < joints.size(); i++) DrawCircleV({ (float)joints[i].x, (float)joints[i].y }, 10.0f, i == 0 ? PAL_INDIGO : PAL_PINK);
        DrawCircleV({ (float)joints.back().x, (float)joints.back().y }, 7.0f, PAL_YELLOW);

        if (hasTarget) {
            Color tc = lastResult.unreachable ? PAL_ORANGE : (lastResult.converged ? (Color){ 46, 163, 100, 255 } : PAL_PINK);
            DrawCircleLines((int)target.x, (int)target.y, 12.0f, tc);
            DrawLine((int)target.x - 16, (int)target.y, (int)target.x + 16, (int)target.y, tc);
            DrawLine((int)target.x, (int)target.y - 16, (int)target.x, (int)target.y + 16, tc);
        }

        DrawRectangle(0, 0, PANEL_W, SCREEN_H, PAL_CREAM);
        DrawRectangle(PANEL_W - 3, 0, 3, SCREEN_H, Fade(PAL_INDIGO, 0.25f));
        int y = 14;
        GuiLabel({ 14, (float)y, 270, 20 }, "Jumlah sendi (N)"); y += 22;
        int nBefore = nJoints;
        GuiSpinner({ 14, (float)y, 270, 26 }, nullptr, &nJoints, 2, 5, false); y += 34;
        if (nJoints != nBefore) { arm.setJointCount(nJoints); hasTarget = false; lastResult = StepResult(); }

        for (int i = 0; i < nJoints; i++) {
            std::string lbl = "Panjang segmen " + std::to_string(i + 1) + ": " + std::to_string((int)segLen[i]);
            GuiLabel({ 14, (float)y, 270, 18 }, lbl.c_str()); y += 18;
            float before = segLen[i];
            GuiSlider({ 14, (float)y, 270, 18 }, "", "", &segLen[i], 30.0f, 160.0f);
            if (segLen[i] != before) arm.lengths[i] = segLen[i];
            y += 26;
        }

        y += 6;
        std::string limLbl = "Batas sudut sendi: +/-" + std::to_string((int)angleLimit) + " deg";
        GuiLabel({ 14, (float)y, 270, 18 }, limLbl.c_str()); y += 18;
        float limBefore = angleLimit;
        GuiSlider({ 14, (float)y, 270, 18 }, "", "", &angleLimit, 20.0f, 180.0f);
        if (angleLimit != limBefore) arm.angleLimitDeg = angleLimit;
        y += 34;

        GuiLabel({ 14, (float)y, 270, 20 }, "Metode Inverse Kinematics"); y += 22;
        const char* methods = "Transpose;Pseudo-Inverse;DLS";
        GuiToggleGroup({ 14, (float)y, 88, 26 }, methods, &methodIdx); y += 40;

        GuiCheckBox({ 14, (float)y, 20, 20 }, "Hindari obstacle (klik kanan)", &avoidObstacles);
        y += 34;

        if (GuiButton({ 14, (float)y, 130, 30 }, "Reset")) {
            arm = RobotArm(nJoints, std::vector<double>(segLen, segLen + nJoints), base);
            arm.angleLimitDeg = angleLimit;
            hasTarget = false; lastResult = StepResult(); obstacles.clear(); iterationCount = 0;
        }
        if (GuiButton({ 154, (float)y, 130, 30 }, "Hapus obstacle")) obstacles.clear();
        y += 44;

        DrawLine(14, y, PANEL_W - 14, y, GRAY); y += 10;
        GuiLabel({ 14, (float)y, 270, 18 }, "Klik kiri kanvas: set target"); y += 18;
        GuiLabel({ 14, (float)y, 270, 18 }, "Klik kanan kanvas: taruh/hapus obstacle"); y += 26;

        GuiLabel({ 14, (float)y, 270, 18 }, TextFormat("Metode aktif: %s", methodName((IKMethod)methodIdx))); y += 20;
        GuiLabel({ 14, (float)y, 270, 18 }, TextFormat("Iterasi: %d", iterationCount)); y += 20;
        GuiLabel({ 14, (float)y, 270, 18 }, TextFormat("Error jarak: %.2f px", lastResult.errorNorm)); y += 20;
        GuiLabel({ 14, (float)y, 270, 18 }, TextFormat("Manipulability: %.4f", lastResult.manipulability)); y += 20;

        {
            const char* statusText;
            Color statusColor;
            int statusSize;
            if (!hasTarget) { statusText = "Klik di kanvas untuk menentukan target"; statusColor = Fade(PAL_INDIGO, 0.7f); statusSize = 18; }
            else if (lastResult.unreachable && lastResult.angleLimited) { statusText = "TARGET TIDAK TERJANGKAU (dibatasi sudut sendi)"; statusColor = PAL_ORANGE; statusSize = 20; }
            else if (lastResult.unreachable) { statusText = "TARGET TIDAK TERJANGKAU (di luar jangkauan jarak)"; statusColor = PAL_ORANGE; statusSize = 20; }
            else if (lastResult.converged) { statusText = "Target tercapai"; statusColor = (Color){ 46, 163, 100, 255 }; statusSize = 20; }
            else if (lastResult.singular) { statusText = "Peringatan: konfigurasi mendekati singular"; statusColor = PAL_ORANGE; statusSize = 20; }
            else { statusText = "Menggerakkan lengan menuju target..."; statusColor = PAL_INDIGO; statusSize = 18; }

            if (haveCustomFont) DrawTextEx(fontSemiBold, statusText, { (float)(PANEL_W + 20), 20 }, (float)statusSize, 0, statusColor);
            else DrawText(statusText, PANEL_W + 20, 20, statusSize, statusColor);
        }

        EndDrawing();
    }

    UnloadFont(fontRegular);
    UnloadFont(fontSemiBold);
    CloseWindow();
    return 0;
}
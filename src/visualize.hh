#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <implot3d.h>

#include "knot.hh"

template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
auto display_knots(const std::vector<PolyLine<RealT>> &knots) -> int
{
    // Initialize GLFW
    if (not glfwInit())
    {
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    // Create window
    auto *window = glfwCreateWindow(720, 640, "Unknot", nullptr, nullptr);
    if (not window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    // Load OpenGL functions using glad
    if (not gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        glfwTerminate();
        return -1;
    }

    // Setup context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();

    // Setup style
    ImGui::StyleColorsDark();
    ImPlot3D::GetStyle().LineWeight = 5;

    // Setup backend
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Main loop
    int which = 0.;
    float time = 0;
    float fps = 60;
    bool play = 0;
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (ImGui::Checkbox("Auto", &play))
        {
            time = ImGui::GetTime();
        }

        ImGui::SameLine();
        if (ImGui::Button("Prev"))
        {
            which = (which + knots.size() - 1) % knots.size();
        }

        ImGui::SameLine();
        if (ImGui::Button("Next"))
        {
            which = (which + 1) % knots.size();
        }

        ImGui::SliderInt("Index", &which, 0U, knots.size() - 1);
        ImGui::SliderFloat("FPS", &fps, 1.F, 60.F);

        if (ImPlot3D::BeginPlot("Knot", ImVec2(500, 500)))
        {
            ImPlot3D::SetupBoxScale(1., 1., 1.);
            ImPlot3D::SetupAxes(
                nullptr,
                nullptr,
                nullptr,
                ImPlot3DAxisFlags_NoDecorations,
                ImPlot3DAxisFlags_NoDecorations,
                ImPlot3DAxisFlags_NoDecorations);

            ImPlot3D::SetupAxesLimits(-2, 2, -2, 2, -2, 2);

            if (play)
            {
                which = static_cast<int>((ImGui::GetTime() - time) * fps);

                if (static_cast<std::size_t>(which) > knots.size())
                {
                    play = false;
                    which = knots.size() - 1;
                }
            }

            const auto &pl = knots[which];
            for (auto i = 0U; i < pl.size(); ++i)
            {
                const auto &[s1, s2] = pl.segment(i);

                const float xs[2] = {s1[0], s2[0]};
                const float ys[2] = {s1[1], s2[1]};
                const float zs[2] = {s1[2], s2[2]};

                ImGui::PushID(i);
                ImPlot3D::PlotLine("##Filled", xs, ys, zs, 2);
                ImGui::PopID();
            }

            ImPlot3D::EndPlot();
        }

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap buffers
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

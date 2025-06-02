#include <iostream>
#include <GLFW/glfw3.h>
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>

// ImGui includes
#include "../external/imgui/imgui.h"
#include "../external/imgui/backends/imgui_impl_glfw.h"
#include "../external/imgui/backends/imgui_impl_opengl3.h"

// Your includes
#include "parser/spice_parser.h"
#include "circuit_manager.h"

// Forward declaration
void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component);

int main() {
    std::cout << "Starting SPICE Simulator with ImGui..." << std::endl;
    
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1400, 900, "SPICE Circuit Simulator", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Application state
    bool show_circuit_editor = true;
    bool show_component_library = true;
    bool show_netlist_editor = true;
    bool simulate = false;
    bool wire_mode = false;
    CircuitElement* wire_start_component = nullptr;
    int wire_start_pin = -1;
    ImVec2 wire_start_pos;
    ImVec2 current_mouse_pos;
    SPICEParser parser;
    
    // Circuit management
    CircuitManager circuit;
    std::string current_netlist = "* Empty Circuit\n.end\n";
    std::string selected_component_type = "";
    bool placing_component = false;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Main menu bar - FIX: Add missing EndMainMenuBar()
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("New Circuit")) {
                    circuit.clear();
                    current_netlist = "* Empty Circuit\n.end\n";
                }
                if (ImGui::MenuItem("Exit")) {
                    glfwSetWindowShouldClose(window, true);
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Component Library", NULL, &show_component_library);
                ImGui::MenuItem("Circuit Editor", NULL, &show_circuit_editor);
                ImGui::MenuItem("Netlist Editor", NULL, &show_netlist_editor);
                ImGui::EndMenu();
            }
            if(ImGui::BeginMenu("Simulation")){
                ImGui::MenuItem("Simulate", "Ctrl+Enter", &simulate);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // Component Library Window
        if (show_component_library) {
            ImGui::Begin("Component Library", &show_component_library);
            
            ImGui::Text("Click to select, then click on canvas to place:");
            ImGui::Separator();
            
            if (ImGui::Button("Resistor", ImVec2(100, 30))) {
                selected_component_type = "resistor";
                placing_component = true;
            }
            
            if (ImGui::Button("Capacitor", ImVec2(100, 30))) {
                selected_component_type = "capacitor";
                placing_component = true;
            }
            
            if (ImGui::Button("DC Voltage", ImVec2(100, 30))) {
                selected_component_type = "vsource";
                placing_component = true;
            }
            if (ImGui::Button("Ground", ImVec2(100, 30))) {
                selected_component_type = "ground";
                placing_component = true;
            }
            ImGui::Separator();
            if (ImGui::Button("Wire Mode", ImVec2(100, 30))) {
                wire_mode = !wire_mode;
                placing_component = false;
                selected_component_type = "";
                wire_start_component = nullptr;
                wire_start_pin = -1;
            }
            
            ImGui::Separator();
            if (ImGui::Button("Clear Circuit")) {
                circuit.clear();
                current_netlist = "* Empty Circuit\n.end\n";
            }

            if (wire_mode) {
                ImGui::TextColored(ImVec4(0, 1, 1, 1), "Wire Mode: Click pins to connect");
                if (wire_start_component) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Click second pin to complete wire");
                }
            }
            
            if (placing_component) {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Click on canvas to place %s", selected_component_type.c_str());
            }
            
            ImGui::End();
        }

        // Circuit Editor Window  
        if (show_circuit_editor) {
            ImGui::Begin("Circuit Editor", &show_circuit_editor);
            
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
            ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
            if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
            if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
            ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

            // Draw background
            draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(30, 30, 30, 255));
            draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

            // Draw grid
            float grid_step = 20.0f;
            for (float x = std::fmod(canvas_p0.x, grid_step); x < canvas_sz.x; x += grid_step) {
                draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), 
                                  ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(50, 50, 50, 255));
            }
            for (float y = std::fmod(canvas_p0.y, grid_step); y < canvas_sz.y; y += grid_step) {
                draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), 
                                  ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(50, 50, 50, 255));
            }

            // Draw components
            for (const auto& component : circuit.getComponents()) {
                drawComponent(draw_list, canvas_p0, component.get());
            }

            // Handle mouse clicks
            ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft);
            current_mouse_pos = ImGui::GetMousePos();

            if (ImGui::IsItemClicked()) {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float rel_x = mouse_pos.x - canvas_p0.x;
                float rel_y = mouse_pos.y - canvas_p0.y;
                
                if (wire_mode) {
                    // Wire mode - try to connect pins
                    auto [component, pin] = circuit.findPinAt(rel_x, rel_y);
                    
                    if (component && pin != -1) {
                        if (!wire_start_component) {
                            // Start a new wire
                            wire_start_component = component;
                            wire_start_pin = pin;
                            auto pins = component->getAbsolutePinPositions();
                            wire_start_pos = ImVec2(canvas_p0.x + pins[pin].first, canvas_p0.y + pins[pin].second);
                            std::cout << "Starting wire from " << component->name << " pin " << pin << std::endl;
                        } else {
                            // Complete the wire
                            if (circuit.connectPins(wire_start_component, wire_start_pin, component, pin)) {
                                current_netlist = circuit.generateNetlist();
                                std::cout << "Wire completed!" << std::endl;
                            } else {
                                std::cout << "Cannot connect these pins" << std::endl;
                            }
                            wire_start_component = nullptr;
                            wire_start_pin = -1;
                        }
                    } else {
                        // Clicked on empty space - cancel wire
                        wire_start_component = nullptr;
                        wire_start_pin = -1;
                    }
                } else if (placing_component) {
                    // Component placement mode
                    rel_x = std::round(rel_x / grid_step) * grid_step;
                    rel_y = std::round(rel_y / grid_step) * grid_step;
                    
                    CircuitElement* new_component = circuit.addComponent(selected_component_type, rel_x, rel_y);
                    if (new_component) {
                        current_netlist = circuit.generateNetlist();
                        std::cout << "Added " << new_component->name << " at (" << rel_x << ", " << rel_y << ")" << std::endl;
                    }
                    placing_component = false;
                    selected_component_type = "";
                }
            }

            // Draw wires
            for (const auto& wire : circuit.getWires()) {
                auto start = wire->getStartPos();
                auto end = wire->getEndPos();
                
                draw_list->AddLine(
                    ImVec2(canvas_p0.x + start.first, canvas_p0.y + start.second),
                    ImVec2(canvas_p0.x + end.first, canvas_p0.y + end.second),
                    IM_COL32(0, 255, 0, 255), 2.0f
                );
            }

            // Draw temporary wire while in progress
            if (wire_mode && wire_start_component) {
                draw_list->AddLine(
                    wire_start_pos,
                    current_mouse_pos,
                    IM_COL32(255, 255, 0, 255), 2.0f
                );
            }
            
            ImGui::End();
        }

        // Netlist Editor Window
        if (show_netlist_editor) {
            ImGui::Begin("Netlist Editor", &show_netlist_editor);
            
            ImGui::Text("Generated SPICE Netlist:");
            ImGui::Separator();
            
            ImGui::TextWrapped("%s", current_netlist.c_str());
            
            ImGui::End();
        }

        if(simulate){
            // Run and show simulation results
            std::cout << "Starting simulation..." << std::endl;
            
            try {
                // Create a temporary netlist file
                std::ofstream temp_file("temp_circuit.cir");
                temp_file << current_netlist;
                temp_file.close();
                
                // Parse and simulate using your existing SPICE parser
                parser.parseFile("temp_circuit.cir");
                parser.printParsedElements();
                
                std::cout << "Simulation completed successfully!" << std::endl;
                
                // Clean up temporary file
                std::remove("temp_circuit.cir");
                
            } catch (const std::exception& e) {
                std::cerr << "Simulation error: " << e.what() << std::endl;
            }
            simulate = false;
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();

    return 0;
}

void drawPins(CircuitElement* component, ImVec2 canvas_offset, ImDrawList* draw_list) {
    for (int i = 0; i < component->getPinCount(); ++i) {
        auto positions = component->getAbsolutePinPositions();
        ImVec2 pin_pos(canvas_offset.x + positions[i].first, canvas_offset.y + positions[i].second);
        
        int node_id = component->getNodeForPin(i);
        
        ImU32 color;
        if (component->getType() == "ground") {
            color = IM_COL32(0, 0, 0, 255);
        } else if (node_id == -1) {
            color = IM_COL32(255, 0, 0, 255);
        } else if (node_id == 0) {
            color = IM_COL32(0, 0, 0, 255);
        } else {
            color = IM_COL32(0, 255, 0, 255);
        }
        
        draw_list->AddCircleFilled(pin_pos, 4.0f, color);
        draw_list->AddCircle(pin_pos, 4.0f, IM_COL32(255, 255, 255, 255), 8, 1.0f);
    }
}
// Component drawing function
void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component) {
    ImVec2 pos(canvas_offset.x + component->x, canvas_offset.y + component->y);
    
    if (component->getType() == "resistor") {
        // Draw resistor symbol
        draw_list->AddLine(ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 15, pos.y), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddRect(ImVec2(pos.x - 15, pos.y - 5), ImVec2(pos.x + 15, pos.y + 5), IM_COL32(255, 255, 255, 255), 0.0f, 0, 2.0f);
        draw_list->AddLine(ImVec2(pos.x + 15, pos.y), ImVec2(pos.x + 25, pos.y), IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Draw labels
        draw_list->AddText(ImVec2(pos.x - 10, pos.y - 20), IM_COL32(255, 255, 0, 255), component->name.c_str());
        draw_list->AddText(ImVec2(pos.x - 10, pos.y + 10), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        // Draw connection points
       drawPins(component, canvas_offset, draw_list);
    } else if (component->getType() == "capacitor") {
        // Draw capacitor symbol (two parallel lines)
        draw_list->AddLine(ImVec2(pos.x - 20, pos.y), ImVec2(pos.x - 5, pos.y), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x - 5, pos.y - 10), ImVec2(pos.x - 5, pos.y + 10), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x + 5, pos.y - 10), ImVec2(pos.x + 5, pos.y + 10), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x + 5, pos.y), ImVec2(pos.x + 20, pos.y), IM_COL32(255, 255, 255, 255), 2.0f);
        
        draw_list->AddText(ImVec2(pos.x - 10, pos.y - 20), IM_COL32(255, 255, 0, 255), component->name.c_str());
        draw_list->AddText(ImVec2(pos.x - 10, pos.y + 10), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
    } else if (component->getType() == "vsource") {
        // Draw voltage source symbol (circle)
        draw_list->AddCircle(pos, 15.0f, IM_COL32(255, 255, 255, 255), 16, 2.0f);
        draw_list->AddLine(ImVec2(pos.x, pos.y - 20), ImVec2(pos.x, pos.y - 15), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x, pos.y + 15), ImVec2(pos.x, pos.y + 20), IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Draw + and - symbols
        draw_list->AddText(ImVec2(pos.x - 3, pos.y - 25), IM_COL32(255, 255, 255, 255), "+");
        draw_list->AddText(ImVec2(pos.x - 3, pos.y + 15), IM_COL32(255, 255, 255, 255), "-");
        
        draw_list->AddText(ImVec2(pos.x - 10, pos.y - 35), IM_COL32(255, 255, 0, 255), component->name.c_str());
        draw_list->AddText(ImVec2(pos.x - 10, pos.y + 25), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
    } else if (component->getType() == "ground") {
        // Draw ground symbol
        draw_list->AddLine(ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + 10), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x - 8, pos.y + 10), ImVec2(pos.x + 8, pos.y + 10), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x - 5, pos.y + 13), ImVec2(pos.x + 5, pos.y + 13), IM_COL32(255, 255, 255, 255), 2.0f);
        draw_list->AddLine(ImVec2(pos.x - 2, pos.y + 16), ImVec2(pos.x + 2, pos.y + 16), IM_COL32(255, 255, 255, 255), 2.0f);
        
        draw_list->AddText(ImVec2(pos.x - 10, pos.y - 20), IM_COL32(255, 255, 0, 255), component->name.c_str());
        
        drawPins(component, canvas_offset, draw_list);
    }
}
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
void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component,  CircuitElement* selected_component);
void drawComponentPreview(ImDrawList* draw_list, ImVec2 pos, const std::string& component_type, int rotation);

ImVec2 snapToGrid(ImVec2 pos, float grid_step){
    return ImVec2(std::round(pos.x / grid_step) * grid_step, std::round(pos.y / grid_step) * grid_step);
}

struct WireSegment {
    ImVec2 start;
    ImVec2 end;
};

// Simulation settings structures (renamed to avoid conflicts)
struct DCSimSettings {
    bool enabled = true;
    // DC analysis doesn't need additional parameters
};

struct TransientSimSettings {
    bool enabled = false;
    double stepTime = 1e-9;     // 1ns default
    double stopTime = 1e-6;     // 1us default
    double startTime = 0.0;
    char stepTimeStr[32] = "1n";
    char stopTimeStr[32] = "1u";
    char startTimeStr[32] = "0";
};

struct ACSimSettings {
    bool enabled = false;
    double startFreq = 1.0;
    double stopFreq = 1e6;
    int pointsPerDecade = 10;
    char startFreqStr[32] = "1";
    char stopFreqStr[32] = "1meg";
    char pointsStr[32] = "10";
    enum Type { DEC, OCT, LIN } type = DEC;
};

struct DCSweepSettings {
    bool enabled = false;
    char sourceName[64] = "V1";
    double startValue = 0.0;
    double stopValue = 5.0;
    double stepValue = 0.1;
    char startStr[32] = "0";
    char stopStr[32] = "5";
    char stepStr[32] = "0.1";
};

// Simulation control variables
struct SimulationConfig {
    DCSimSettings dc;
    TransientSimSettings transient;
    ACSimSettings ac;
    DCSweepSettings dcSweep;
} simConfig;

// Dialog control
bool show_simulation_dialog = false;

double parseEngValue(const std::string& str) {
    if (str.empty()) return 0.0;
    
    std::string s = str;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    
    double multiplier = 1.0;
    if (s.back() >= 'a' && s.back() <= 'z') {
        char suffix = s.back();
        s.pop_back();
        
        switch (suffix) {
            case 't': multiplier = 1e12; break;
            case 'g': multiplier = 1e9; break;
            case 'k': multiplier = 1e3; break;
            case 'm':
                if (s.length() >= 2 && s.substr(s.length()-2) == "me") {
                    multiplier = 1e6; // meg
                    s = s.substr(0, s.length()-2);
                } else {
                    multiplier = 1e-3; // m
                }
                break;
            case 'u': multiplier = 1e-6; break;
            case 'n': multiplier = 1e-9; break;
            case 'p': multiplier = 1e-12; break;
            case 'f': multiplier = 1e-15; break;
        }
    }
    
    try {
        return std::stod(s) * multiplier;
    } catch (...) {
        return 0.0;
    }
}

// Generate simulation commands based on settings
std::string generateSimulationCommands(const SimulationConfig& config) {
    std::string commands;
    
    if (config.dc.enabled) {
        commands += ".op\n";
    }
    
    if (config.transient.enabled) {
        double step = parseEngValue(config.transient.stepTimeStr);
        double stop = parseEngValue(config.transient.stopTimeStr);
        double start = parseEngValue(config.transient.startTimeStr);
        
        commands += ".tran " + std::string(config.transient.stepTimeStr) + " " + 
                   std::string(config.transient.stopTimeStr);
        if (start != 0.0) {
            commands += " " + std::string(config.transient.startTimeStr);
        }
        commands += "\n";
    }
    
    if (config.ac.enabled) {
        std::string typeStr = (config.ac.type == ACSimSettings::DEC) ? "dec" :
                             (config.ac.type == ACSimSettings::OCT) ? "oct" : "lin";
        commands += ".ac " + typeStr + " " + std::string(config.ac.pointsStr) + " " +
                   std::string(config.ac.startFreqStr) + " " + std::string(config.ac.stopFreqStr) + "\n";
    }
    
    if (config.dcSweep.enabled) {
        commands += ".dc " + std::string(config.dcSweep.sourceName) + " " +
                   std::string(config.dcSweep.startStr) + " " + std::string(config.dcSweep.stopStr) + 
                   " " + std::string(config.dcSweep.stepStr) + "\n";
    }
    
    return commands;
}

std::string generateNetlistWithSettings(const CircuitManager& circuit, const SimulationConfig& config) {
    std::string netlist = "* Generated SPICE Netlist\n";
    
    // Add components (only fully connected ones)
    for (const auto& component : circuit.getComponents()) {
        if (component->getType() == "ground") continue;  // Skip ground symbols
        
        if (component->isFullyConnected()) {
            std::string line = component->toSpiceLine();
            if (!line.empty()) {
                netlist += line + "\n";
            }
        } else {
            netlist += "* " + component->name + " not fully connected (" 
                     + std::to_string(component->getUnconnectedPinCount()) 
                     + " unconnected pins)\n";
        }
    }
    
    // Add simulation commands
    netlist += generateSimulationCommands(config);
    netlist += ".end\n";
    
    return netlist;
}

void ShowComponentEditDialog(CircuitElement* component, bool* show_dialog, char* edit_name_buffer, char* edit_value_buffer) {
    if (!*show_dialog || !component) return;
    
    ImGui::SetNextWindowSize(ImVec2(350, 250), ImGuiCond_FirstUseEver);
    std::string title = "Edit " + component->name;
    
    if (ImGui::Begin(title.c_str(), show_dialog)) {
        ImGui::Text("Component: %s", component->name.c_str());
        ImGui::Text("Type: %s", component->getType().c_str());
        ImGui::Separator();
        
        // Component name editing
        ImGui::Text("Name:");
        ImGui::SameLine();
        if (ImGui::InputText("##name", edit_name_buffer, 64)) {
            // Name is updated when Apply is clicked
        }
        
        // Component value editing
        ImGui::Text("Value:");
        ImGui::SameLine();
        if (ImGui::InputText("##value", edit_value_buffer, 64)) {
            // Value is updated when Apply is clicked
        }
        
        // Rotation control
        ImGui::Text("Rotation: %d°", component->getRotation());
        if (ImGui::Button("Rotate 90°")) {
            component->rotate90();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Rotation")) {
            component->setRotation(0);
        }
        
        // Position info (read-only)
        ImGui::Separator();
        ImGui::Text("Position: (%.0f, %.0f)", component->x, component->y);
        ImGui::Text("Pins: %d", component->getPinCount());
        
        // Connection status
        ImGui::Text("Connections:");
        for (int i = 0; i < component->getPinCount(); i++) {
            int node = component->getNodeForPin(i);
            if (node == -1) {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "  %s: Unconnected", 
                                  component->getPinName(i).c_str());
            } else {
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "  %s: Node %d", 
                                  component->getPinName(i).c_str(), node);
            }
        }
        
        ImGui::Separator();
        
        // Buttons
        if (ImGui::Button("Apply")) {
            // Apply name change
            std::string new_name = std::string(edit_name_buffer);
            if (!new_name.empty() && new_name != component->name) {
                component->name = new_name;
            }
            
            // Apply value change
            std::string new_value = std::string(edit_value_buffer);
            if (!new_value.empty()) {
                try {
                    component->setValue(new_value);
                    std::cout << "Updated " << component->name << " value to " << new_value << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Error setting value: " << e.what() << std::endl;
                }
            }
            
            *show_dialog = false;
        }
        ImGui::SameLine();
        
        if (ImGui::Button("Cancel")) {
            *show_dialog = false;
        }
        ImGui::SameLine();
        
        if (ImGui::Button("Delete Component")) {
            std::cout << "Delete component requested: " << component->name << std::endl;
            *show_dialog = false;
            // TODO: Add deletion functionality
        }
    }
    ImGui::End();
}

void ShowSimulationDialog(SimulationConfig& config, bool* show_dialog) {
    if (!*show_dialog) return;
    
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Simulation Settings", show_dialog)) {
        
        if (ImGui::BeginTabBar("SimulationTabs")) {
            
            // DC Analysis Tab
            if (ImGui::BeginTabItem("DC Analysis")) {
                ImGui::Checkbox("Enable DC Operating Point Analysis", &config.dc.enabled);
                ImGui::Separator();
                
                if (config.dc.enabled) {
                    ImGui::TextWrapped("DC operating point analysis calculates the steady-state voltages and currents in the circuit.");
                    ImGui::Text("Command: .op");
                } else {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "DC analysis disabled");
                }
                
                ImGui::EndTabItem();
            }
            
            // Transient Analysis Tab
            if (ImGui::BeginTabItem("Transient")) {
                ImGui::Checkbox("Enable Transient Analysis", &config.transient.enabled);
                ImGui::Separator();
                
                if (config.transient.enabled) {
                    ImGui::Text("Time Parameters:");
                    ImGui::InputText("Step Time", config.transient.stepTimeStr, sizeof(config.transient.stepTimeStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 1n, 10u, 1m)");
                    
                    ImGui::InputText("Stop Time", config.transient.stopTimeStr, sizeof(config.transient.stopTimeStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 1u, 10m, 1)");
                    
                    ImGui::InputText("Start Time", config.transient.startTimeStr, sizeof(config.transient.startTimeStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(optional, default 0)");
                    
                    ImGui::Separator();
                    ImGui::TextWrapped("Transient analysis simulates circuit behavior over time.");
                    
                    // Show preview of command
                    std::string cmd = ".tran " + std::string(config.transient.stepTimeStr) + " " + 
                                     std::string(config.transient.stopTimeStr);
                    if (parseEngValue(config.transient.startTimeStr) != 0.0) {
                        cmd += " " + std::string(config.transient.startTimeStr);
                    }
                    ImGui::Text("Command: %s", cmd.c_str());
                } else {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Transient analysis disabled");
                }
                
                ImGui::EndTabItem();
            }
            
            // AC Analysis Tab
            if (ImGui::BeginTabItem("AC Analysis")) {
                ImGui::Checkbox("Enable AC Analysis", &config.ac.enabled);
                ImGui::Separator();
                
                if (config.ac.enabled) {
                    ImGui::Text("Frequency Sweep:");
                    
                    // Sweep type
                    const char* types[] = { "Decade", "Octave", "Linear" };
                    int currentType = (int)config.ac.type;
                    if (ImGui::Combo("Sweep Type", &currentType, types, 3)) {
                        config.ac.type = (ACSimSettings::Type)currentType;
                    }
                    
                    ImGui::InputText("Start Frequency", config.ac.startFreqStr, sizeof(config.ac.startFreqStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 1, 10k, 1meg)");
                    
                    ImGui::InputText("Stop Frequency", config.ac.stopFreqStr, sizeof(config.ac.stopFreqStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 1meg, 1g)");
                    
                    ImGui::InputText("Points per Decade/Octave", config.ac.pointsStr, sizeof(config.ac.pointsStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 10, 50)");
                    
                    ImGui::Separator();
                    ImGui::TextWrapped("AC analysis calculates frequency response (magnitude and phase).");
                    
                    // Show preview of command
                    std::string typeStr = (config.ac.type == ACSimSettings::DEC) ? "dec" :
                                         (config.ac.type == ACSimSettings::OCT) ? "oct" : "lin";
                    std::string cmd = ".ac " + typeStr + " " + std::string(config.ac.pointsStr) + " " +
                                     std::string(config.ac.startFreqStr) + " " + std::string(config.ac.stopFreqStr);
                    ImGui::Text("Command: %s", cmd.c_str());
                } else {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "AC analysis disabled");
                }
                
                ImGui::EndTabItem();
            }
            
            // DC Sweep Tab
            if (ImGui::BeginTabItem("DC Sweep")) {
                ImGui::Checkbox("Enable DC Sweep Analysis", &config.dcSweep.enabled);
                ImGui::Separator();
                
                if (config.dcSweep.enabled) {
                    ImGui::Text("Source Sweep Parameters:");
                    ImGui::InputText("Source Name", config.dcSweep.sourceName, sizeof(config.dcSweep.sourceName));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., V1, VDD)");
                    
                    ImGui::InputText("Start Value", config.dcSweep.startStr, sizeof(config.dcSweep.startStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 0, -5)");
                    
                    ImGui::InputText("Stop Value", config.dcSweep.stopStr, sizeof(config.dcSweep.stopStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 5, 10)");
                    
                    ImGui::InputText("Step Value", config.dcSweep.stepStr, sizeof(config.dcSweep.stepStr));
                    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(e.g., 0.1, 0.01)");
                    
                    ImGui::Separator();
                    ImGui::TextWrapped("DC sweep varies a voltage/current source and analyzes circuit response.");
                    
                    // Show preview of command
                    std::string cmd = ".dc " + std::string(config.dcSweep.sourceName) + " " +
                                     std::string(config.dcSweep.startStr) + " " + std::string(config.dcSweep.stopStr) + 
                                     " " + std::string(config.dcSweep.stepStr);
                    ImGui::Text("Command: %s", cmd.c_str());
                } else {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "DC sweep analysis disabled");
                }
                
                ImGui::EndTabItem();
            }
            
            ImGui::EndTabBar();
        }
        
        ImGui::Separator();
        
        // Buttons
        if (ImGui::Button("Apply Settings")) {
            *show_dialog = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            *show_dialog = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset to Defaults")) {
            config = SimulationConfig{}; // Reset to default values
        }
        
        // Show summary of enabled analyses
        ImGui::Separator();
        ImGui::Text("Enabled Analyses:");
        if (config.dc.enabled) ImGui::BulletText("DC Operating Point");
        if (config.transient.enabled) ImGui::BulletText("Transient Analysis");
        if (config.ac.enabled) ImGui::BulletText("AC Analysis");
        if (config.dcSweep.enabled) ImGui::BulletText("DC Sweep");
        
        if (!config.dc.enabled && !config.transient.enabled && !config.ac.enabled && !config.dcSweep.enabled) {
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "No analyses enabled!");
        }
    }
    ImGui::End();
}

// Circuit management
CircuitManager circuit;
std::string current_netlist = "* Empty Circuit\n.end\n";
std::string selected_component_type = "";
bool placing_component = false;
CircuitElement* selected_component = nullptr;
bool dragging_component = false;
ImVec2 drag_start_pos;
ImVec2 component_start_pos;
bool show_component_preview = false;
ImVec2 preview_position;
int preview_rotation = 0;


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
    bool show_netlist_editor = false;
    bool simulate = false;
    bool wire_mode = false;
    CircuitElement* wire_start_component = nullptr;
    int wire_start_pin = -1;
    ImVec2 wire_start_pos;
    ImVec2 current_mouse_pos;
    SPICEParser parser;
    std::vector<WireSegment> current_wire_segments;
    bool wire_horizontal_first = true;
    

    // Component Editor
    bool show_component_edit_dialog = false;
    CircuitElement* editing_component = nullptr;
    char edit_value_buffer[64] = "";
    char edit_name_buffer[64] = "";

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Main menu bar
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
                if (ImGui::MenuItem("Simulation Settings")) {
                    show_simulation_dialog = true;
                }
                ImGui::MenuItem("Simulate", "Ctrl+Enter", &simulate);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ImVec2 main_window_size = ImGui::GetIO().DisplaySize;
        float menu_bar_height = ImGui::GetFrameHeight();

        // Define layout parameters
        float component_library_width = 200.0f;
        float window_padding = 8.0f;

        // Component Library Window - Fixed to right side
        if (show_component_library) {
            ImGui::SetNextWindowPos(ImVec2(main_window_size.x - component_library_width - window_padding, 
                                        menu_bar_height + window_padding));
            ImGui::SetNextWindowSize(ImVec2(component_library_width, 
                                            main_window_size.y - menu_bar_height - window_padding * 2));
            
            ImGui::Begin("Component Library", &show_component_library, 
                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
            
            ImGui::Text("Click to select, then click on canvas to place:");
            ImGui::Separator();
            
            if (ImGui::Button("Resistor", ImVec2(-1, 30))) {
                selected_component_type = "resistor";
                placing_component = true;
            }
            
            if (ImGui::Button("Capacitor", ImVec2(-1, 30))) {
                selected_component_type = "capacitor";
                placing_component = true;
            }
            
            if (ImGui::Button("Inductor", ImVec2(-1, 30))) {
                selected_component_type = "inductor";
                placing_component = true;
            }

            if (ImGui::Button("Diode", ImVec2(-1, 30))) {
                selected_component_type = "diode";
                placing_component = true;
            }

            if (ImGui::Button("NMOS", ImVec2(-1, 30))) {
                selected_component_type = "nmosfet";
                placing_component = true;
            }
            
            if (ImGui::Button("PMOS", ImVec2(-1, 30))) {
                selected_component_type = "pmosfet";
                placing_component = true;
            }
            
            if (ImGui::Button("DC Voltage", ImVec2(-1, 30))) {
                selected_component_type = "vsource";
                placing_component = true;
            }
            
            if (ImGui::Button("Ground", ImVec2(-1, 30))) {
                selected_component_type = "ground";
                placing_component = true;
            }
            
            ImGui::Separator();
            
            if (ImGui::Button("Wire Mode", ImVec2(-1, 30))) {
                wire_mode = !wire_mode;
                placing_component = false;
                selected_component_type = "";
                wire_start_component = nullptr;
                wire_start_pin = -1;
            }
            
            ImGui::Separator();
            
            if (ImGui::Button("Clear Circuit", ImVec2(-1, 30))) {
                circuit.clear();
                current_netlist = "* Empty Circuit\n.end\n";
            }

            ImGui::Separator();
            
            // Status information
            if (wire_mode) {
                ImGui::TextColored(ImVec4(0, 1, 1, 1), "Wire Mode Active");
                if (wire_start_component) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Click second pin");
                } else {
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Click first pin");
                }
            }
            
            if (placing_component) {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Placing:");
                ImGui::TextWrapped("%s", selected_component_type.c_str());
            }
            
            // Circuit statistics
            ImGui::Separator();
            ImGui::Text("Circuit Info:");
            ImGui::Text("Components: %zu", circuit.getComponents().size());
            ImGui::Text("Wires: %zu", circuit.getWires().size());
            ImGui::Text("Nodes: %d", circuit.getNodeCount());
            
            ImGui::End();
        }

        // Circuit Editor Window - Takes remaining space on the left
        if (show_circuit_editor) {
            float circuit_editor_width = show_component_library ? 
                main_window_size.x - component_library_width - window_padding * 3 : 
                main_window_size.x - window_padding * 2;
            
            ImGui::SetNextWindowPos(ImVec2(window_padding, menu_bar_height + window_padding));
            ImGui::SetNextWindowSize(ImVec2(circuit_editor_width, 
                                            main_window_size.y - menu_bar_height - window_padding * 2));
            
            ImGui::Begin("Circuit Editor", &show_circuit_editor, 
                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
            
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
                drawComponent(draw_list, canvas_p0, component.get(), selected_component);
            }

            if (show_component_preview && placing_component && !selected_component_type.empty()) {
                ImVec2 relative_preview_pos = ImVec2(preview_position.x - canvas_p0.x, preview_position.y - canvas_p0.y);
                drawComponentPreview(draw_list, preview_position, selected_component_type, preview_rotation);
            }


            // Handle mouse clicks and interactions
            ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
            current_mouse_pos = ImGui::GetMousePos();

            // Update preview position when placing components
            if (placing_component) {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float rel_x = mouse_pos.x - canvas_p0.x;
                float rel_y = mouse_pos.y - canvas_p0.y;
                
                // Snap preview position to grid
                ImVec2 snapped_pos = snapToGrid(ImVec2(rel_x, rel_y), grid_step);
                preview_position = ImVec2(canvas_p0.x + snapped_pos.x, canvas_p0.y + snapped_pos.y);
                show_component_preview = true;
            } else {
                show_component_preview = false;
            }

            // Check for keyboard shortcuts
            if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                if (placing_component) {
                    // Cancel component placement
                    placing_component = false;
                    selected_component_type = "";
                    show_component_preview = false;
                    std::cout << "Component placement cancelled" << std::endl;
                } else if (wire_mode && wire_start_component) {
                    // Cancel wire placement
                    wire_start_component = nullptr;
                    wire_start_pin = -1;
                    current_wire_segments.clear();
                    std::cout << "Wire placement cancelled" << std::endl;
                } else if (selected_component) {
                    // Deselect component
                    selected_component = nullptr;
                    std::cout << "Component deselected" << std::endl;
                }
            }

            if (ImGui::IsKeyPressed(ImGuiKey_R)) {
                if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && selected_component) {
                    // Rotate selected component
                    selected_component->rotate90();
                    std::cout << "Rotated " << selected_component->name << " to " << selected_component->getRotation() << "°" << std::endl;
                } else if (placing_component) {
                    // Rotate preview component
                    preview_rotation = (preview_rotation + 90) % 360;
                    std::cout << "Preview rotation: " << preview_rotation << "°" << std::endl;
                }
            }

            // Handle right-click for component editing
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float rel_x = mouse_pos.x - canvas_p0.x;
                float rel_y = mouse_pos.y - canvas_p0.y;
                
                // Find component at mouse position
                CircuitElement* clicked_component = nullptr;
                for (const auto& component : circuit.getComponents()) {
                    if (component->isPointInside(rel_x, rel_y)) {
                        clicked_component = component.get();
                        break;
                    }
                }
                
                if (clicked_component) {
                    editing_component = clicked_component;
                    // Initialize edit buffers with current values
                    strncpy(edit_name_buffer, clicked_component->name.c_str(), sizeof(edit_name_buffer) - 1);
                    strncpy(edit_value_buffer, clicked_component->getValue().c_str(), sizeof(edit_value_buffer) - 1);
                    show_component_edit_dialog = true;
                    std::cout << "Right-clicked on " << clicked_component->name << std::endl;
                }
            }

            // Handle left-click for selection and interaction
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float rel_x = mouse_pos.x - canvas_p0.x;
                float rel_y = mouse_pos.y - canvas_p0.y;
                
                if (placing_component) {
                    // Component placement mode with grid snapping
                    ImVec2 snapped_pos = snapToGrid(ImVec2(rel_x, rel_y), grid_step);
                    
                    std::cout << "Attempting to place component: " << selected_component_type << std::endl;
                    
                    CircuitElement* new_component = circuit.addComponent(selected_component_type, snapped_pos.x, snapped_pos.y);
                    if (new_component) {
                        new_component->setRotation(preview_rotation);

                        current_netlist = circuit.generateNetlist();
                        std::cout << "Added " << new_component->name << " at (" << snapped_pos.x << ", " << snapped_pos.y << ")" << std::endl;
                    } else {
                        std::cout << "Failed to add component: " << selected_component_type << std::endl;
                    }
                    
                    // Keep placement mode active (user can place multiple components)
                    // To exit placement mode, they need to press Escape or click a different button
                    preview_rotation = 0;
                    show_component_preview = true; // Keep showing preview for next placement
                    
                } else if (wire_mode) {
                    // Wire mode - enhanced routing
                    auto [component, pin] = circuit.findPinAt(rel_x, rel_y);
                    
                    if (component && pin != -1) {
                        // Clicked on a pin
                        if (!wire_start_component) {
                            // Start a new wire
                            wire_start_component = component;
                            wire_start_pin = pin;
                            auto pins = component->getAbsolutePinPositions();
                            wire_start_pos = ImVec2(canvas_p0.x + pins[pin].first, canvas_p0.y + pins[pin].second);
                            current_wire_segments.clear();
                            std::cout << "Starting wire from " << component->name << " pin " << pin << std::endl;
                        } else {
                            // Complete the wire - connect to end pin
                            if (circuit.connectPins(wire_start_component, wire_start_pin, component, pin)) {
                                current_netlist = circuit.generateNetlist();
                                std::cout << "Wire completed!" << std::endl;
                            } else {
                                std::cout << "Cannot connect these pins" << std::endl;
                            }
                            wire_start_component = nullptr;
                            wire_start_pin = -1;
                            current_wire_segments.clear();
                        }
                    } else if (wire_start_component) {
                        // Clicked on empty space while routing - add a waypoint and switch direction
                        ImVec2 snapped_click = snapToGrid(ImVec2(rel_x, rel_y), grid_step);
                        ImVec2 snapped_click_canvas = ImVec2(canvas_p0.x + snapped_click.x, canvas_p0.y + snapped_click.y);
                        
                        ImVec2 current_start = current_wire_segments.empty() ? wire_start_pos : current_wire_segments.back().end;
                        
                        // Calculate the corner point based on current direction preference
                        ImVec2 corner_point;
                        if (wire_horizontal_first) {
                            // Go horizontal first, then vertical
                            corner_point = ImVec2(snapped_click_canvas.x, current_start.y);
                        } else {
                            // Go vertical first, then horizontal
                            corner_point = ImVec2(current_start.x, snapped_click_canvas.y);
                        }
                        
                        // Add segments to reach the clicked point
                        if (corner_point.x != current_start.x || corner_point.y != current_start.y) {
                            current_wire_segments.push_back({current_start, corner_point});
                        }
                        if (corner_point.x != snapped_click_canvas.x || corner_point.y != snapped_click_canvas.y) {
                            current_wire_segments.push_back({corner_point, snapped_click_canvas});
                        }
                        
                        // Switch direction preference for next segment
                        wire_horizontal_first = !wire_horizontal_first;
                        
                        std::cout << "Added wire waypoint at (" << snapped_click.x << ", " << snapped_click.y << ")" << std::endl;
                    } else {
                        // Clicked on empty space with no active wire - cancel any active wire
                        wire_start_component = nullptr;
                        wire_start_pin = -1;
                        current_wire_segments.clear();
                    }
                } else {
                    // Normal mode - component selection and movement
                    CircuitElement* clicked_component = nullptr;
                    
                    // Check if we clicked on a component
                    for (const auto& component : circuit.getComponents()) {
                        if (component->isPointInside(rel_x, rel_y)) {
                            clicked_component = component.get();
                            break;
                        }
                    }
                    
                    if (clicked_component) {
                        if (selected_component == clicked_component) {
                            // Already selected - start dragging
                            dragging_component = true;
                            drag_start_pos = ImVec2(rel_x, rel_y);
                            component_start_pos = ImVec2(clicked_component->x, clicked_component->y);
                            std::cout << "Started dragging " << clicked_component->name << std::endl;
                        } else {
                            // Select the component
                            selected_component = clicked_component;
                            std::cout << "Selected " << clicked_component->name << std::endl;
                        }
                    } else {
                        // Clicked on empty space - deselect
                        selected_component = nullptr;
                        dragging_component = false;
                    }
                }
            }

            // Handle dragging
            if (dragging_component && selected_component && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float rel_x = mouse_pos.x - canvas_p0.x;
                float rel_y = mouse_pos.y - canvas_p0.y;
                
                // Calculate new position with grid snapping
                ImVec2 new_pos = ImVec2(
                    component_start_pos.x + (rel_x - drag_start_pos.x),
                    component_start_pos.y + (rel_y - drag_start_pos.y)
                );
                ImVec2 snapped_pos = snapToGrid(new_pos, grid_step);
                
                selected_component->setPosition(snapped_pos.x, snapped_pos.y);
            } else if (dragging_component) {
                // Finished dragging
                dragging_component = false;
                std::cout << "Finished dragging " << selected_component->name << 
                            " to (" << selected_component->x << ", " << selected_component->y << ")" << std::endl;
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

        // Show component edit dialog
        ShowComponentEditDialog(editing_component, &show_component_edit_dialog, edit_name_buffer, edit_value_buffer);

        // Show simulation dialog  
        ShowSimulationDialog(simConfig, &show_simulation_dialog);

        // Netlist Editor Window
        if (show_netlist_editor) {
            ImGui::Begin("Netlist Editor", &show_netlist_editor);
            
            ImGui::Text("Generated SPICE Netlist:");
            ImGui::Separator();
            
            std::string display_netlist = generateNetlistWithSettings(circuit, simConfig);
            ImGui::TextWrapped("%s", display_netlist.c_str());
            
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

ImVec2 rotatePoint(ImVec2 point, ImVec2 center, int rotation) {
    if (rotation == 0) return point;
    
    float dx = point.x - center.x;
    float dy = point.y - center.y;
    
    float cos_angle, sin_angle;
    switch (rotation) {
        case 90:
            cos_angle = 0; sin_angle = 1;
            break;
        case 180:
            cos_angle = -1; sin_angle = 0;
            break;
        case 270:
            cos_angle = 0; sin_angle = -1;
            break;
        default:
            cos_angle = 1; sin_angle = 0;
            break;
    }
    
    float new_dx = dx * cos_angle - dy * sin_angle;
    float new_dy = dx * sin_angle + dy * cos_angle;
    
    return ImVec2(center.x + new_dx, center.y + new_dy);
}

// Helper function to draw a rotated line
void drawRotatedLine(ImDrawList* draw_list, ImVec2 start, ImVec2 end, ImVec2 center, int rotation, ImU32 color, float thickness = 1.0f) {
    ImVec2 rotated_start = rotatePoint(start, center, rotation);
    ImVec2 rotated_end = rotatePoint(end, center, rotation);
    draw_list->AddLine(rotated_start, rotated_end, color, thickness);
}

// Helper function to draw a rotated rectangle
void drawRotatedRect(ImDrawList* draw_list, ImVec2 min, ImVec2 max, ImVec2 center, int rotation, ImU32 color, float thickness = 1.0f) {
    ImVec2 corners[4] = {
        ImVec2(min.x, min.y),  // top-left
        ImVec2(max.x, min.y),  // top-right
        ImVec2(max.x, max.y),  // bottom-right
        ImVec2(min.x, max.y)   // bottom-left
    };
    
    for (int i = 0; i < 4; i++) {
        corners[i] = rotatePoint(corners[i], center, rotation);
    }
    
    for (int i = 0; i < 4; i++) {
        draw_list->AddLine(corners[i], corners[(i + 1) % 4], color, thickness);
    }
}

// Helper function to draw rotated text (we'll use offset positioning)
void drawRotatedText(ImDrawList* draw_list, ImVec2 pos, ImVec2 center, int rotation, ImU32 color, const char* text) {
    ImVec2 rotated_pos = rotatePoint(pos, center, rotation);
    draw_list->AddText(rotated_pos, color, text);
}

// Function to draw component preview (ghost/translucent version) with rotation
void drawComponentPreview(ImDrawList* draw_list, ImVec2 pos, const std::string& component_type, int rotation = 0) {
    // Use a translucent color for preview
    ImU32 preview_color = IM_COL32(255, 255, 255, 100);  // Semi-transparent white
    ImU32 preview_text_color = IM_COL32(255, 255, 0, 120);  // Semi-transparent yellow
    
    // Helper lambda for rotating preview elements
    auto rotatePreviewPoint = [&](ImVec2 point) -> ImVec2 {
        return rotatePoint(point, pos, rotation);
    };
    
    // Helper lambda for drawing rotated preview lines
    auto drawPreviewLine = [&](ImVec2 start, ImVec2 end, float thickness = 2.0f) {
        draw_list->AddLine(rotatePreviewPoint(start), rotatePreviewPoint(end), preview_color, thickness);
    };
    
    // Helper lambda for drawing rotated preview rectangles
    auto drawPreviewRect = [&](ImVec2 min, ImVec2 max, float thickness = 2.0f) {
        ImVec2 corners[4] = {
            ImVec2(min.x, min.y),
            ImVec2(max.x, min.y),
            ImVec2(max.x, max.y),
            ImVec2(min.x, max.y)
        };
        
        for (int i = 0; i < 4; i++) {
            corners[i] = rotatePreviewPoint(corners[i]);
        }
        
        for (int i = 0; i < 4; i++) {
            draw_list->AddLine(corners[i], corners[(i + 1) % 4], preview_color, thickness);
        }
    };
    
    // Helper lambda for drawing rotated preview text
    auto drawPreviewText = [&](ImVec2 text_pos, const char* text) {
        ImVec2 rotated_pos = rotatePreviewPoint(text_pos);
        draw_list->AddText(rotated_pos, preview_text_color, text);
    };
    
    if (component_type == "resistor") {
        // Draw resistor preview with rotation
        drawPreviewLine(ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 15, pos.y));
        drawPreviewRect(ImVec2(pos.x - 15, pos.y - 5), ImVec2(pos.x + 15, pos.y + 5));
        drawPreviewLine(ImVec2(pos.x + 15, pos.y), ImVec2(pos.x + 25, pos.y));
        drawPreviewText(ImVec2(pos.x - 10, pos.y - 20), "R?");
        
    } else if (component_type == "capacitor") {
        // Draw capacitor preview with rotation
        drawPreviewLine(ImVec2(pos.x - 20, pos.y), ImVec2(pos.x - 5, pos.y));
        drawPreviewLine(ImVec2(pos.x - 5, pos.y - 10), ImVec2(pos.x - 5, pos.y + 10));
        drawPreviewLine(ImVec2(pos.x + 5, pos.y - 10), ImVec2(pos.x + 5, pos.y + 10));
        drawPreviewLine(ImVec2(pos.x + 5, pos.y), ImVec2(pos.x + 20, pos.y));
        drawPreviewText(ImVec2(pos.x - 10, pos.y - 20), "C?");
        
    } else if (component_type == "inductor") {
        // Draw inductor preview with rotation
        for (int i = 0; i < 3; i++) {
            float center_x = pos.x - 15 + i * 10;
            ImVec2 circle_center = rotatePreviewPoint(ImVec2(center_x, pos.y));
            draw_list->AddCircle(circle_center, 5.0f, preview_color, 8, 1.0f);
        }
        drawPreviewLine(ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 20, pos.y));
        drawPreviewLine(ImVec2(pos.x + 20, pos.y), ImVec2(pos.x + 25, pos.y));
        drawPreviewText(ImVec2(pos.x - 10, pos.y - 20), "L?");
        
    } else if (component_type == "diode") {
        // Draw diode preview with rotation
        ImVec2 triangle_points[3] = {
            ImVec2(pos.x - 5, pos.y - 8),
            ImVec2(pos.x - 5, pos.y + 8),
            ImVec2(pos.x + 5, pos.y)
        };
        
        // Rotate triangle points
        for (int i = 0; i < 3; i++) {
            triangle_points[i] = rotatePreviewPoint(triangle_points[i]);
        }
        
        draw_list->AddPolyline(triangle_points, 3, preview_color, true, 2.0f);
        
        drawPreviewLine(ImVec2(pos.x + 5, pos.y - 8), ImVec2(pos.x + 5, pos.y + 8));
        drawPreviewLine(ImVec2(pos.x - 15, pos.y), ImVec2(pos.x - 5, pos.y));
        drawPreviewLine(ImVec2(pos.x + 5, pos.y), ImVec2(pos.x + 15, pos.y));
        drawPreviewText(ImVec2(pos.x - 10, pos.y - 25), "D?");
        
    } else if (component_type == "vsource") {
        // Draw voltage source preview with rotation (circle doesn't rotate, but connections do)
        draw_list->AddCircle(pos, 15.0f, preview_color, 16, 2.0f);
        drawPreviewLine(ImVec2(pos.x, pos.y - 20), ImVec2(pos.x, pos.y - 15));
        drawPreviewLine(ImVec2(pos.x, pos.y + 15), ImVec2(pos.x, pos.y + 20));
        
        // Draw + and - symbols (rotated)
        ImVec2 plus_pos = rotatePreviewPoint(ImVec2(pos.x - 4, pos.y - 13));
        ImVec2 minus_pos = rotatePreviewPoint(ImVec2(pos.x - 4, pos.y - 2));
        draw_list->AddText(plus_pos, preview_color, "+");
        draw_list->AddText(minus_pos, preview_color, "-");
        
        drawPreviewText(ImVec2(pos.x - 10, pos.y - 35), "V?");
        
    } else if (component_type == "ground") {
        // Draw ground preview with rotation
        drawPreviewLine(ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + 10));
        drawPreviewLine(ImVec2(pos.x - 8, pos.y + 10), ImVec2(pos.x + 8, pos.y + 10));
        drawPreviewLine(ImVec2(pos.x - 5, pos.y + 13), ImVec2(pos.x + 5, pos.y + 13));
        drawPreviewLine(ImVec2(pos.x - 2, pos.y + 16), ImVec2(pos.x + 2, pos.y + 16));
        drawPreviewText(ImVec2(pos.x - 15, pos.y - 20), "GND?");
        
    } else if (component_type == "nmosfet" || component_type == "pmosfet") {
        // Draw MOSFET preview with rotation
        // Gate line
        drawPreviewLine(ImVec2(pos.x - 20, pos.y - 10), ImVec2(pos.x - 20, pos.y + 10));
        drawPreviewLine(ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 20, pos.y));
        
        // Channel lines
        drawPreviewLine(ImVec2(pos.x - 15, pos.y - 8), ImVec2(pos.x - 15, pos.y - 3), 3.0f);
        drawPreviewLine(ImVec2(pos.x - 15, pos.y + 3), ImVec2(pos.x - 15, pos.y + 8), 3.0f);
        
        // Drain and source connections
        drawPreviewLine(ImVec2(pos.x - 15, pos.y - 5), ImVec2(pos.x, pos.y - 5));
        drawPreviewLine(ImVec2(pos.x, pos.y - 5), ImVec2(pos.x, pos.y - 15));
        drawPreviewLine(ImVec2(pos.x - 15, pos.y + 5), ImVec2(pos.x, pos.y + 5));
        drawPreviewLine(ImVec2(pos.x, pos.y + 5), ImVec2(pos.x, pos.y + 15));
        
        // Body connection
        drawPreviewLine(ImVec2(pos.x - 15, pos.y), ImVec2(pos.x + 15, pos.y), 1.0f);
        drawPreviewLine(ImVec2(pos.x + 15, pos.y), ImVec2(pos.x + 20, pos.y));
        
        // Arrow for NMOS vs PMOS (rotated)
        ImVec2 arrow[3];
        if (component_type == "nmosfet") {
            arrow[0] = ImVec2(pos.x - 5, pos.y - 2);
            arrow[1] = ImVec2(pos.x - 5, pos.y + 2);
            arrow[2] = ImVec2(pos.x - 10, pos.y);
        } else {
            arrow[0] = ImVec2(pos.x - 10, pos.y - 2);
            arrow[1] = ImVec2(pos.x - 10, pos.y + 2);
            arrow[2] = ImVec2(pos.x - 5, pos.y);
        }
        
        // Rotate arrow points
        for (int i = 0; i < 3; i++) {
            arrow[i] = rotatePreviewPoint(arrow[i]);
        }
        draw_list->AddPolyline(arrow, 3, preview_color, true, 1.0f);
        
        std::string label = (component_type == "nmosfet") ? "NMOS?" : "PMOS?";
        drawPreviewText(ImVec2(pos.x - 20, pos.y - 30), label.c_str());
    }
    
    // Draw preview pins (rotated)
    std::vector<ImVec2> pin_positions;
    
    if (component_type == "resistor" || component_type == "capacitor" || component_type == "inductor") {
        pin_positions.push_back(ImVec2(pos.x - 20, pos.y));
        pin_positions.push_back(ImVec2(pos.x + 20, pos.y));
    } else if (component_type == "diode") {
        pin_positions.push_back(ImVec2(pos.x - 10, pos.y));
        pin_positions.push_back(ImVec2(pos.x + 10, pos.y));
    } else if (component_type == "vsource") {
        pin_positions.push_back(ImVec2(pos.x, pos.y - 15));
        pin_positions.push_back(ImVec2(pos.x, pos.y + 15));
    } else if (component_type == "ground") {
        pin_positions.push_back(ImVec2(pos.x, pos.y));
    } else if (component_type == "nmosfet" || component_type == "pmosfet") {
        pin_positions.push_back(ImVec2(pos.x, pos.y - 15));    // drain
        pin_positions.push_back(ImVec2(pos.x - 25, pos.y));    // gate
        pin_positions.push_back(ImVec2(pos.x, pos.y + 15));    // source
        pin_positions.push_back(ImVec2(pos.x + 20, pos.y));    // bulk
    }
    
    // Draw preview pins (rotated)
    for (const auto& pin_pos : pin_positions) {
        ImVec2 rotated_pin = rotatePreviewPoint(pin_pos);
        draw_list->AddCircleFilled(rotated_pin, 4.0f, IM_COL32(255, 0, 0, 100));
        draw_list->AddCircle(rotated_pin, 4.0f, IM_COL32(255, 255, 255, 120), 8, 1.0f);
    }
    
    // Draw rotation indicator
    if (rotation != 0) {
        ImVec2 indicator_pos = ImVec2(pos.x + 30, pos.y - 30);
        draw_list->AddText(indicator_pos, IM_COL32(0, 255, 255, 150), (std::to_string(rotation) + "°").c_str());
    }
}

void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component, CircuitElement* selected_component) {
    ImVec2 pos(canvas_offset.x + component->x, canvas_offset.y + component->y);
    int rotation = component->getRotation();

    // Draw selection highlight if component is selected
    if (component == selected_component) {
        draw_list->AddCircle(pos, 35.0f, IM_COL32(255, 255, 0, 150), 16, 3.0f);
    }
    
    if (component->getType() == "resistor") {
        // Draw resistor symbol with rotation
        drawRotatedLine(draw_list, ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 15, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedRect(draw_list, ImVec2(pos.x - 15, pos.y - 5), ImVec2(pos.x + 15, pos.y + 5), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 15, pos.y), ImVec2(pos.x + 25, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Draw labels (offset based on rotation)
        ImVec2 name_offset, value_offset;
        switch (rotation) {
            case 90:
                name_offset = ImVec2(pos.x + 20, pos.y - 10);
                value_offset = ImVec2(pos.x - 20, pos.y - 10);
                break;
            case 180:
                name_offset = ImVec2(pos.x - 10, pos.y + 20);
                value_offset = ImVec2(pos.x - 10, pos.y - 10);
                break;
            case 270:
                name_offset = ImVec2(pos.x - 20, pos.y - 10);
                value_offset = ImVec2(pos.x + 20, pos.y - 10);
                break;
            default: // 0 degrees
                name_offset = ImVec2(pos.x - 10, pos.y - 20);
                value_offset = ImVec2(pos.x - 10, pos.y + 10);
                break;
        }
        
        draw_list->AddText(name_offset, IM_COL32(255, 255, 0, 255), component->name.c_str());
        draw_list->AddText(value_offset, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "capacitor") {
        // Draw capacitor symbol with rotation
        drawRotatedLine(draw_list, ImVec2(pos.x - 20, pos.y), ImVec2(pos.x - 5, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 5, pos.y - 10), ImVec2(pos.x - 5, pos.y + 10), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 5, pos.y - 10), ImVec2(pos.x + 5, pos.y + 10), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 5, pos.y), ImVec2(pos.x + 20, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y - 20), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y + 10), pos, rotation, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "vsource") {
        // Draw voltage source symbol (circle doesn't need rotation, but lines do)
        draw_list->AddCircle(pos, 15.0f, IM_COL32(255, 255, 255, 255), 16, 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x, pos.y - 20), ImVec2(pos.x, pos.y - 15), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x, pos.y + 15), ImVec2(pos.x, pos.y + 20), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Draw + and - symbols (rotated)
        drawRotatedText(draw_list, ImVec2(pos.x - 4, pos.y - 13), pos, rotation, IM_COL32(255, 255, 255, 255), "+");
        drawRotatedText(draw_list, ImVec2(pos.x - 4, pos.y - 2), pos, rotation, IM_COL32(255, 255, 255, 255), "-");
        
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y - 35), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y + 25), pos, rotation, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "ground") {
        // Draw ground symbol with rotation
        drawRotatedLine(draw_list, ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + 10), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 8, pos.y + 10), ImVec2(pos.x + 8, pos.y + 10), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 5, pos.y + 13), ImVec2(pos.x + 5, pos.y + 13), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 2, pos.y + 16), ImVec2(pos.x + 2, pos.y + 16), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y - 20), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "inductor") {
        // Draw inductor symbol with rotation
        for (int i = 0; i < 3; i++) {
            float center_x = pos.x - 15 + i * 10;
            ImVec2 circle_center = rotatePoint(ImVec2(center_x, pos.y), pos, rotation);
            draw_list->AddCircle(circle_center, 5.0f, IM_COL32(255, 255, 255, 255), 8, 1.0f);
        }
        drawRotatedLine(draw_list, ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 20, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 20, pos.y), ImVec2(pos.x + 25, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y - 20), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y + 10), pos, rotation, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "diode") {
        // Draw diode symbol with rotation
        ImVec2 triangle_points[3] = {
            ImVec2(pos.x - 5, pos.y - 8),
            ImVec2(pos.x - 5, pos.y + 8),
            ImVec2(pos.x + 5, pos.y)
        };
        
        // Rotate triangle points
        for (int i = 0; i < 3; i++) {
            triangle_points[i] = rotatePoint(triangle_points[i], pos, rotation);
        }
        
        draw_list->AddPolyline(triangle_points, 3, IM_COL32(255, 255, 255, 255), true, 2.0f);
        
        drawRotatedLine(draw_list, ImVec2(pos.x + 5, pos.y - 8), ImVec2(pos.x + 5, pos.y + 8), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y), ImVec2(pos.x - 5, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 5, pos.y), ImVec2(pos.x + 15, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        drawRotatedText(draw_list, ImVec2(pos.x - 10, pos.y - 25), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawRotatedText(draw_list, ImVec2(pos.x - 15, pos.y + 15), pos, rotation, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
        
    } else if (component->getType() == "nmosfet" || component->getType() == "pmosfet") {
        // Draw MOSFET symbol with rotation
        // Gate line
        drawRotatedLine(draw_list, ImVec2(pos.x - 20, pos.y - 10), ImVec2(pos.x - 20, pos.y + 10), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 25, pos.y), ImVec2(pos.x - 20, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Channel lines
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y - 8), ImVec2(pos.x - 15, pos.y - 3), pos, rotation, IM_COL32(255, 255, 255, 255), 3.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y + 3), ImVec2(pos.x - 15, pos.y + 8), pos, rotation, IM_COL32(255, 255, 255, 255), 3.0f);
        
        // Drain and source connections
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y - 5), ImVec2(pos.x, pos.y - 5), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x, pos.y - 5), ImVec2(pos.x, pos.y - 15), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y + 5), ImVec2(pos.x, pos.y + 5), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x, pos.y + 5), ImVec2(pos.x, pos.y + 15), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Body connection
        drawRotatedLine(draw_list, ImVec2(pos.x - 15, pos.y), ImVec2(pos.x + 15, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 1.0f);
        drawRotatedLine(draw_list, ImVec2(pos.x + 15, pos.y), ImVec2(pos.x + 20, pos.y), pos, rotation, IM_COL32(255, 255, 255, 255), 2.0f);
        
        // Arrow for NMOS vs PMOS (rotated)
        ImVec2 arrow[3];
        if (component->getType() == "nmosfet") {
            // Arrow pointing in (substrate to channel)
            arrow[0] = ImVec2(pos.x - 5, pos.y - 2);
            arrow[1] = ImVec2(pos.x - 5, pos.y + 2);
            arrow[2] = ImVec2(pos.x - 10, pos.y);
        } else {
            // Arrow pointing out (channel to substrate)
            arrow[0] = ImVec2(pos.x - 10, pos.y - 2);
            arrow[1] = ImVec2(pos.x - 10, pos.y + 2);
            arrow[2] = ImVec2(pos.x - 5, pos.y);
        }
        
        // Rotate arrow points
        for (int i = 0; i < 3; i++) {
            arrow[i] = rotatePoint(arrow[i], pos, rotation);
        }
        draw_list->AddPolyline(arrow, 3, IM_COL32(255, 255, 255, 255), true, 1.0f);
        
        drawRotatedText(draw_list, ImVec2(pos.x - 15, pos.y - 30), pos, rotation, IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawRotatedText(draw_list, ImVec2(pos.x - 20, pos.y + 20), pos, rotation, IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
        drawPins(component, canvas_offset, draw_list);
    }
}
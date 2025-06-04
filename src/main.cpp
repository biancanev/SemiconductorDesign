/** 
 * SPICE interface. Generates a netlist based on the user defined circuit.
 * 
 * Author: Ryan Kwong (ryan.kwong.04@berkeley.edu)
 * 
 * TODO:
 *  - Improve wire routing for 90° wire bends
 *  - Implement delete component function
 *  - I kind of just placed functions wherever they worked. Eventually, I need to refactor this code so that it is more organized :(
 *  **/

#include <iostream>
#include <iomanip>
#include <GLFW/glfw3.h>
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>

// ImGui includes
#define IMGUI_DEFINE_MATH_OPERATORS 
#include "../external/imgui/imgui.h"
#include "../external/imgui/imgui_internal.h" 
#include "../external/imgui/backends/imgui_impl_glfw.h"
#include "../external/imgui/backends/imgui_impl_opengl3.h"

// Force ImGui version consistency
#if IMGUI_VERSION_NUM < 18700
#error "This code requires ImGui 1.87 or higher"
#endif

// Ensure we're using the new key system
#ifndef IMGUI_DISABLE_OBSOLETE_KEYIO
#define IMGUI_DISABLE_OBSOLETE_KEYIO
#endif

#include "parser/spice_parser.h"
#include "circuit_manager.h"

// Forward declaration
void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component, CircuitElement* selected_component, float zoom, ImVec2 pan);
void drawComponentPreview(ImDrawList* draw_list, ImVec2 screen_pos, const std::string& component_type, int rotation, ImVec2 canvas_offset, float zoom, ImVec2 pan);

ImVec2 snapToGrid(ImVec2 pos, float grid_step){
    return ImVec2(std::round(pos.x / grid_step) * grid_step, std::round(pos.y / grid_step) * grid_step);
}

struct WireSegment {
    ImVec2 start;
    ImVec2 end;
};

// Simulation settings structures
struct DCSimSettings {
    bool enabled = true;
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
bool show_dc_results_dialog = false;
DCAnalysis* current_dc_analysis = nullptr;
std::vector<std::string> dc_error_messages;

// Zoom and pan state
float zoom_level = 1.0f;
ImVec2 pan_offset = ImVec2(0, 0);
bool panning = false;
ImVec2 pan_start_mouse_pos;
ImVec2 pan_start_offset;

// Zoom constraints
const float MIN_ZOOM = 0.1f;
const float MAX_ZOOM = 5.0f;

ImVec2 worldToScreen(ImVec2 world_pos, ImVec2 canvas_offset, float zoom, ImVec2 pan) {
    return ImVec2(
        canvas_offset.x + (world_pos.x + pan.x) * zoom,
        canvas_offset.y + (world_pos.y + pan.y) * zoom
    );
}

// Transform screen coordinates to world coordinates
ImVec2 screenToWorld(ImVec2 screen_pos, ImVec2 canvas_offset, float zoom, ImVec2 pan) {
    return ImVec2(
        (screen_pos.x - canvas_offset.x) / zoom - pan.x,
        (screen_pos.y - canvas_offset.y) / zoom - pan.y
    );
}

// Apply zoom to a distance/size value
float applyZoom(float value, float zoom) {
    return value * zoom;
}

// Get mouse position in world coordinates
ImVec2 getMouseWorldPos(ImVec2 canvas_offset, float zoom, ImVec2 pan) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    return screenToWorld(mouse_pos, canvas_offset, zoom, pan);
}

void ShowDCResultsDialog(bool* show_dialog, DCAnalysis* dc_analysis, const CircuitManager& circuit, const std::vector<std::string>& error_messages) {
    if (!*show_dialog) return;
    
    ImGui::SetNextWindowSize(ImVec2(600, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("DC Analysis Results", show_dialog)) {
        
        if (!error_messages.empty()) {
            // Show errors if any
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Analysis Errors:");
            ImGui::Separator();
            for (const auto& error : error_messages) {
                ImGui::TextWrapped("• %s", error.c_str());
            }
            
            ImGui::Separator();
            if (ImGui::Button("Close")) {
                *show_dialog = false;
            }
        } else if (dc_analysis) {
            // Show results in tabs
            if (ImGui::BeginTabBar("DCResultsTabs")) {
                
                // Node Voltages Tab
                if (ImGui::BeginTabItem("Node Voltages")) {
                    ImGui::Text("DC Node Voltages");
                    ImGui::Separator();
                    
                    // Create a table for better formatting
                    if (ImGui::BeginTable("NodeVoltagesTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("Node", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Voltage (V)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();
                        
                        // Ground node
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("0 (Ground)");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("0.000");
                        
                        // Other nodes
                        const auto& used_nodes = circuit.getUsedNodes();
                        for (int node : used_nodes) {
                            if (node != 0) { // Skip ground, already shown
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::Text("%d", node);
                                ImGui::TableSetColumnIndex(1);
                                double voltage = dc_analysis->getNodeVoltage(node);
                                ImGui::Text("%.6f", voltage);
                            }
                        }
                        
                        ImGui::EndTable();
                    }
                    
                    ImGui::EndTabItem();
                }
                
                // Component Voltages Tab
                if (ImGui::BeginTabItem("Component Voltages")) {
                    ImGui::Text("Voltage Across Components");
                    ImGui::Separator();
                    
                    if (ImGui::BeginTable("ComponentVoltagesTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("Component", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                        ImGui::TableSetupColumn("Nodes", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                        ImGui::TableSetupColumn("Voltage (V)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();
                        
                        for (const auto& component : circuit.getComponents()) {
                            if (component->getType() == "ground") continue; // Skip ground symbols
                            if (!component->isFullyConnected()) continue; // Skip unconnected components
                            
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::Text("%s", component->name.c_str());
                            
                            ImGui::TableSetColumnIndex(1);
                            ImGui::Text("%s", component->getType().c_str());
                            
                            // Calculate voltage across component
                            if (component->getPinCount() >= 2) {
                                int node1 = component->getNodeForPin(0);
                                int node2 = component->getPinCount() > 1 ? component->getNodeForPin(1) : 0;
                                
                                ImGui::TableSetColumnIndex(2);
                                ImGui::Text("%d-%d", node1, node2);
                                
                                ImGui::TableSetColumnIndex(3);
                                double v1 = dc_analysis->getNodeVoltage(node1);
                                double v2 = dc_analysis->getNodeVoltage(node2);
                                double voltage_across = v1 - v2;
                                ImGui::Text("%.6f", voltage_across);
                            } else {
                                ImGui::TableSetColumnIndex(2);
                                ImGui::Text("N/A");
                                ImGui::TableSetColumnIndex(3);
                                ImGui::Text("N/A");
                            }
                        }
                        
                        ImGui::EndTable();
                    }
                    
                    ImGui::EndTabItem();
                }
                
                // Component Currents Tab
                if (ImGui::BeginTabItem("Component Currents")) {
                    ImGui::Text("Current Through Components");
                    ImGui::Separator();
                    
                    if (ImGui::BeginTable("ComponentCurrentsTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("Component", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Current (A)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();
                        
                        for (const auto& component : circuit.getComponents()) {
                            if (component->getType() == "ground") continue;
                            if (!component->isFullyConnected()) continue;
                            
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::Text("%s", component->name.c_str());
                            
                            ImGui::TableSetColumnIndex(1);
                            ImGui::Text("%s", component->getType().c_str());
                            
                            ImGui::TableSetColumnIndex(2);
                            ImGui::Text("%s", component->getValue().c_str());
                            
                            ImGui::TableSetColumnIndex(3);
                            
                            // Calculate current based on component type
                            if (component->getType() == "vsource") {
                                // For voltage sources, get the current directly from analysis
                                double current = dc_analysis->getVoltagSourceCurrent(component->name);
                                ImGui::Text("%.6f", current);
                            } else if (component->getType() == "resistor" && component->getPinCount() >= 2) {
                                // For resistors, use Ohm's law: I = V/R
                                int node1 = component->getNodeForPin(0);
                                int node2 = component->getNodeForPin(1);
                                double v1 = dc_analysis->getNodeVoltage(node1);
                                double v2 = dc_analysis->getNodeVoltage(node2);
                                double voltage_across = v1 - v2;
                                
                                // Get resistance value
                                if (const Resistor* resistor = dynamic_cast<const Resistor*>(component.get())) {
                                    double current = voltage_across / resistor->r;
                                    ImGui::Text("%.6f", current);
                                } else {
                                    ImGui::Text("N/A");
                                }
                            } else if (component->getType() == "capacitor") {
                                // For DC analysis, capacitor current is 0 (open circuit)
                                ImGui::Text("0.000000");
                            } else if (component->getType() == "inductor") {
                                // For DC analysis, calculate current through inductor (short circuit)
                                if (component->getPinCount() >= 2) {
                                    int node1 = component->getNodeForPin(0);
                                    int node2 = component->getNodeForPin(1);
                                    double v1 = dc_analysis->getNodeVoltage(node1);
                                    double v2 = dc_analysis->getNodeVoltage(node2);
                                    
                                    // For ideal inductor in DC, V1 should equal V2 (short circuit)
                                    // Current would need to be calculated from surrounding circuit
                                    if (std::abs(v1 - v2) < 1e-6) {
                                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Calculated*");
                                    } else {
                                        ImGui::Text("%.6f", (v1 - v2) / 1e-6); // Using very small resistance
                                    }
                                } else {
                                    ImGui::Text("N/A");
                                }
                            } else {
                                // For other components (diodes, transistors, etc.)
                                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Not calculated");
                            }
                        }
                        
                        ImGui::EndTable();
                    }
                    
                    ImGui::Text("* Inductor current requires circuit analysis");
                    
                    ImGui::EndTabItem();
                }
                
                // Power Dissipation Tab
                if (ImGui::BeginTabItem("Power")) {
                    ImGui::Text("Power Dissipation");
                    ImGui::Separator();
                    
                    if (ImGui::BeginTable("PowerTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("Component", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Voltage (V)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Current (A)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Power (W)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();
                        
                        double total_power = 0.0;
                        
                        for (const auto& component : circuit.getComponents()) {
                            if (component->getType() == "ground") continue;
                            if (!component->isFullyConnected()) continue;
                            
                            if (component->getType() == "resistor" && component->getPinCount() >= 2) {
                                int node1 = component->getNodeForPin(0);
                                int node2 = component->getNodeForPin(1);
                                double v1 = dc_analysis->getNodeVoltage(node1);
                                double v2 = dc_analysis->getNodeVoltage(node2);
                                double voltage_across = v1 - v2;
                                
                                if (const Resistor* resistor = dynamic_cast<const Resistor*>(component.get())) {
                                    double current = voltage_across / resistor->r;
                                    double power = voltage_across * current; // P = VI
                                    total_power += std::abs(power);
                                    
                                    ImGui::TableNextRow();
                                    ImGui::TableSetColumnIndex(0);
                                    ImGui::Text("%s", component->name.c_str());
                                    ImGui::TableSetColumnIndex(1);
                                    ImGui::Text("%.6f", voltage_across);
                                    ImGui::TableSetColumnIndex(2);
                                    ImGui::Text("%.6f", current);
                                    ImGui::TableSetColumnIndex(3);
                                    ImGui::Text("%.6f", power);
                                    ImGui::TableSetColumnIndex(4);
                                    ImGui::Text("Dissipated");
                                }
                            } else if (component->getType() == "vsource") {
                                double current = dc_analysis->getVoltagSourceCurrent(component->name);
                                if (const VoltageSource* vsrc = dynamic_cast<const VoltageSource*>(component.get())) {
                                    double power = vsrc->v * current;
                                    
                                    ImGui::TableNextRow();
                                    ImGui::TableSetColumnIndex(0);
                                    ImGui::Text("%s", component->name.c_str());
                                    ImGui::TableSetColumnIndex(1);
                                    ImGui::Text("%.6f", vsrc->v);
                                    ImGui::TableSetColumnIndex(2);
                                    ImGui::Text("%.6f", current);
                                    ImGui::TableSetColumnIndex(3);
                                    ImGui::Text("%.6f", power);
                                    ImGui::TableSetColumnIndex(4);
                                    ImGui::Text(power > 0 ? "Supplied" : "Absorbed");
                                }
                            }
                        }
                        
                        // Add total row
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "TOTAL");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("--");
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("--");
                        ImGui::TableSetColumnIndex(3);
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%.6f", total_power);
                        ImGui::TableSetColumnIndex(4);
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Dissipated");
                        
                        ImGui::EndTable();
                    }
                    
                    ImGui::EndTabItem();
                }
                
                // Circuit Summary Tab
                if (ImGui::BeginTabItem("Summary")) {
                    ImGui::Text("Circuit Analysis Summary");
                    ImGui::Separator();
                    
                    // Count components
                    int resistor_count = 0, capacitor_count = 0, inductor_count = 0;
                    int vsource_count = 0, other_count = 0;
                    
                    for (const auto& component : circuit.getComponents()) {
                        if (component->getType() == "resistor") resistor_count++;
                        else if (component->getType() == "capacitor") capacitor_count++;
                        else if (component->getType() == "inductor") inductor_count++;
                        else if (component->getType() == "vsource") vsource_count++;
                        else if (component->getType() != "ground") other_count++;
                    }
                    
                    ImGui::Text("Circuit Composition:");
                    ImGui::BulletText("Resistors: %d", resistor_count);
                    ImGui::BulletText("Capacitors: %d", capacitor_count);
                    ImGui::BulletText("Inductors: %d", inductor_count);
                    ImGui::BulletText("Voltage Sources: %d", vsource_count);
                    ImGui::BulletText("Other Components: %d", other_count);
                    ImGui::BulletText("Total Nodes: %d", circuit.getNodeCount());
                    ImGui::BulletText("Total Wires: %zu", circuit.getWires().size());
                    
                    ImGui::Separator();
                    
                    // Analysis information
                    ImGui::Text("Analysis Information:");
                    ImGui::BulletText("Analysis Type: DC Operating Point");
                    ImGui::BulletText("Solution Method: Modified Nodal Analysis");
                    ImGui::BulletText("Matrix Size: %dx%d", circuit.getNodeCount()-1 + vsource_count, circuit.getNodeCount()-1 + vsource_count);
                    
                    ImGui::Separator();
                    
                    // Validation results
                    auto validation_errors = circuit.validateCircuit();
                    if (validation_errors.empty()) {
                        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Circuit validation passed");
                    } else {
                        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "⚠ Circuit validation warnings:");
                        for (const auto& error : validation_errors) {
                            ImGui::TextWrapped("  • %s", error.c_str());
                        }
                    }
                    
                    ImGui::EndTabItem();
                }
                
                ImGui::EndTabBar();
            }
            
            ImGui::Separator();
            
            // Export and close buttons
            if (ImGui::Button("Export Results")) {
                // TODO: Implement results export
                std::cout << "Export results functionality not implemented yet" << std::endl;
            }
            ImGui::SameLine();
            if (ImGui::Button("Close")) {
                *show_dialog = false;
            }
        } else {
            ImGui::Text("No analysis results available.");
            ImGui::Separator();
            if (ImGui::Button("Close")) {
                *show_dialog = false;
            }
        }
    }
    ImGui::End();
}


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
    ImVec2 last_wire_point;

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
            
            if (ImGui::Button("Voltage Source", ImVec2(-1, 30))) {
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
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Click pin or wire");
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
            ImGui::Text("Junctions: %zu", circuit.getJunctions().size());
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

            // Draw grid with zoom and pan
            float base_grid_step = 20.0f;
            float grid_step = applyZoom(base_grid_step, zoom_level);
            
            // Only draw grid if it's not too dense or too sparse
            if (grid_step > 2.0f && grid_step < 200.0f) {
                float grid_offset_x = std::fmod(pan_offset.x * zoom_level, grid_step);
                float grid_offset_y = std::fmod(pan_offset.y * zoom_level, grid_step);
                
                for (float x = grid_offset_x; x < canvas_sz.x; x += grid_step) {
                    draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), 
                                    ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(50, 50, 50, 255));
                }
                for (float y = grid_offset_y; y < canvas_sz.y; y += grid_step) {
                    draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), 
                                    ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(50, 50, 50, 255));
                }
            }

            // Handle mouse interactions
            ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
            current_mouse_pos = ImGui::GetMousePos();

            // Handle zooming
            if (ImGui::IsItemHovered()) {
                float wheel = ImGui::GetIO().MouseWheel;
                if (wheel != 0.0f) {
                    // Get mouse position in world coordinates before zoom
                    ImVec2 mouse_world_pos_before = screenToWorld(current_mouse_pos, canvas_p0, zoom_level, pan_offset);
                    
                    // Apply zoom
                    float zoom_factor = 1.0f + wheel * 0.1f;
                    float new_zoom = zoom_level * zoom_factor;
                    new_zoom = std::max(MIN_ZOOM, std::min(MAX_ZOOM, new_zoom));
                    
                    if (new_zoom != zoom_level) {
                        // Get mouse position in world coordinates after zoom
                        ImVec2 mouse_world_pos_after = screenToWorld(current_mouse_pos, canvas_p0, new_zoom, pan_offset);
                        
                        // Adjust pan to keep mouse position stable
                        pan_offset.x += mouse_world_pos_before.x - mouse_world_pos_after.x;
                        pan_offset.y += mouse_world_pos_before.y - mouse_world_pos_after.y;
                        
                        zoom_level = new_zoom;
                        std::cout << "Zoom: " << std::fixed << std::setprecision(2) << zoom_level << "x" << std::endl;
                    }
                }
            }

            // Handle panning
            bool start_panning = (ImGui::IsItemClicked(ImGuiMouseButton_Middle)) || 
                                (ImGui::IsItemClicked(ImGuiMouseButton_Left) && ImGui::IsKeyDown(static_cast<ImGuiKey>(ImGuiKey_LeftCtrl)));
            
            if (start_panning && !panning) {
                panning = true;
                pan_start_mouse_pos = current_mouse_pos;
                pan_start_offset = pan_offset;
                std::cout << "Started panning" << std::endl;
            }
            
            if (panning) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || 
                (ImGui::IsMouseDown(static_cast<ImGuiMouseButton>(ImGuiMouseButton_Left)) && ImGui::IsKeyDown(static_cast<ImGuiKey>(ImGuiKey_LeftCtrl)))) {
                    ImVec2 mouse_delta = ImVec2(current_mouse_pos.x - pan_start_mouse_pos.x, 
                                            current_mouse_pos.y - pan_start_mouse_pos.y);
                    pan_offset.x = pan_start_offset.x + mouse_delta.x / zoom_level;
                    pan_offset.y = pan_start_offset.y + mouse_delta.y / zoom_level;
                } else {
                    panning = false;
                    std::cout << "Stopped panning" << std::endl;
                }
            }

            // Update preview position when placing components (with zoom/pan)
            if (placing_component && !panning) {
                ImVec2 mouse_world_pos = getMouseWorldPos(canvas_p0, zoom_level, pan_offset);
                ImVec2 snapped_world_pos = snapToGrid(mouse_world_pos, base_grid_step);
                preview_position = worldToScreen(snapped_world_pos, canvas_p0, zoom_level, pan_offset);
                show_component_preview = true;
            } else if (!placing_component) {
                show_component_preview = false;
            }

            // Check for keyboard shortcuts
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_Escape))) {
                if (placing_component) {
                    // Cancel component placement
                    placing_component = false;
                    selected_component_type = "";
                    show_component_preview = false;
                    preview_rotation = 0;  // Reset preview rotation
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

            // Handle rotation
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_R))) {
                if (ImGui::IsKeyDown(static_cast<ImGuiKey>(ImGuiKey_LeftCtrl))){
                    if (selected_component) {
                        // Rotate selected component
                        selected_component->rotate90();
                        std::cout << "Rotated " << selected_component->name << " to " << selected_component->getRotation() << "°" << std::endl;
                    } else if (placing_component) {
                        // Rotate preview component
                        preview_rotation = (preview_rotation + 90) % 360;
                        std::cout << "Preview rotation: " << preview_rotation << "°" << std::endl;
                    }
                } else {
                    placing_component = true;
                    selected_component_type = "resistor";
                }
            }
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_C))) {
                placing_component = true;
                selected_component_type = "capacitor";
            }
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_D))) {
                placing_component = true;
                selected_component_type = "diode";
            }
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_L))) {
                placing_component = true;
                selected_component_type = "inductor";
            }
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_G))) {
                placing_component = true;
                selected_component_type = "ground";
            }
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_W))) {
                placing_component = false;
                wire_mode = true;
            }



            // Handle right-click for component editing
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                ImVec2 mouse_world_pos = getMouseWorldPos(canvas_p0, zoom_level, pan_offset);
                
                // Find component at mouse position
                CircuitElement* clicked_component = nullptr;
                for (const auto& component : circuit.getComponents()) {
                    if (component->isPointInside(mouse_world_pos.x, mouse_world_pos.y)) {
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
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && !panning && !ImGui::IsKeyDown(static_cast<ImGuiKey>(ImGuiKey_LeftCtrl))) {
                ImVec2 mouse_world_pos = getMouseWorldPos(canvas_p0, zoom_level, pan_offset);
                
                if (placing_component) {
                    // Component placement mode with grid snapping
                    ImVec2 snapped_pos = snapToGrid(mouse_world_pos, base_grid_step);
                    
                    std::cout << "Attempting to place component: " << selected_component_type << std::endl;
                    
                    CircuitElement* new_component = circuit.addComponent(selected_component_type, snapped_pos.x, snapped_pos.y);
                    if (new_component) {
                        // Apply the preview rotation to the newly placed component
                        new_component->setRotation(preview_rotation);
                        
                        current_netlist = circuit.generateNetlist();
                        std::cout << "Added " << new_component->name << " at (" << snapped_pos.x << ", " << snapped_pos.y 
                                << ") with rotation " << preview_rotation << "°" << std::endl;
                    } else {
                        std::cout << "Failed to add component: " << selected_component_type << std::endl;
                    }
                    
                    // Keep placement mode active but don't reset rotation
                    show_component_preview = true;
                    
                } else if (wire_mode) {
                    // Wire mode - enhanced routing with multi-segment support and wire-to-wire connections
                    // NOTE: I cheated and used Claude for wire management in the GUI because my code kept segfaulting.
                    // I need to figure out how this code actually works.
                    auto [component, pin] = circuit.findPinAt(mouse_world_pos.x, mouse_world_pos.y);
                    auto [existing_wire, intersection] = circuit.findWireAt(mouse_world_pos.x, mouse_world_pos.y);
                    
                    if (component && pin != -1) {
                        // Clicked on a component pin
                        if (!wire_start_component) {
                            // Start a new wire
                            wire_start_component = component;
                            wire_start_pin = pin;
                            auto pins = component->getAbsolutePinPositions();
                            
                            // Store start position in world coordinates
                            last_wire_point = ImVec2(pins[pin].first, pins[pin].second);
                            wire_start_pos = worldToScreen(last_wire_point, canvas_p0, zoom_level, pan_offset);
                            
                            current_wire_segments.clear();
                            wire_horizontal_first = true;
                            std::cout << "Starting wire from " << component->name << " pin " << pin << std::endl;
                        } else {
                            // Complete the wire - connect to end pin with the current path
                            std::vector<ImVec2> wire_path;
                            
                            // Convert current segments to world coordinates for storage
                            for (const auto& segment : current_wire_segments) {
                                ImVec2 world_start = screenToWorld(segment.start, canvas_p0, zoom_level, pan_offset);
                                ImVec2 world_end = screenToWorld(segment.end, canvas_p0, zoom_level, pan_offset);
                                
                                // Add the end point of each segment (start of first segment is the pin position)
                                if (wire_path.empty() || (world_end.x != wire_path.back().x || world_end.y != wire_path.back().y)) {
                                    wire_path.push_back(world_end);
                                }
                            }
                            
                            // Connect with the path
                            if (circuit.connectPins(wire_start_component, wire_start_pin, component, pin, wire_path)) {
                                current_netlist = circuit.generateNetlist();
                                std::cout << "Wire completed with " << wire_path.size() << " waypoints!" << std::endl;
                            } else {
                                std::cout << "Cannot connect these pins" << std::endl;
                            }
                            
                            // Reset wire routing state
                            wire_start_component = nullptr;
                            wire_start_pin = -1;
                            current_wire_segments.clear();
                            last_wire_point = ImVec2(0, 0);
                        }
                    } else if (existing_wire && wire_start_component) {
                        // Connect current wire to an existing wire (create junction)
                        ImVec2 snapped_intersection = snapToGrid(ImVec2(intersection.x, intersection.y), base_grid_step);
                        
                        if (circuit.connectToWire(wire_start_component, wire_start_pin, 
                                                 existing_wire, snapped_intersection.x, snapped_intersection.y)) {
                            current_netlist = circuit.generateNetlist();
                            std::cout << "Connected to existing wire at junction!" << std::endl;
                        }
                        
                        wire_start_component = nullptr;
                        wire_start_pin = -1;
                        current_wire_segments.clear();
                        last_wire_point = ImVec2(0, 0);
                    } else if (wire_start_component) {
                        // Clicked on empty space while routing - add a waypoint
                        ImVec2 snapped_world_pos = snapToGrid(mouse_world_pos, base_grid_step);
                        ImVec2 snapped_screen_pos = worldToScreen(snapped_world_pos, canvas_p0, zoom_level, pan_offset);
                        
                        // Get the current start point for this segment
                        ImVec2 current_start_screen = current_wire_segments.empty() ? 
                            wire_start_pos : current_wire_segments.back().end;
                        
                        // Calculate the corner point based on current direction preference
                        ImVec2 corner_point;
                        if (wire_horizontal_first) {
                            // Go horizontal first, then vertical
                            corner_point = ImVec2(snapped_screen_pos.x, current_start_screen.y);
                        } else {
                            // Go vertical first, then horizontal
                            corner_point = ImVec2(current_start_screen.x, snapped_screen_pos.y);
                        }
                        
                        // Add segments to reach the clicked point
                        if (std::abs(corner_point.x - current_start_screen.x) > 1.0f || 
                            std::abs(corner_point.y - current_start_screen.y) > 1.0f) {
                            current_wire_segments.push_back({current_start_screen, corner_point});
                        }
                        
                        if (std::abs(corner_point.x - snapped_screen_pos.x) > 1.0f || 
                            std::abs(corner_point.y - snapped_screen_pos.y) > 1.0f) {
                            current_wire_segments.push_back({corner_point, snapped_screen_pos});
                        }
                        
                        // Update last wire point for next preview
                        last_wire_point = snapped_world_pos;
                        
                        // Switch direction preference for next segment
                        wire_horizontal_first = !wire_horizontal_first;
                        
                        std::cout << "Added wire waypoint at (" << snapped_world_pos.x << ", " << snapped_world_pos.y << ")" << std::endl;
                    } else {
                        // Clicked on empty space with no active wire - cancel any active wire
                        wire_start_component = nullptr;
                        wire_start_pin = -1;
                        current_wire_segments.clear();
                        last_wire_point = ImVec2(0, 0);
                    }
                } else {
                    // Normal mode - component selection and movement
                    CircuitElement* clicked_component = nullptr;
                    
                    // Check if we clicked on a component
                    for (const auto& component : circuit.getComponents()) {
                        if (component->isPointInside(mouse_world_pos.x, mouse_world_pos.y)) {
                            clicked_component = component.get();
                            break;
                        }
                    }
                    
                    if (clicked_component) {
                        if (selected_component == clicked_component) {
                            // Already selected - start dragging
                            dragging_component = true;
                            drag_start_pos = mouse_world_pos;
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

            // Handle dragging in world coordinates
            if (dragging_component && selected_component && ImGui::IsMouseDown(ImGuiMouseButton_Left) && !panning) {
                ImVec2 mouse_world_pos = getMouseWorldPos(canvas_p0, zoom_level, pan_offset);
                
                // Calculate new position with grid snapping
                ImVec2 new_pos = ImVec2(
                    component_start_pos.x + (mouse_world_pos.x - drag_start_pos.x),
                    component_start_pos.y + (mouse_world_pos.y - drag_start_pos.y)
                );
                ImVec2 snapped_pos = snapToGrid(new_pos, base_grid_step);
                
                selected_component->setPosition(snapped_pos.x, snapped_pos.y);
            } else if (dragging_component) {
                // Finished dragging
                dragging_component = false;
                std::cout << "Finished dragging " << selected_component->name << 
                            " to (" << selected_component->x << ", " << selected_component->y << ")" << std::endl;
            }

            // Draw components with zoom and pan
            for (const auto& component : circuit.getComponents()) {
                drawComponent(draw_list, canvas_p0, component.get(), selected_component, zoom_level, pan_offset);
            }

            // Draw component preview with zoom and pan
            if (show_component_preview && placing_component && !selected_component_type.empty()) {
                drawComponentPreview(draw_list, preview_position, selected_component_type, preview_rotation, canvas_p0, zoom_level, pan_offset);
            }

            // Draw wires with enhanced junction support
            for (const auto& wire : circuit.getWires()) {
                try {
                    auto complete_path = wire->getCompletePath();
                    
                    if (complete_path.size() >= 2) {
                        // Draw each segment of the wire path
                        for (size_t i = 0; i < complete_path.size() - 1; ++i) {
                            ImVec2 start_screen = worldToScreen(complete_path[i], canvas_p0, zoom_level, pan_offset);
                            ImVec2 end_screen = worldToScreen(complete_path[i + 1], canvas_p0, zoom_level, pan_offset);
                            
                            // Validate coordinates before drawing
                            if (std::isfinite(start_screen.x) && std::isfinite(start_screen.y) &&
                                std::isfinite(end_screen.x) && std::isfinite(end_screen.y) &&
                                std::abs(start_screen.x) < 10000 && std::abs(start_screen.y) < 10000 &&
                                std::abs(end_screen.x) < 10000 && std::abs(end_screen.y) < 10000) {
                                
                                draw_list->AddLine(start_screen, end_screen, IM_COL32(0, 255, 0, 255), applyZoom(2.0f, zoom_level));
                            }
                        }
                        
                        // Draw waypoints safely
                        if (zoom_level > 0.5f && complete_path.size() > 2) {
                            for (size_t i = 1; i < complete_path.size() - 1; ++i) {
                                ImVec2 waypoint_screen = worldToScreen(complete_path[i], canvas_p0, zoom_level, pan_offset);
                                if (std::isfinite(waypoint_screen.x) && std::isfinite(waypoint_screen.y) &&
                                    std::abs(waypoint_screen.x) < 10000 && std::abs(waypoint_screen.y) < 10000) {
                                    draw_list->AddCircleFilled(waypoint_screen, applyZoom(3.0f, zoom_level), IM_COL32(0, 200, 0, 255));
                                }
                            }
                        }
                    }
                } catch (...) {
                    // Skip this wire if there's any problem
                    std::cout << "Error drawing this wire" << std::endl;
                    continue;
                }
            }

            // Draw junctions safely
            for (const auto& junction : circuit.getJunctions()) {
                try {
                    ImVec2 junction_screen = worldToScreen(ImVec2(junction->x, junction->y), canvas_p0, zoom_level, pan_offset);
                    float junction_radius = applyZoom(5.0f, zoom_level);
                    
                    if (junction_radius >= 1.0f && junction_radius < 50.0f &&
                        std::isfinite(junction_screen.x) && std::isfinite(junction_screen.y) &&
                        std::abs(junction_screen.x) < 10000 && std::abs(junction_screen.y) < 10000) {
                        
                        // Use different colors for ground vs regular junctions
                        ImU32 junction_color = (junction->node_id == 0) ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 0, 255);
                        ImU32 border_color = (junction->node_id == 0) ? IM_COL32(255, 255, 255, 255) : IM_COL32(0, 0, 0, 255);
                        
                        draw_list->AddCircleFilled(junction_screen, junction_radius, junction_color);
                        draw_list->AddCircle(junction_screen, junction_radius, border_color, 8, applyZoom(2.0f, zoom_level));
                    }
                } catch (...) {
                    // Skip this junction if there's any problem
                    continue;
                }
            }

            // Draw wire segments while routing (in-progress segments)
            for (const auto& segment : current_wire_segments) {
                draw_list->AddLine(segment.start, segment.end, IM_COL32(255, 255, 0, 255), applyZoom(2.0f, zoom_level));
            }

            // Draw temporary wire while in progress with proper multi-segment preview
            if (wire_mode && wire_start_component) {
                ImVec2 current_start = current_wire_segments.empty() ? 
                    wire_start_pos : current_wire_segments.back().end;
                
                // Get current mouse position and snap it
                ImVec2 mouse_world_pos = getMouseWorldPos(canvas_p0, zoom_level, pan_offset);
                ImVec2 snapped_world_pos = snapToGrid(mouse_world_pos, base_grid_step);
                ImVec2 snapped_screen_pos = worldToScreen(snapped_world_pos, canvas_p0, zoom_level, pan_offset);
                
                // Check if we're hovering over an existing wire
                auto [preview_wire, preview_intersection] = circuit.findWireAt(mouse_world_pos.x, mouse_world_pos.y);
               
               if (preview_wire) {
                   // Highlight the wire that would be connected to
                   auto preview_complete_path = preview_wire->getCompletePath();
                   for (size_t i = 0; i < preview_complete_path.size() - 1; ++i) {
                       ImVec2 start_screen = worldToScreen(preview_complete_path[i], canvas_p0, zoom_level, pan_offset);
                       ImVec2 end_screen = worldToScreen(preview_complete_path[i + 1], canvas_p0, zoom_level, pan_offset);
                       
                       draw_list->AddLine(start_screen, end_screen, IM_COL32(255, 255, 0, 150), applyZoom(4.0f, zoom_level));
                   }
                   
                   // Draw preview junction point
                   ImVec2 snapped_intersection = snapToGrid(ImVec2(preview_intersection.x, preview_intersection.y), base_grid_step);
                   ImVec2 junction_preview_screen = worldToScreen(snapped_intersection, canvas_p0, zoom_level, pan_offset);
                   draw_list->AddCircleFilled(junction_preview_screen, applyZoom(6.0f, zoom_level), IM_COL32(255, 255, 0, 200));
                   draw_list->AddCircle(junction_preview_screen, applyZoom(6.0f, zoom_level), IM_COL32(255, 255, 255, 255), 8, applyZoom(2.0f, zoom_level));
                   
                   // Draw wire from current position to junction
                   draw_list->AddLine(current_start, junction_preview_screen, IM_COL32(255, 255, 0, 128), applyZoom(2.0f, zoom_level));
               } else {
                   // Draw preview of the current segment being routed (normal waypoint mode)
                   ImVec2 corner_point;
                   if (wire_horizontal_first) {
                       corner_point = ImVec2(snapped_screen_pos.x, current_start.y);
                   } else {
                       corner_point = ImVec2(current_start.x, snapped_screen_pos.y);
                   }
                   
                   // Draw the preview segments
                   if (std::abs(corner_point.x - current_start.x) > 1.0f || 
                       std::abs(corner_point.y - current_start.y) > 1.0f) {
                       draw_list->AddLine(current_start, corner_point, IM_COL32(255, 255, 0, 128), applyZoom(2.0f, zoom_level));
                   }
                   
                   if (std::abs(corner_point.x - snapped_screen_pos.x) > 1.0f || 
                       std::abs(corner_point.y - snapped_screen_pos.y) > 1.0f) {
                       draw_list->AddLine(corner_point, snapped_screen_pos, IM_COL32(255, 255, 0, 128), applyZoom(2.0f, zoom_level));
                   }
               }
           }
           
           // Draw zoom level indicator
           std::string zoom_text = "Zoom: " + std::to_string((int)(zoom_level * 100)) + "%";
           draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p1.y - 65), IM_COL32(255, 255, 255, 200), zoom_text.c_str());
           
           // Draw pan offset indicator
           std::string pan_text = "Pan: (" + std::to_string((int)pan_offset.x) + ", " + std::to_string((int)pan_offset.y) + ")";
           draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p1.y - 45), IM_COL32(255, 255, 255, 200), pan_text.c_str());
           
           // Draw mode indicator
           if (placing_component) {
               std::string mode_text = "Placing: " + selected_component_type;
               if (preview_rotation != 0) {
                   mode_text += " (" + std::to_string(preview_rotation) + "°)";
               }
               draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p0.y + 10), IM_COL32(255, 255, 0, 255), mode_text.c_str());
           } else if (wire_mode) {
               std::string mode_text;
               if (wire_start_component) {
                   mode_text = "Wire: Click pin/wire or waypoint";
               } else {
                   mode_text = "Wire: Click first pin";
               }
               draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p0.y + 10), IM_COL32(0, 255, 255, 255), mode_text.c_str());
           } else if (selected_component) {
               std::string mode_text = "Selected: " + selected_component->name;
               draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p0.y + 10), IM_COL32(255, 255, 0, 255), mode_text.c_str());
           }
           
           // Draw junction count for debugging
           if (circuit.getJunctions().size() > 0) {
               std::string junction_text = "Junctions: " + std::to_string(circuit.getJunctions().size());
               draw_list->AddText(ImVec2(canvas_p0.x + 10, canvas_p1.y - 25), IM_COL32(255, 255, 0, 200), junction_text.c_str());
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

       // Show DC results dialog
       ShowDCResultsDialog(&show_dc_results_dialog, current_dc_analysis, circuit, dc_error_messages);

       // Handle simulation
       if(simulate){
           // Run and show simulation results
           std::cout << "Starting simulation..." << std::endl;
           
           try {
               // Clear previous results
               dc_error_messages.clear();
               current_dc_analysis = nullptr;
               
               // Validate circuit first
               auto validation_errors = circuit.validateCircuit();
               if (!validation_errors.empty()) {
                   std::cout << "Circuit validation warnings:" << std::endl;
                   for (const auto& error : validation_errors) {
                       std::cout << "  " << error << std::endl;
                       dc_error_messages.push_back(error);
                   }
               }
               
               // Generate netlist with current simulation settings
               std::string netlist_with_sim = generateNetlistWithSettings(circuit, simConfig);
               current_netlist = netlist_with_sim;
               
               // Create a temporary netlist file
               std::ofstream temp_file("temp_circuit.cir");
               temp_file << netlist_with_sim;
               temp_file.close();
               
               // Parse the circuit
               parser.parseFile("temp_circuit.cir");
               parser.printParsedElements();
               
               // Run DC analysis if enabled
               if (simConfig.dc.enabled) {
                   std::cout << "Running DC analysis..." << std::endl;
                   
                   // Create a new DC analysis instance
                   static DCAnalysis dc_analysis_instance(const_cast<std::vector<std::unique_ptr<CircuitElement>>&>(parser.getElements()), parser.getNumNodes());
                   current_dc_analysis = &dc_analysis_instance;
                   
                   // Run the analysis
                   current_dc_analysis->solve();
                   
                   // Show results dialog
                   show_dc_results_dialog = true;
               }
               
               // Run other analysis types based on settings
               if (simConfig.transient.enabled) {
                   std::cout << "Transient analysis would run here..." << std::endl;
                   // TODO: Implement transient analysis results dialog
               }
               
               if (simConfig.ac.enabled) {
                   std::cout << "AC analysis would run here..." << std::endl;
                   // TODO: Implement AC analysis results dialog
               }
               
               if (simConfig.dcSweep.enabled) {
                   std::cout << "DC sweep analysis would run here..." << std::endl;
                   // TODO: Implement DC sweep results dialog
               }
               
               std::cout << "Simulation completed successfully!" << std::endl;
               
               // Clean up temporary file
               std::remove("temp_circuit.cir");
               
           } catch (const std::exception& e) {
               std::cerr << "Simulation error: " << e.what() << std::endl;
               dc_error_messages.push_back("Simulation error: " + std::string(e.what()));
               show_dc_results_dialog = true; // Show dialog with error
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


void drawPinsZoomed(CircuitElement* component, ImVec2 canvas_offset, ImDrawList* draw_list, float zoom, ImVec2 pan) {
    for (int i = 0; i < component->getPinCount(); ++i) {
        auto positions = component->getAbsolutePinPositions();
        ImVec2 world_pin_pos = ImVec2(positions[i].first, positions[i].second);
        ImVec2 pin_pos = worldToScreen(world_pin_pos, canvas_offset, zoom, pan);
        
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
        
        float pin_radius = applyZoom(4.0f, zoom);
        if (pin_radius >= 1.0f) { // Only draw pins when they're visible
            draw_list->AddCircleFilled(pin_pos, pin_radius, color);
            draw_list->AddCircle(pin_pos, pin_radius, IM_COL32(255, 255, 255, 255), 8, applyZoom(1.0f, zoom));
        }
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

// Function to draw component preview (ghost/translucent version) with rotation, zoom, and pan
void drawComponentPreview(ImDrawList* draw_list, ImVec2 screen_pos, const std::string& component_type, int rotation, ImVec2 canvas_offset, float zoom, ImVec2 pan) {
    // Use a translucent color for preview
    ImU32 preview_color = IM_COL32(255, 255, 255, 100);  // Semi-transparent white
    ImU32 preview_text_color = IM_COL32(255, 255, 0, 120);  // Semi-transparent yellow
    
    // Helper lambda for rotating preview elements
    auto rotatePreviewPoint = [&](ImVec2 point) -> ImVec2 {
        return rotatePoint(point, screen_pos, rotation);
    };
    
    // Helper lambda for drawing rotated preview lines with zoom
    auto drawPreviewLine = [&](ImVec2 start, ImVec2 end, float thickness = 2.0f) {
        draw_list->AddLine(rotatePreviewPoint(start), rotatePreviewPoint(end), preview_color, applyZoom(thickness, zoom));
    };
    
    // Helper lambda for drawing rotated preview rectangles with zoom
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
            draw_list->AddLine(corners[i], corners[(i + 1) % 4], preview_color, applyZoom(thickness, zoom));
        }
    };
    
    // Helper lambda for drawing rotated preview text with zoom
    auto drawPreviewText = [&](ImVec2 text_pos, const char* text) {
        if (zoom < 0.5f) return; // Don't draw text when too zoomed out
        ImVec2 rotated_pos = rotatePreviewPoint(text_pos);
        draw_list->AddText(rotated_pos, preview_text_color, text);
    };
    
    if (component_type == "resistor") {
        // Draw resistor preview with rotation and zoom
        float line_len = applyZoom(25.0f, zoom);
        float rect_width = applyZoom(30.0f, zoom);
        float rect_height = applyZoom(10.0f, zoom);
        
        drawPreviewLine(ImVec2(screen_pos.x - line_len, screen_pos.y), ImVec2(screen_pos.x - line_len/2, screen_pos.y));
        drawPreviewRect(ImVec2(screen_pos.x - rect_width/2, screen_pos.y - rect_height/2), ImVec2(screen_pos.x + rect_width/2, screen_pos.y + rect_height/2));
        drawPreviewLine(ImVec2(screen_pos.x + line_len/2, screen_pos.y), ImVec2(screen_pos.x + line_len, screen_pos.y));
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - applyZoom(20.0f, zoom)), "R?");
        
    } else if (component_type == "capacitor") {
        // Draw capacitor preview with rotation and zoom
        float line_len = applyZoom(20.0f, zoom);
        float plate_height = applyZoom(20.0f, zoom);
        float gap = applyZoom(10.0f, zoom);
        
        drawPreviewLine(ImVec2(screen_pos.x - line_len, screen_pos.y), ImVec2(screen_pos.x - gap/2, screen_pos.y));
        drawPreviewLine(ImVec2(screen_pos.x - gap/2, screen_pos.y - plate_height/2), ImVec2(screen_pos.x - gap/2, screen_pos.y + plate_height/2));
        drawPreviewLine(ImVec2(screen_pos.x + gap/2, screen_pos.y - plate_height/2), ImVec2(screen_pos.x + gap/2, screen_pos.y + plate_height/2));
        drawPreviewLine(ImVec2(screen_pos.x + gap/2, screen_pos.y), ImVec2(screen_pos.x + line_len, screen_pos.y));
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - applyZoom(20.0f, zoom)), "C?");
        
    } else if (component_type == "inductor") {
        // Draw inductor preview with rotation and zoom
        float coil_radius = applyZoom(5.0f, zoom);
        float coil_spacing = applyZoom(10.0f, zoom);
        float line_len = applyZoom(25.0f, zoom);
        
        for (int i = 0; i < 3; i++) {
            float center_x = screen_pos.x - applyZoom(15.0f, zoom) + i * coil_spacing;
            ImVec2 circle_center = rotatePreviewPoint(ImVec2(center_x, screen_pos.y));
            if (coil_radius >= 1.0f) { // Only draw if visible
                draw_list->AddCircle(circle_center, coil_radius, preview_color, 8, applyZoom(1.0f, zoom));
            }
        }
        drawPreviewLine(ImVec2(screen_pos.x - line_len, screen_pos.y), ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y));
        drawPreviewLine(ImVec2(screen_pos.x + applyZoom(20.0f, zoom), screen_pos.y), ImVec2(screen_pos.x + line_len, screen_pos.y));
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - applyZoom(20.0f, zoom)), "L?");
        
    } else if (component_type == "diode") {
        // Draw diode preview with rotation and zoom
        float triangle_size = applyZoom(8.0f, zoom);
        float line_len = applyZoom(15.0f, zoom);
        
        ImVec2 triangle_points[3] = {
            ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y - triangle_size),
            ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y + triangle_size),
            ImVec2(screen_pos.x + applyZoom(5.0f, zoom), screen_pos.y)
        };
        
        // Rotate triangle points
        for (int i = 0; i < 3; i++) {
            triangle_points[i] = rotatePreviewPoint(triangle_points[i]);
        }
        
        draw_list->AddPolyline(triangle_points, 3, preview_color, true, applyZoom(2.0f, zoom));
        
        drawPreviewLine(ImVec2(screen_pos.x + applyZoom(5.0f, zoom), screen_pos.y - triangle_size), ImVec2(screen_pos.x + applyZoom(5.0f, zoom), screen_pos.y + triangle_size));
        drawPreviewLine(ImVec2(screen_pos.x - line_len, screen_pos.y), ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y));
        drawPreviewLine(ImVec2(screen_pos.x + applyZoom(5.0f, zoom), screen_pos.y), ImVec2(screen_pos.x + line_len, screen_pos.y));
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - applyZoom(25.0f, zoom)), "D?");
        
    } else if (component_type == "vsource") {
        // Draw voltage source preview with rotation and zoom
        float radius = applyZoom(15.0f, zoom);
        float line_len = applyZoom(20.0f, zoom);
        
        if (radius >= 2.0f) { // Only draw circle if visible
            draw_list->AddCircle(screen_pos, radius, preview_color, 16, applyZoom(2.0f, zoom));
        }
        
        drawPreviewLine(ImVec2(screen_pos.x, screen_pos.y - line_len), ImVec2(screen_pos.x, screen_pos.y - radius));
        drawPreviewLine(ImVec2(screen_pos.x, screen_pos.y + radius), ImVec2(screen_pos.x, screen_pos.y + line_len));
        
        if (zoom > 0.3f) { // Only draw symbols when zoomed in enough
            drawPreviewText(ImVec2(screen_pos.x - applyZoom(4.0f, zoom), screen_pos.y - applyZoom(13.0f, zoom)), "+");
            drawPreviewText(ImVec2(screen_pos.x - applyZoom(4.0f, zoom), screen_pos.y - applyZoom(2.0f, zoom)), "-");
        }
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - applyZoom(35.0f, zoom)), "V?");
        
    } else if (component_type == "ground") {
        // Draw ground preview with rotation and zoom
        float line_height = applyZoom(10.0f, zoom);
        float width1 = applyZoom(16.0f, zoom);
        float width2 = applyZoom(10.0f, zoom);
        float width3 = applyZoom(4.0f, zoom);
        float spacing = applyZoom(3.0f, zoom);
        
        drawPreviewLine(ImVec2(screen_pos.x, screen_pos.y), ImVec2(screen_pos.x, screen_pos.y + line_height));
        drawPreviewLine(ImVec2(screen_pos.x - width1/2, screen_pos.y + line_height), ImVec2(screen_pos.x + width1/2, screen_pos.y + line_height));
        drawPreviewLine(ImVec2(screen_pos.x - width2/2, screen_pos.y + line_height + spacing), ImVec2(screen_pos.x + width2/2, screen_pos.y + line_height + spacing));
        drawPreviewLine(ImVec2(screen_pos.x - width3/2, screen_pos.y + line_height + spacing*2), ImVec2(screen_pos.x + width3/2, screen_pos.y + line_height + spacing*2));
        
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y - applyZoom(20.0f, zoom)), "GND?");
        
    } else if (component_type == "nmosfet" || component_type == "pmosfet") {
        // Draw MOSFET preview with rotation and zoom
        float gate_line_height = applyZoom(20.0f, zoom);
        float gate_line_width = applyZoom(5.0f, zoom);
        float channel_height = applyZoom(5.0f, zoom);
        float channel_gap = applyZoom(3.0f, zoom);
        float drain_source_len = applyZoom(15.0f, zoom);
        float body_line_len = applyZoom(30.0f, zoom);
        
        // Gate line
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y - gate_line_height/2), 
                       ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y + gate_line_height/2));
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(25.0f, zoom), screen_pos.y), 
                       ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y));
        
        // Channel lines
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y - channel_height - channel_gap), 
                       ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y - channel_gap), 3.0f);
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y + channel_gap), 
                       ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y + channel_height + channel_gap), 3.0f);
        
        // Drain and source connections
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y - applyZoom(5.0f, zoom)), 
                       ImVec2(screen_pos.x, screen_pos.y - applyZoom(5.0f, zoom)));
        drawPreviewLine(ImVec2(screen_pos.x, screen_pos.y - applyZoom(5.0f, zoom)), 
                       ImVec2(screen_pos.x, screen_pos.y - drain_source_len));
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y + applyZoom(5.0f, zoom)), 
                       ImVec2(screen_pos.x, screen_pos.y + applyZoom(5.0f, zoom)));
        drawPreviewLine(ImVec2(screen_pos.x, screen_pos.y + applyZoom(5.0f, zoom)), 
                       ImVec2(screen_pos.x, screen_pos.y + drain_source_len));
        
        // Body connection
        drawPreviewLine(ImVec2(screen_pos.x - applyZoom(15.0f, zoom), screen_pos.y), 
                       ImVec2(screen_pos.x + applyZoom(15.0f, zoom), screen_pos.y), 1.0f);
        drawPreviewLine(ImVec2(screen_pos.x + applyZoom(15.0f, zoom), screen_pos.y), 
                       ImVec2(screen_pos.x + applyZoom(20.0f, zoom), screen_pos.y));
        
        // Arrow for NMOS vs PMOS (rotated and zoomed)
        float arrow_size = applyZoom(2.0f, zoom);
        ImVec2 arrow[3];
        if (component_type == "nmosfet") {
            arrow[0] = ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y - arrow_size);
            arrow[1] = ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y + arrow_size);
            arrow[2] = ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y);
        } else {
            arrow[0] = ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y - arrow_size);
            arrow[1] = ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y + arrow_size);
            arrow[2] = ImVec2(screen_pos.x - applyZoom(5.0f, zoom), screen_pos.y);
        }
        
        // Rotate arrow points
        for (int i = 0; i < 3; i++) {
            arrow[i] = rotatePreviewPoint(arrow[i]);
        }
        
        if (arrow_size >= 0.5f) { // Only draw arrow if visible
            draw_list->AddPolyline(arrow, 3, preview_color, true, applyZoom(1.0f, zoom));
        }
        
        std::string label = (component_type == "nmosfet") ? "NMOS?" : "PMOS?";
        drawPreviewText(ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y - applyZoom(30.0f, zoom)), label.c_str());
    }
    
    // Draw preview pins (rotated and zoomed)
    std::vector<ImVec2> pin_positions;
    
    if (component_type == "resistor" || component_type == "capacitor" || component_type == "inductor") {
        pin_positions.push_back(ImVec2(screen_pos.x - applyZoom(20.0f, zoom), screen_pos.y));
        pin_positions.push_back(ImVec2(screen_pos.x + applyZoom(20.0f, zoom), screen_pos.y));
    } else if (component_type == "diode") {
        pin_positions.push_back(ImVec2(screen_pos.x - applyZoom(10.0f, zoom), screen_pos.y));
        pin_positions.push_back(ImVec2(screen_pos.x + applyZoom(10.0f, zoom), screen_pos.y));
    } else if (component_type == "vsource") {
        pin_positions.push_back(ImVec2(screen_pos.x, screen_pos.y - applyZoom(15.0f, zoom)));
        pin_positions.push_back(ImVec2(screen_pos.x, screen_pos.y + applyZoom(15.0f, zoom)));
    } else if (component_type == "ground") {
        pin_positions.push_back(ImVec2(screen_pos.x, screen_pos.y));
    } else if (component_type == "nmosfet" || component_type == "pmosfet") {
        pin_positions.push_back(ImVec2(screen_pos.x, screen_pos.y - applyZoom(15.0f, zoom)));    // drain
        pin_positions.push_back(ImVec2(screen_pos.x - applyZoom(25.0f, zoom), screen_pos.y));    // gate
        pin_positions.push_back(ImVec2(screen_pos.x, screen_pos.y + applyZoom(15.0f, zoom)));    // source
        pin_positions.push_back(ImVec2(screen_pos.x + applyZoom(20.0f, zoom), screen_pos.y));    // bulk
    }
    
    // Draw preview pins (rotated and zoomed)
    float pin_radius = applyZoom(4.0f, zoom);
    if (pin_radius >= 1.0f) { // Only draw pins if they're visible
        for (const auto& pin_pos : pin_positions) {
            ImVec2 rotated_pin = rotatePreviewPoint(pin_pos);
            draw_list->AddCircleFilled(rotated_pin, pin_radius, IM_COL32(255, 0, 0, 100));
            draw_list->AddCircle(rotated_pin, pin_radius, IM_COL32(255, 255, 255, 120), 8, applyZoom(1.0f, zoom));
        }
    }
    
    // Draw rotation indicator (scaled with zoom)
    if (rotation != 0 && zoom > 0.3f) {
        ImVec2 indicator_pos = ImVec2(screen_pos.x + applyZoom(30.0f, zoom), screen_pos.y - applyZoom(30.0f, zoom));
        draw_list->AddText(indicator_pos, IM_COL32(0, 255, 255, 150), (std::to_string(rotation) + "°").c_str());
    }
}

// Component drawing function with rotation and zoom/pan support
void drawComponent(ImDrawList* draw_list, ImVec2 canvas_offset, CircuitElement* component, CircuitElement* selected_component, float zoom, ImVec2 pan) {
    // Transform component position to screen coordinates
    ImVec2 world_pos = ImVec2(component->x, component->y);
    ImVec2 pos = worldToScreen(world_pos, canvas_offset, zoom, pan);
    
    int rotation = component->getRotation();

    // Draw selection highlight if component is selected (scaled with zoom)
    if (component == selected_component) {
        draw_list->AddCircle(pos, applyZoom(35.0f, zoom), IM_COL32(255, 255, 0, 150), 16, 3.0f);
    }
    
    // Helper function to apply zoom to rotated drawing
    auto drawZoomedRotatedLine = [&](ImVec2 start, ImVec2 end, ImU32 color, float thickness = 2.0f) {
        ImVec2 rotated_start = rotatePoint(start, pos, rotation);
        ImVec2 rotated_end = rotatePoint(end, pos, rotation);
        draw_list->AddLine(rotated_start, rotated_end, color, applyZoom(thickness, zoom));
    };
    
    auto drawZoomedRotatedRect = [&](ImVec2 min, ImVec2 max, ImU32 color, float thickness = 2.0f) {
        ImVec2 corners[4] = {
            ImVec2(min.x, min.y), ImVec2(max.x, min.y),
            ImVec2(max.x, max.y), ImVec2(min.x, max.y)
        };
        
        for (int i = 0; i < 4; i++) {
            corners[i] = rotatePoint(corners[i], pos, rotation);
        }
        
        for (int i = 0; i < 4; i++) {
            draw_list->AddLine(corners[i], corners[(i + 1) % 4], color, applyZoom(thickness, zoom));
        }
    };
    
    auto drawZoomedRotatedText = [&](ImVec2 text_pos, ImU32 color, const char* text) {
        if (zoom < 0.5f) return; // Don't draw text when too zoomed out
        ImVec2 rotated_pos = rotatePoint(text_pos, pos, rotation);
        draw_list->AddText(rotated_pos, color, text);
    };
    
    if (component->getType() == "resistor") {
        // Draw resistor symbol with zoom
        float line_len = applyZoom(25.0f, zoom);
        float rect_width = applyZoom(30.0f, zoom);
        float rect_height = applyZoom(10.0f, zoom);
        
        drawZoomedRotatedLine(ImVec2(pos.x - line_len, pos.y), ImVec2(pos.x - line_len/2, pos.y), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedRect(ImVec2(pos.x - rect_width/2, pos.y - rect_height/2), ImVec2(pos.x + rect_width/2, pos.y + rect_height/2), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x + line_len/2, pos.y), ImVec2(pos.x + line_len, pos.y), IM_COL32(255, 255, 255, 255));
        
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y - applyZoom(20.0f, zoom)), IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y + applyZoom(10.0f, zoom)), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
    } else if (component->getType() == "capacitor") {
        // Draw capacitor symbol with zoom
        float line_len = applyZoom(20.0f, zoom);
        float plate_height = applyZoom(20.0f, zoom);
        float gap = applyZoom(10.0f, zoom);
        
        drawZoomedRotatedLine(ImVec2(pos.x - line_len, pos.y), ImVec2(pos.x - gap/2, pos.y), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x - gap/2, pos.y - plate_height/2), ImVec2(pos.x - gap/2, pos.y + plate_height/2), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x + gap/2, pos.y - plate_height/2), ImVec2(pos.x + gap/2, pos.y + plate_height/2), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x + gap/2, pos.y), ImVec2(pos.x + line_len, pos.y), IM_COL32(255, 255, 255, 255));
        
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y - applyZoom(20.0f, zoom)), IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y + applyZoom(10.0f, zoom)), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
    } else if (component->getType() == "vsource") {
        // Draw voltage source symbol with zoom
        float radius = applyZoom(15.0f, zoom);
        float line_len = applyZoom(20.0f, zoom);
        
        draw_list->AddCircle(pos, radius, IM_COL32(255, 255, 255, 255), 16, applyZoom(2.0f, zoom));
        drawZoomedRotatedLine(ImVec2(pos.x, pos.y - line_len), ImVec2(pos.x, pos.y - radius), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x, pos.y + radius), ImVec2(pos.x, pos.y + line_len), IM_COL32(255, 255, 255, 255));
        
        if (zoom > 0.3f) { // Only draw symbols when zoomed in enough
            drawZoomedRotatedText(ImVec2(pos.x - applyZoom(4.0f, zoom), pos.y - applyZoom(13.0f, zoom)), IM_COL32(255, 255, 255, 255), "+");
            drawZoomedRotatedText(ImVec2(pos.x - applyZoom(4.0f, zoom), pos.y - applyZoom(2.0f, zoom)), IM_COL32(255, 255, 255, 255), "-");
        }
        
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y - applyZoom(35.0f, zoom)), IM_COL32(255, 255, 0, 255), component->name.c_str());
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y + applyZoom(25.0f, zoom)), IM_COL32(200, 200, 200, 255), component->getValue().c_str());
        
    } else if (component->getType() == "ground") {
        // Draw ground symbol with zoom
        float line_height = applyZoom(10.0f, zoom);
        float width1 = applyZoom(16.0f, zoom);
        float width2 = applyZoom(10.0f, zoom);
        float width3 = applyZoom(4.0f, zoom);
        
        drawZoomedRotatedLine(ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + line_height), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x - width1/2, pos.y + line_height), ImVec2(pos.x + width1/2, pos.y + line_height), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x - width2/2, pos.y + line_height + applyZoom(3.0f, zoom)), ImVec2(pos.x + width2/2, pos.y + line_height + applyZoom(3.0f, zoom)), IM_COL32(255, 255, 255, 255));
        drawZoomedRotatedLine(ImVec2(pos.x - width3/2, pos.y + line_height + applyZoom(6.0f, zoom)), ImVec2(pos.x + width3/2, pos.y + line_height + applyZoom(6.0f, zoom)), IM_COL32(255, 255, 255, 255));
        
        drawZoomedRotatedText(ImVec2(pos.x - applyZoom(10.0f, zoom), pos.y - applyZoom(20.0f, zoom)), IM_COL32(255, 255, 0, 255), component->name.c_str());
        
    } 
    // Add similar zoom/pan support for other component types (inductor, diode, mosfets...)
    // Following the same pattern as above
    
    // Draw pins with zoom
    drawPinsZoomed(component, canvas_offset, draw_list, zoom, pan);
}
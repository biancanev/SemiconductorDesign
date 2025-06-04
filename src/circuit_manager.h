#pragma once
#include "parser/circuit_element.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <iostream>

// Wire class updated for n-pin support
class Wire {
public:
    CircuitElement* component1;
    int pin1;
    CircuitElement* component2;
    int pin2;
    int node_id;
    std::vector<ImVec2> path_points; // Store the complete path in world coordinates
    
    Wire(CircuitElement* comp1, int p1, CircuitElement* comp2, int p2, int node, const std::vector<ImVec2>& path = {}) 
        : component1(comp1), pin1(p1), component2(comp2), pin2(p2), node_id(node), path_points(path) {}
    
    // Get start position in world coordinates
    std::pair<float, float> getStartPos() const {
        if (component1) {
            auto pins = component1->getAbsolutePinPositions();
            if (pin1 >= 0 && pin1 < static_cast<int>(pins.size())) {
                return pins[pin1];
            }
        }
        // If no component1, this might be a junction wire - return first path point or (0,0)
        if (!path_points.empty()) {
            return {path_points[0].x, path_points[0].y};
        }
        return {0, 0};
    }

    std::pair<float, float> getEndPos() const {
        if (component2) {
            auto pins = component2->getAbsolutePinPositions();
            if (pin2 >= 0 && pin2 < static_cast<int>(pins.size())) {
                return pins[pin2];
            }
        }
        // If no component2, this might be a junction wire - return last path point or (0,0)
        if (!path_points.empty()) {
            return {path_points.back().x, path_points.back().y};
        }
        return {0, 0};
    }
    
    // Get the complete wire path including start, intermediate points, and end
    std::vector<ImVec2> getCompletePath() const {
        std::vector<ImVec2> complete_path;
        
        try {
            // Add start point (from component1)
            if (component1) {
                auto start = getStartPos();
                complete_path.emplace_back(start.first, start.second);
            }
            
            // Add intermediate points
            for (const auto& point : path_points) {
                complete_path.push_back(point);
            }
            
            // Add end point (from component2, if it exists)
            if (component2) {
                auto end = getEndPos();
                complete_path.emplace_back(end.first, end.second);
            } else if (!path_points.empty()) {
                // This is a junction wire - the last path point IS the endpoint
                // Don't add it again since it's already in path_points
                // But ensure we have at least 2 points for drawing
                if (complete_path.size() == 1) {
                    // If we only have the start point, duplicate the last path point
                    complete_path.push_back(path_points.back());
                }
            }
            
            // Safety check - ensure we have at least 2 points for drawing
            if (complete_path.size() < 2) {
                if (complete_path.size() == 1) {
                    complete_path.push_back(complete_path[0]);
                } else {
                    complete_path = {{0, 0}, {0, 0}};
                }
            }
            
            return complete_path;
            
        } catch (...) {
            // Return safe default if anything goes wrong
            return {{0, 0}, {0, 0}};
        }
    }
    
    // Get descriptive string for debugging
    std::string getDescription() const {
        std::string desc = component1->name + "." + component1->getPinName(pin1) + 
                          " -> " + component2->name + "." + component2->getPinName(pin2) + 
                          " (node " + std::to_string(node_id) + ")";
        if (!path_points.empty()) {
            desc += " [" + std::to_string(path_points.size()) + " waypoints]";
        }
        return desc;
    }
};

struct Junction{
    int node_id;
    float x, y;
    std::vector<Wire*> connected_wires;

    Junction(int node, float px, float py) : node_id(node), x(px), y(py) {}
    
    // Default constructor
    Junction() : node_id(-1), x(0.0f), y(0.0f) {}
};

class CircuitManager {
private:
    std::vector<std::unique_ptr<CircuitElement>> components;
    std::vector<std::unique_ptr<Wire>> wires;
    std::vector<std::unique_ptr<Junction>> junctions;
    std::unordered_map<std::string, int> component_counters;
    std::unordered_set<int> used_nodes;
    int next_node_id = 1;  // 0 is reserved for ground
    
    bool isPointOnLineSegment(float px, float py, float x1, float y1, 
                             float x2, float y2, float tolerance) {
        // Calculate distance from point to line segment
        float A = px - x1;
        float B = py - y1;
        float C = x2 - x1;
        float D = y2 - y1;
        
        float dot = A * C + B * D;
        float len_sq = C * C + D * D;
        
        if (len_sq == 0) return false; // Line segment has zero length
        
        float param = dot / len_sq;
        
        // Check if closest point is within the line segment
        if (param < 0 || param > 1) return false;
        
        // Calculate closest point on line
        float xx = x1 + param * C;
        float yy = y1 + param * D;
        
        // Check if distance is within tolerance
        float dx = px - xx;
        float dy = py - yy;
        return (dx * dx + dy * dy) <= tolerance * tolerance;
    }
    
    ImVec2 getClosestPointOnLine(float px, float py, float x1, float y1, 
                                float x2, float y2) {
        float A = px - x1;
        float B = py - y1;
        float C = x2 - x1;
        float D = y2 - y1;
        
        float dot = A * C + B * D;
        float len_sq = C * C + D * D;
        
        float param = dot / len_sq;
        param = std::max(0.0f, std::min(1.0f, param)); // Clamp to segment
        
        return ImVec2(x1 + param * C, y1 + param * D);
    }

    
    // Helper to merge two nodes across all components
    void mergeNodes(int old_node, int new_node) {
        if (old_node == new_node) return;
        
        std::cout << "Merging node " << old_node << " into node " << new_node << std::endl;
        
        // Update all pins across all components
        for (auto& component : components) {
            for (int i = 0; i < component->getPinCount(); ++i) {
                if (component->getNodeForPin(i) == old_node) {
                    component->setNodeForPin(i, new_node);
                }
            }
        }
        
        // Update all wires
        for (auto& wire : wires) {
            if (wire->node_id == old_node) {
                wire->node_id = new_node;
            }
        }
        
        // Update node tracking
        used_nodes.erase(old_node);
        used_nodes.insert(new_node);
    }
    
    // Ground a specific node (make it node 0)
    void groundNode(int node_id) {
        if (node_id == 0) return;  // Already ground
        
        std::cout << "Grounding node " << node_id << std::endl;
        
        // Change all components using this node to use ground (0)
        for (auto& component : components) {
            for (int i = 0; i < component->getPinCount(); ++i) {
                if (component->getNodeForPin(i) == node_id) {
                    component->setNodeForPin(i, 0);
                    std::cout << "  Grounded " << component->name << " pin " << i << std::endl;
                }
            }
        }
        
        // Update wires
        for (auto& wire : wires) {
            if (wire->node_id == node_id) {
                wire->node_id = 0;
                std::cout << "  Grounded wire: " << wire->getDescription() << std::endl;
            }
        }
        
        // Update junctions
        for (auto& junction : junctions) {
            if (junction->node_id == node_id) {
                junction->node_id = 0;
                std::cout << "  Grounded junction at (" << junction->x << ", " << junction->y << ")" << std::endl;
            }
        }
        
        used_nodes.erase(node_id);
        used_nodes.insert(0);
        
        std::cout << "Node " << node_id << " successfully grounded" << std::endl;
    }
    
    // Get all nodes connected to a specific node (for ground propagation)
    std::unordered_set<int> getConnectedNodes(int start_node) const {
        std::unordered_set<int> connected;
        std::vector<int> to_check = {start_node};
        
        while (!to_check.empty()) {
            int current = to_check.back();
            to_check.pop_back();
            
            if (connected.count(current)) continue;
            connected.insert(current);
            
            // Find all directly connected nodes via wires
            for (const auto& wire : wires) {
                if (wire->node_id == current) {
                    int node1 = wire->component1->getNodeForPin(wire->pin1);
                    int node2 = wire->component2->getNodeForPin(wire->pin2);
                    
                    if (!connected.count(node1)) to_check.push_back(node1);
                    if (!connected.count(node2)) to_check.push_back(node2);
                }
            }
        }
        
        return connected;
    }
    
public:
    CircuitManager() {
        // Initialize component counters for different types
        component_counters["R"] = 0;   // Resistors
        component_counters["C"] = 0;   // Capacitors
        component_counters["L"] = 0;   // Inductors
        component_counters["V"] = 0;   // Voltage sources
        component_counters["I"] = 0;   // Current sources
        component_counters["Q"] = 0;   // Transistors
        component_counters["U"] = 0;   // ICs/Op-amps
        component_counters["D"] = 0;   // Diodes
        component_counters["GND"] = 0; // Ground symbols
        
        used_nodes.insert(0);  // Ground is always present
    }

    bool connectToWire(CircuitElement* component, int pin, Wire* existing_wire, float junction_x, float junction_y) {
        if (!component || !existing_wire) {
            std::cerr << "Error: Invalid component or wire for junction connection" << std::endl;
            return false;
        }
        
        std::cout << "Connecting " << component->name << " pin " << pin 
                << " to existing wire at (" << junction_x << ", " << junction_y << ")" << std::endl;
        
        // Get the existing wire's complete path and node
        auto existing_path = existing_wire->getCompletePath();
        int existing_node = existing_wire->node_id;
        std::cout << "Existing wire has " << existing_path.size() << " path points and node " << existing_node << std::endl;
        
        // Determine the final node for this connection
        int final_node;
        if (component->getType() == "ground") {
            final_node = 0;
            std::cout << "Ground connection detected - will ground the network" << std::endl;
        } else if (existing_node == 0) {
            final_node = 0;
            std::cout << "Connecting to already grounded wire" << std::endl;
        } else {
            final_node = existing_node;
            std::cout << "Regular junction connection to node " << final_node << std::endl;
        }
        
        // Handle grounding if needed
        if (final_node == 0 && existing_node != 0) {
            std::cout << "Grounding network..." << std::endl;
            
            for (auto& comp : components) {
                for (int i = 0; i < comp->getPinCount(); ++i) {
                    if (comp->getNodeForPin(i) == existing_node) {
                        comp->setNodeForPin(i, 0);
                        std::cout << "  Grounded " << comp->name << " pin " << i << std::endl;
                    }
                }
            }
            
            for (auto& wire : wires) {
                if (wire->node_id == existing_node) {
                    wire->node_id = 0;
                    std::cout << "  Grounded wire" << std::endl;
                }
            }
            
            for (auto& junction : junctions) {
                if (junction->node_id == existing_node) {
                    junction->node_id = 0;
                    std::cout << "  Grounded junction" << std::endl;
                }
            }
            
            used_nodes.erase(existing_node);
            used_nodes.insert(0);
        }
        
        // Set the component's pin to the final node
        component->setNodeForPin(pin, final_node);
        std::cout << "Set " << component->name << " pin " << pin << " to node " << final_node << std::endl;
        
        try {
            // STEP 1: Split the existing wire at the junction point
            // Find where the junction point lies on the existing wire's path
            ImVec2 junction_point(junction_x, junction_y);
            int split_segment = -1;
            float best_distance = std::numeric_limits<float>::max();
            
            // Find the closest segment to split
            for (size_t i = 0; i < existing_path.size() - 1; ++i) {
                ImVec2 seg_start = existing_path[i];
                ImVec2 seg_end = existing_path[i + 1];
                
                if (isPointOnLineSegment(junction_x, junction_y, seg_start.x, seg_start.y, 
                                    seg_end.x, seg_end.y, 8.0f)) {
                    float dist = std::sqrt((junction_x - seg_start.x) * (junction_x - seg_start.x) +
                                        (junction_y - seg_start.y) * (junction_y - seg_start.y)) +
                            std::sqrt((junction_x - seg_end.x) * (junction_x - seg_end.x) +
                                    (junction_y - seg_end.y) * (junction_y - seg_end.y));
                    if (dist < best_distance) {
                        best_distance = dist;
                        split_segment = static_cast<int>(i);
                    }
                }
            }
            
            if (split_segment == -1) {
                std::cerr << "Could not find segment to split!" << std::endl;
                return false;
            }
            
            std::cout << "Splitting existing wire at segment " << split_segment << std::endl;
            
            // STEP 2: Create two new wires to replace the existing wire
            std::vector<ImVec2> path1, path2;
            
            // Path1: from start to junction point
            for (int i = 0; i <= split_segment; ++i) {
                if (i < static_cast<int>(existing_path.size())) {
                    path1.push_back(existing_path[i]);
                }
            }
            // Add junction point if it's not already the last point
            if (path1.empty() || 
                (std::abs(path1.back().x - junction_x) > 1.0f || std::abs(path1.back().y - junction_y) > 1.0f)) {
                path1.push_back(junction_point);
            }
            
            // Path2: from junction point to end
            path2.push_back(junction_point);
            for (size_t i = split_segment + 1; i < existing_path.size(); ++i) {
                path2.push_back(existing_path[i]);
            }
            
            std::cout << "Created path1 with " << path1.size() << " points and path2 with " << path2.size() << " points" << std::endl;
            
            // STEP 3: Create replacement wires with preserved waypoints
            
            // Wire 1: from original start to junction
            std::vector<ImVec2> wire1_waypoints;
            if (path1.size() > 2) {
                // Copy waypoints between start and junction (exclude endpoints)
                for (size_t i = 1; i < path1.size() - 1; ++i) {
                    wire1_waypoints.push_back(path1[i]);
                }
            }
            
            auto wire1 = std::make_unique<Wire>(existing_wire->component1, existing_wire->pin1, 
                                            nullptr, -1, final_node, wire1_waypoints);
            // Add the junction point as the final waypoint
            if (!wire1_waypoints.empty() || path1.size() > 1) {
                wire1->path_points.push_back(junction_point);
            }
            
            // Wire 2: from junction to original end
            std::vector<ImVec2> wire2_waypoints;
            wire2_waypoints.push_back(junction_point); // Start with junction point
            if (path2.size() > 2) {
                // Copy waypoints between junction and end (exclude endpoints)
                for (size_t i = 1; i < path2.size() - 1; ++i) {
                    wire2_waypoints.push_back(path2[i]);
                }
            }
            
            auto wire2 = std::make_unique<Wire>(nullptr, -1, existing_wire->component2, 
                                            existing_wire->pin2, final_node, wire2_waypoints);
            
            // STEP 4: Create the new component-to-junction wire
            auto pin_positions = component->getAbsolutePinPositions();
            ImVec2 component_pin_pos(pin_positions[pin].first, pin_positions[pin].second);
            
            std::vector<ImVec2> junction_wire_path;
            float dx = junction_x - component_pin_pos.x;
            float dy = junction_y - component_pin_pos.y;
            if (dx*dx + dy*dy > 1.0f) {
                junction_wire_path.push_back(junction_point);
            }
            
            auto junction_wire = std::make_unique<Wire>(component, pin, nullptr, -1, final_node, junction_wire_path);
            
            // STEP 5: Create or find the junction
            Junction* junction = nullptr;
            for (auto& j : junctions) {
                float dx = j->x - junction_x;
                float dy = j->y - junction_y;
                if (dx*dx + dy*dy < 4.0f) {
                    junction = j.get();
                    junction->node_id = final_node;
                    std::cout << "Found existing junction, updated to node " << final_node << std::endl;
                    break;
                }
            }
            
            if (!junction) {
                auto new_junction = std::make_unique<Junction>(final_node, junction_x, junction_y);
                junction = new_junction.get();
                junctions.push_back(std::move(new_junction));
                std::cout << "Created new junction at (" << junction_x << ", " << junction_y << ")" << std::endl;
            }
            
            // STEP 6: Store wire pointers before moving them
            Wire* wire1_ptr = wire1.get();
            Wire* wire2_ptr = wire2.get();
            Wire* junction_wire_ptr = junction_wire.get();
            
            // STEP 7: Remove the old wire and add the new ones
            auto wire_it = std::find_if(wires.begin(), wires.end(), 
                [existing_wire](const std::unique_ptr<Wire>& w) { return w.get() == existing_wire; });
            
            if (wire_it != wires.end()) {
                wires.erase(wire_it);
                std::cout << "Removed original wire" << std::endl;
            }
            
            // Add the new wires
            wires.push_back(std::move(wire1));
            wires.push_back(std::move(wire2));
            wires.push_back(std::move(junction_wire));
            
            // STEP 8: Update junction's connected wires
            junction->connected_wires.clear(); // Clear and rebuild
            junction->connected_wires.push_back(wire1_ptr);
            junction->connected_wires.push_back(wire2_ptr);
            junction->connected_wires.push_back(junction_wire_ptr);
            
            std::cout << "Junction connection completed successfully" << std::endl;
            std::cout << "Junction now has " << junction->connected_wires.size() << " connected wires" << std::endl;
            std::cout << "Wire1 waypoints: " << wire1_ptr->path_points.size() << std::endl;
            std::cout << "Wire2 waypoints: " << wire2_ptr->path_points.size() << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Exception creating junction: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "Unknown exception creating junction" << std::endl;
            return false;
        }
    }

    std::pair<Wire*, ImVec2> findWireAt(float x, float y, float tolerance = 8.0f) {
        for (auto& wire : wires) {
            auto complete_path = wire->getCompletePath();
            
            // Check each segment of the wire path
            for (size_t i = 0; i < complete_path.size() - 1; ++i) {
                ImVec2 start = complete_path[i];
                ImVec2 end = complete_path[i + 1];
                
                if (isPointOnLineSegment(x, y, start.x, start.y, end.x, end.y, tolerance)) {
                    ImVec2 intersection = getClosestPointOnLine(x, y, start.x, start.y, end.x, end.y);
                    return {wire.get(), intersection};
                }
            }
        }
        return {nullptr, ImVec2(0, 0)};
    }
    
    int createJunctionNode(float x, float y, Wire* existing_wire) {
        if (!existing_wire) {
            std::cerr << "Error: Cannot create junction with null wire" << std::endl;
            return -1;
        }
        
        // Create new junction node
        int junction_node = next_node_id++;
        used_nodes.insert(junction_node);
        
        std::cout << "Creating junction node " << junction_node << " at (" << x << ", " << y << ")" << std::endl;
        
        // Create junction
        auto junction = std::make_unique<Junction>(junction_node, x, y);
        junction->connected_wires.push_back(existing_wire);
        
        // Don't merge nodes yet - just update the existing wire's node
        // The mergeNodes call will happen when we connect the component
        std::cout << "Junction created successfully with node " << junction_node << std::endl;
        
        Junction* junction_ptr = junction.get();
        junctions.push_back(std::move(junction));
        
        return junction_node;
    }
    
    Junction* findJunctionByNode(int node_id) {
        for (auto& junction : junctions) {
            if (junction->node_id == node_id) {
                return junction.get();
            }
        }
        return nullptr;
    }
    
    const std::vector<std::unique_ptr<Junction>>& getJunctions() const {
        return junctions;
    }
    
    // Add a component to the circuit
    CircuitElement* addComponent(const std::string& type, float x, float y) {
        std::string prefix = getComponentPrefix(type);
        std::string name = prefix + std::to_string(++component_counters[prefix]);
        
        std::unique_ptr<CircuitElement> component;
        
        if (type == "resistor") {
            component = std::make_unique<Resistor>(name, 1000.0);
        } else if (type == "capacitor") {
            component = std::make_unique<Capacitor>(name, 1e-6);
        } else if (type == "vsource") {
            component = std::make_unique<VoltageSource>(name, 5.0);
        } else if (type == "ground") {
            component = std::make_unique<Ground>(name);
        }else if (type == "inductor") {
            component = std::make_unique<Inductor>(name, 1e-6);  // 1µH default
        } else if (type == "diode") {
            component = std::make_unique<Diode>(name);
        } else if (type == "nmosfet") {
            component = std::make_unique<NMOSFET>(name);
        } else if (type == "pmosfet") {
            component = std::make_unique<PMOSFET>(name);
        }else if (type == "opamp") {
            component = std::make_unique<OpAmp>(name);
        } else {
            std::cerr << "Unknown component type: " << type << std::endl;
            return nullptr;
        }
        
        component->setPosition(x, y);
        
        CircuitElement* ptr = component.get();
        components.push_back(std::move(component));
        
        std::cout << "Added " << ptr->name << " (" << ptr->getType() << ") with " 
                  << ptr->getPinCount() << " pins" << std::endl;
        
        return ptr;
    }
    
    // Find component and pin at a specific position
    std::pair<CircuitElement*, int> findPinAt(float x, float y) {
        for (auto& component : components) {
            int pin = component->getPinAt(x, y);
            if (pin != -1) {
                return {component.get(), pin};
            }
        }
        return {nullptr, -1};
    }
    
    // Connect two pins with a wire
    bool connectPins(CircuitElement* comp1, int pin1, CircuitElement* comp2, int pin2, const std::vector<ImVec2>& path) {
        if (!comp1 || !comp2 || comp1 == comp2) {
            std::cerr << "Invalid components for connection" << std::endl;
            return false;
        }
        
        // Validate pin indices
        if (pin1 < 0 || pin1 >= comp1->getPinCount()) {
            std::cerr << "Invalid pin index " << pin1 << " for " << comp1->name 
                    << " (has " << comp1->getPinCount() << " pins)" << std::endl;
            return false;
        }
        
        if (pin2 < 0 || pin2 >= comp2->getPinCount()) {
            std::cerr << "Invalid pin index " << pin2 << " for " << comp2->name 
                    << " (has " << comp2->getPinCount() << " pins)" << std::endl;
            return false;
        }
        
        // Handle ground connections specially
        if (comp1->getType() == "ground") {
            return connectToGround(comp2, pin2, comp1, pin1, path);
        }
        
        if (comp2->getType() == "ground") {
            return connectToGround(comp1, pin1, comp2, pin2, path);
        }
        
        // Regular pin-to-pin connection
        int node1_id = comp1->getNodeForPin(pin1);
        int node2_id = comp2->getNodeForPin(pin2);
        int final_node_id;
        
        if (node1_id == -1 && node2_id == -1) {
            // Both pins unconnected - create new node
            final_node_id = next_node_id++;
            used_nodes.insert(final_node_id);
        } else if (node1_id != -1 && node2_id == -1) {
            // Pin1 connected, pin2 not - use pin1's node
            final_node_id = node1_id;
        } else if (node1_id == -1 && node2_id != -1) {
            // Pin2 connected, pin1 not - use pin2's node
            final_node_id = node2_id;
        } else if (node1_id == node2_id) {
            // Already connected to same node
            std::cout << "Pins already connected to same node " << node1_id << std::endl;
            return false;
        } else {
            // Both connected to different nodes - merge them
            final_node_id = node1_id;
            mergeNodes(node2_id, node1_id);
        }
        
        // Set the node IDs
        comp1->setNodeForPin(pin1, final_node_id);
        comp2->setNodeForPin(pin2, final_node_id);
        
        // Create wire with path
        auto wire = std::make_unique<Wire>(comp1, pin1, comp2, pin2, final_node_id, path);
        
        std::cout << "Connected " << comp1->name << "." << comp1->getPinName(pin1) 
                << " to " << comp2->name << "." << comp2->getPinName(pin2) 
                << " (node " << final_node_id << ") with " << path.size() << " waypoints" << std::endl;
        
        wires.push_back(std::move(wire));
        return true;
    }

    // Add ground connection with path
    bool connectToGround(CircuitElement* component, int pin, CircuitElement* ground_symbol, int ground_pin, const std::vector<ImVec2>& path) {
        if (!component || !ground_symbol) return false;
        
        int current_node = component->getNodeForPin(pin);
        
        if (current_node == 0) {
            std::cout << component->name << "." << component->getPinName(pin) 
                    << " already connected to ground" << std::endl;
            return false;
        }
        
        if (current_node != -1) {
            // Ground an existing node
            groundNode(current_node);
        } else {
            // Direct connection to ground
            component->setNodeForPin(pin, 0);
        }
        
        ground_symbol->setNodeForPin(ground_pin, 0);
        
        // Create wire to ground with path
        auto wire = std::make_unique<Wire>(component, pin, ground_symbol, ground_pin, 0, path);
        
        std::cout << "Connected " << component->name << "." << component->getPinName(pin) 
                << " to ground with " << path.size() << " waypoints" << std::endl;
        
        wires.push_back(std::move(wire));
        return true;
    }
    
    // Generate SPICE netlist from circuit
    std::string generateNetlist() const {
        std::string netlist = "* Generated SPICE Netlist\n";
        
        // Add components (only fully connected ones)
        for (const auto& component : components) {
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
        
        netlist += ".end\n";
        return netlist;
    }
    
    // Check if circuit has ground reference
    bool hasGroundReference() const {
        for (const auto& component : components) {
            for (int i = 0; i < component->getPinCount(); ++i) {
                if (component->getNodeForPin(i) == 0) {
                    return true;
                }
            }
        }
        return false;
    }
    
    // Get validation errors
    std::vector<std::string> validateCircuit() const {
        std::vector<std::string> errors;
        
        if (components.empty()) {
            errors.push_back("No components in circuit");
            return errors;
        }
        
        if (!hasGroundReference()) {
            errors.push_back("Circuit has no ground reference! Add a ground symbol and connect it to your circuit.");
        }
        
        // Check for unconnected pins
        int total_unconnected = 0;
        for (const auto& component : components) {
            if (component->getType() == "ground") continue;
            
            int unconnected = component->getUnconnectedPinCount();
            if (unconnected > 0) {
                total_unconnected += unconnected;
                errors.push_back(component->name + " has " + std::to_string(unconnected) + " unconnected pins");
            }
        }
        
        // Check for voltage sources
        bool has_voltage_source = false;
        for (const auto& component : components) {
            if (component->getType() == "vsource" && component->isFullyConnected()) {
                has_voltage_source = true;
                break;
            }
        }
        
        if (!has_voltage_source) {
            errors.push_back("No connected voltage source found");
        }
        
        return errors;
    }
    
    // Debug: Print circuit topology
    void printCircuitTopology() const {
        std::cout << "\n=== Circuit Topology ===" << std::endl;
        std::cout << "Components: " << components.size() << std::endl;
        std::cout << "Wires: " << wires.size() << std::endl;
        std::cout << "Used nodes: ";
        for (int node : used_nodes) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
        
        // Print component pin assignments
        for (const auto& component : components) {
            std::cout << component->name << " (" << component->getType() << "): ";
            for (int i = 0; i < component->getPinCount(); ++i) {
                std::cout << component->getPinName(i) << "=" << component->getNodeForPin(i) << " ";
            }
            std::cout << std::endl;
        }
        
        // Print wire connections
        std::cout << "Wire connections:" << std::endl;
        for (const auto& wire : wires) {
            std::cout << "  " << wire->getDescription() << std::endl;
        }
        std::cout << "=========================" << std::endl;
    }
    
    // Getters
    const std::vector<std::unique_ptr<CircuitElement>>& getComponents() const {
        return components;
    }
    
    const std::vector<std::unique_ptr<Wire>>& getWires() const {
        return wires;
    }
    
    int getNodeCount() const {
        return static_cast<int>(used_nodes.size());
    }
    
    const std::unordered_set<int>& getUsedNodes() const {
        return used_nodes;
    }
    
    // Clear the entire circuit
    void clear() {
        components.clear();
        wires.clear();
        junctions.clear();
        used_nodes.clear();
        used_nodes.insert(0);  // Ground always present
        
        // Reset counters
        for (auto& counter : component_counters) {
            counter.second = 0;
        }
        
        next_node_id = 1;
        
        std::cout << "Circuit cleared" << std::endl;
    }

private:
    std::string getComponentPrefix(const std::string& type) {
        if (type == "resistor") return "R";
        if (type == "capacitor") return "C";
        if (type == "inductor") return "L";
        if (type == "vsource") return "V";
        if (type == "isource") return "I";
        if (type == "inductor") return "L";
        if (type == "diode") return "D";
        if (type == "nmosfet" || type == "pmosfet") return "M";
        if (type == "opamp" || type == "ic") return "U";
        if (type == "ground") return "GND";
        return "X";
    }
};
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
    
    Wire(CircuitElement* comp1, int p1, CircuitElement* comp2, int p2, int node) 
        : component1(comp1), pin1(p1), component2(comp2), pin2(p2), node_id(node) {}
    
    // Get start position in world coordinates
    std::pair<float, float> getStartPos() const {
        auto pins = component1->getAbsolutePinPositions();
        if (pin1 >= 0 && pin1 < static_cast<int>(pins.size())) {
            return pins[pin1];
        }
        return {0, 0};
    }
    
    // Get end position in world coordinates
    std::pair<float, float> getEndPos() const {
        auto pins = component2->getAbsolutePinPositions();
        if (pin2 >= 0 && pin2 < static_cast<int>(pins.size())) {
            return pins[pin2];
        }
        return {0, 0};
    }
    
    // Get descriptive string for debugging
    std::string getDescription() const {
        return component1->name + "." + component1->getPinName(pin1) + 
               " -> " + component2->name + "." + component2->getPinName(pin2) + 
               " (node " + std::to_string(node_id) + ")";
    }
};

class CircuitManager {
private:
    std::vector<std::unique_ptr<CircuitElement>> components;
    std::vector<std::unique_ptr<Wire>> wires;
    std::unordered_map<std::string, int> component_counters;
    std::unordered_set<int> used_nodes;
    int next_node_id = 1;  // 0 is reserved for ground
    
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
                }
            }
        }
        
        // Update wires
        for (auto& wire : wires) {
            if (wire->node_id == node_id) {
                wire->node_id = 0;
            }
        }
        
        used_nodes.erase(node_id);
        used_nodes.insert(0);
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
    bool connectPins(CircuitElement* comp1, int pin1, CircuitElement* comp2, int pin2) {
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
            return connectToGround(comp2, pin2, comp1, pin1);
        }
        
        if (comp2->getType() == "ground") {
            return connectToGround(comp1, pin1, comp2, pin2);
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
        
        // Create wire
        auto wire = std::make_unique<Wire>(comp1, pin1, comp2, pin2, final_node_id);
        
        std::cout << "Connected " << comp1->name << "." << comp1->getPinName(pin1) 
                  << " to " << comp2->name << "." << comp2->getPinName(pin2) 
                  << " (node " << final_node_id << ")" << std::endl;
        
        wires.push_back(std::move(wire));
        return true;
    }
    
    // Connect a component pin to ground
    bool connectToGround(CircuitElement* component, int pin, CircuitElement* ground_symbol, int ground_pin) {
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
        
        // Create wire to ground
        auto wire = std::make_unique<Wire>(component, pin, ground_symbol, ground_pin, 0);
        
        std::cout << "Connected " << component->name << "." << component->getPinName(pin) 
                  << " to ground" << std::endl;
        
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
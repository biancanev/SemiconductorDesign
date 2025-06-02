#ifndef CIRCUIT_ELEMENT_H
#define CIRCUIT_ELEMENT_H

#include <vector>
#include <string>
#include <cmath>

// Pin structure to hold pin information
struct Pin {
    std::string name;        // e.g., "anode", "cathode", "base", "collector", "emitter"
    int node_id = -1;        // Connected node (-1 = unconnected)
    float rel_x, rel_y;      // Position relative to component center
    
    Pin(const std::string& name, float x, float y) 
        : name(name), node_id(-1), rel_x(x), rel_y(y) {}
};

class CircuitElement{
public:
    std::string name;
    std::vector<Pin> pins;           // Vector of pins instead of node1, node2
    std::vector<std::string> nodes;  // Keep for compatibility
    
    // Add GUI properties
    float x = 0.0f, y = 0.0f;
    
    CircuitElement(const std::string& name) : name(name), x(0), y(0) {}
    virtual ~CircuitElement() = default;
    
    // Add GUI methods
    virtual std::string getType() const = 0;
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string& value) = 0;
    virtual std::string toSpiceLine() const = 0;
    
    // GUI helper methods
    void setPosition(float px, float py) { x = px; y = py; }
    
    bool isAt(float px, float py, float tolerance = 20.0f) const {
        float dx = x - px, dy = y - py;
        return (dx*dx + dy*dy) <= tolerance*tolerance;
    }
    
    // Get pin positions in world coordinates
    std::vector<std::pair<float, float>> getAbsolutePinPositions() const {
        std::vector<std::pair<float, float>> positions;
        for (const auto& pin : pins) {
            positions.emplace_back(x + pin.rel_x, y + pin.rel_y);
        }
        return positions;
    }
    
    // Check if a point is near a pin and return pin index
    int getPinAt(float px, float py, float tolerance = 8.0f) const {
        auto positions = getAbsolutePinPositions();
        for (size_t i = 0; i < positions.size(); ++i) {
            float dx = positions[i].first - px;
            float dy = positions[i].second - py;
            if (dx*dx + dy*dy <= tolerance*tolerance) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }
    
    // Get node ID for a specific pin
    int getNodeForPin(int pin_index) const {
        if (pin_index >= 0 && pin_index < static_cast<int>(pins.size())) {
            return pins[pin_index].node_id;
        }
        return -1;
    }
    
    // Set node ID for a specific pin
    void setNodeForPin(int pin_index, int node_id) {
        if (pin_index >= 0 && pin_index < static_cast<int>(pins.size())) {
            pins[pin_index].node_id = node_id;
        }
    }
    
    // Get pin name
    std::string getPinName(int pin_index) const {
        if (pin_index >= 0 && pin_index < static_cast<int>(pins.size())) {
            return pins[pin_index].name;
        }
        return "unknown";
    }
    
    // Get number of pins
    int getPinCount() const {
        return static_cast<int>(pins.size());
    }
    
    // Check if component has all pins connected
    bool isFullyConnected() const {
        for (const auto& pin : pins) {
            if (pin.node_id == -1) return false;
        }
        return true;
    }
    
    // Get unconnected pin count
    int getUnconnectedPinCount() const {
        int count = 0;
        for (const auto& pin : pins) {
            if (pin.node_id == -1) count++;
        }
        return count;
    }
    
    // Legacy compatibility for 2-pin components
    int node1() const { return getNodeForPin(0); }
    int node2() const { return getNodeForPin(1); }
    void setNode1(int node) { setNodeForPin(0, node); }
    void setNode2(int node) { setNodeForPin(1, node); }
};

class Resistor: public CircuitElement{
public:
    double r;
    
    Resistor(const std::string& name, double r) : CircuitElement(name), r(r) {
        // Add two pins for resistor
        pins.emplace_back("pin1", -20, 0);
        pins.emplace_back("pin2", 20, 0);
    }
    
    std::string getType() const override { return "resistor"; }
    std::string getValue() const override { 
        if (r >= 1000000) return std::to_string(static_cast<int>(r/1000000)) + "M";
        if (r >= 1000) return std::to_string(static_cast<int>(r/1000)) + "k";
        return std::to_string(static_cast<int>(r));
    }
    void setValue(const std::string& value) override { 
        r = std::stod(value);
    }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + std::to_string(pins[1].node_id) + " " + std::to_string(r);
    }
};

class Capacitor: public CircuitElement{
public:
    double c;
    
    Capacitor(const std::string& name, double c) : CircuitElement(name), c(c) {
        pins.emplace_back("pin1", -15, 0);
        pins.emplace_back("pin2", 15, 0);
    }
    
    std::string getType() const override { return "capacitor"; }
    std::string getValue() const override { 
        if (c >= 1e-3) return std::to_string(static_cast<int>(c*1000)) + "m";
        if (c >= 1e-6) return std::to_string(static_cast<int>(c*1000000)) + "u";
        if (c >= 1e-9) return std::to_string(static_cast<int>(c*1000000000)) + "n";
        return std::to_string(c) + "F";
    }
    void setValue(const std::string& value) override { 
        c = std::stod(value);
    }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + std::to_string(pins[1].node_id) + " " + std::to_string(c);
    }
};

class VoltageSource: public CircuitElement{
public:
    double v;
    
    VoltageSource(const std::string& name, double v) : CircuitElement(name), v(v) {
        pins.emplace_back("positive", 0, -15);
        pins.emplace_back("negative", 0, 15);
    }
    
    std::string getType() const override { return "vsource"; }
    std::string getValue() const override { 
        return std::to_string(static_cast<int>(v)) + "V";
    }
    void setValue(const std::string& value) override { 
        v = std::stod(value);
    }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + std::to_string(pins[1].node_id) + " " + std::to_string(v);
    }
};

class Ground: public CircuitElement{
public:
    Ground(const std::string& name) : CircuitElement(name) {
        pins.emplace_back("gnd", 0, 0);
        pins[0].node_id = 0;  // Ground is always node 0
    }
    
    std::string getType() const override { return "ground"; }
    std::string getValue() const override { return "GND"; }
    void setValue(const std::string& value) override { }
    
    std::string toSpiceLine() const override {
        return "";  // Ground doesn't generate SPICE line
    }
    
    bool isFullyConnected() const{
        return true;  // Ground is always connected
    }
    
    void setNodeForPin(int pin_index, int node_id) {
        // When something connects to ground, force the other component's node to 0
        if (pin_index == 0) {
            pins[0].node_id = 0;  // Ground pin stays at 0
        }
    }
};

// Example 3-pin component: NPN Transistor
class NPNTransistor: public CircuitElement{
public:
    std::string model;
    
    NPNTransistor(const std::string& name, const std::string& model = "2N2222") 
        : CircuitElement(name), model(model) {
        // BJT has 3 pins: Collector, Base, Emitter
        pins.emplace_back("collector", 0, -15);
        pins.emplace_back("base", -20, 0);
        pins.emplace_back("emitter", 0, 15);
    }
    
    std::string getType() const override { return "npn"; }
    std::string getValue() const override { return model; }
    void setValue(const std::string& value) override { model = value; }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + 
               std::to_string(pins[1].node_id) + " " + std::to_string(pins[2].node_id) + " " + model;
    }
};

// Example 4-pin component: Op-Amp  
class OpAmp: public CircuitElement{
public:
    std::string model;
    
    OpAmp(const std::string& name, const std::string& model = "LM741") 
        : CircuitElement(name), model(model) {
        // Op-amp has 4 main pins: non-inverting input, inverting input, output, and power (simplified)
        pins.emplace_back("non_inv", -25, -10);  // Pin 1: Non-inverting input
        pins.emplace_back("inv", -25, 10);       // Pin 2: Inverting input  
        pins.emplace_back("output", 25, 0);      // Pin 3: Output
        pins.emplace_back("vcc", 0, -20);        // Pin 4: Power supply
    }
    
    std::string getType() const override { return "opamp"; }
    std::string getValue() const override { return model; }
    void setValue(const std::string& value) override { model = value; }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + 
               std::to_string(pins[1].node_id) + " " + std::to_string(pins[2].node_id) + " " + 
               std::to_string(pins[3].node_id) + " " + model;
    }
};

#endif
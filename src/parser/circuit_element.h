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
protected:
    int rotation = 0;
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
    void rotate90() { 
        rotation = (rotation + 90) % 360; 
        updatePinPositions();
    }
    int getRotation() const { return rotation; }
    void setRotation(int rot) { 
        rotation = rot % 360; 
        updatePinPositions();
    }

    // Update pin positions based on rotation
    virtual void updatePinPositions() {
        // Base implementation - derived classes can override for custom behavior
        if (rotation == 0) return;  // No rotation needed
        
        for (auto& pin : pins) {
            float original_x = pin.rel_x;
            float original_y = pin.rel_y;
            
            switch (rotation) {
                case 90:
                    pin.rel_x = -original_y;
                    pin.rel_y = original_x;
                    break;
                case 180:
                    pin.rel_x = -original_x;
                    pin.rel_y = -original_y;
                    break;
                case 270:
                    pin.rel_x = original_y;
                    pin.rel_y = -original_x;
                    break;
            }
        }
    }
    
    // Check if point is inside component bounds (for selection)
    virtual bool isPointInside(float px, float py, float tolerance = 25.0f) const {
        float dx = x - px;
        float dy = y - py;
        return (dx*dx + dy*dy) <= tolerance*tolerance;
    }
    
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
class Inductor: public CircuitElement{
public:
    double l;  // Inductance in Henries
    
    Inductor(const std::string& name, double l) : CircuitElement(name), l(l) {
        pins.emplace_back("pin1", -15, 0);
        pins.emplace_back("pin2", 15, 0);
    }
    
    std::string getType() const override { return "inductor"; }
    std::string getValue() const override { 
        if (l >= 1.0) return std::to_string(static_cast<int>(l)) + "H";
        if (l >= 1e-3) return std::to_string(static_cast<int>(l*1000)) + "mH";
        if (l >= 1e-6) return std::to_string(static_cast<int>(l*1000000)) + "uH";
        return std::to_string(l) + "H";
    }
    void setValue(const std::string& value) override { 
        l = std::stod(value);
    }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + std::to_string(pins[1].node_id) + " " + std::to_string(l);
    }
};

class Diode: public CircuitElement{
public:
    std::string model;
    double Is = 1e-14;     // Saturation current
    double n = 1.0;        // Ideality factor
    double Vt = 0.026;     // Thermal voltage at room temp
    
    Diode(const std::string& name, const std::string& model = "D1N4148") 
        : CircuitElement(name), model(model) {
        pins.emplace_back("anode", -10, 0);
        pins.emplace_back("cathode", 10, 0);
    }
    
    std::string getType() const override { return "diode"; }
    std::string getValue() const override { return model; }
    void setValue(const std::string& value) override { model = value; }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + std::to_string(pins[1].node_id) + " " + model;
    }
    
    // Diode I-V relationship: I = Is * (exp(V/(n*Vt)) - 1)
    double getCurrent(double voltage) const {
        if (voltage < -5.0 * n * Vt) {
            return -Is;  // Reverse saturation
        }
        return Is * (std::exp(voltage / (n * Vt)) - 1.0);
    }
    
    // Conductance = dI/dV
    double getConductance(double voltage) const {
        if (voltage < -5.0 * n * Vt) {
            return 1e-12;  // Very small conductance in reverse
        }
        return (Is / (n * Vt)) * std::exp(voltage / (n * Vt));
    }
};

class NMOSFET: public CircuitElement{
public:
    std::string model;
    double W = 10e-6;      // Width in meters
    double L = 1e-6;       // Length in meters  
    double Vth = 0.7;      // Threshold voltage
    double Kn = 100e-6;    // Process parameter (A/V²)
    double lambda = 0.01;  // Channel length modulation
    
    NMOSFET(const std::string& name, const std::string& model = "NMOS") 
        : CircuitElement(name), model(model) {
        pins.emplace_back("drain", 0, -15);
        pins.emplace_back("gate", -20, 0);
        pins.emplace_back("source", 0, 15);
        pins.emplace_back("bulk", 20, 0);  // Body/substrate
    }
    
    std::string getType() const override { return "nmosfet"; }
    std::string getValue() const override { 
        return model + " W=" + std::to_string(W*1e6) + "u L=" + std::to_string(L*1e6) + "u"; 
    }
    void setValue(const std::string& value) override { model = value; }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + 
               std::to_string(pins[1].node_id) + " " + std::to_string(pins[2].node_id) + " " + 
               std::to_string(pins[3].node_id) + " " + model;
    }
    
    // MOSFET operating regions
    enum class Region { CUTOFF, TRIODE, SATURATION };
    
    Region getOperatingRegion(double Vgs, double Vds) const {
        if (Vgs < Vth) return Region::CUTOFF;
        if (Vds < (Vgs - Vth)) return Region::TRIODE;
        return Region::SATURATION;
    }
    
    // Drain current calculation
    double getDrainCurrent(double Vgs, double Vds) const {
        if (Vgs < Vth) return 0.0;  // Cutoff
        
        double beta = Kn * (W / L);
        
        if (Vds < (Vgs - Vth)) {
            // Triode/Linear region
            return beta * ((Vgs - Vth) * Vds - 0.5 * Vds * Vds) * (1 + lambda * Vds);
        } else {
            // Saturation region
            return 0.5 * beta * (Vgs - Vth) * (Vgs - Vth) * (1 + lambda * Vds);
        }
    }
    
    // Transconductance gm = dId/dVgs
    double getTransconductance(double Vgs, double Vds) const {
        if (Vgs < Vth) return 0.0;
        
        double beta = Kn * (W / L);
        
        if (Vds < (Vgs - Vth)) {
            // Triode region
            return beta * Vds * (1 + lambda * Vds);
        } else {
            // Saturation region
            return beta * (Vgs - Vth) * (1 + lambda * Vds);
        }
    }
    
    // Output conductance gds = dId/dVds
    double getOutputConductance(double Vgs, double Vds) const {
        if (Vgs < Vth) return 1e-12;  // Very small leakage
        
        double beta = Kn * (W / L);
        
        if (Vds < (Vgs - Vth)) {
            // Triode region
            return beta * (Vgs - Vth - Vds) * (1 + lambda * Vds) + 
                   beta * ((Vgs - Vth) * Vds - 0.5 * Vds * Vds) * lambda;
        } else {
            // Saturation region
            return 0.5 * beta * lambda * (Vgs - Vth) * (Vgs - Vth);
        }
    }
};

class PMOSFET: public CircuitElement{
public:
    std::string model;
    double W = 20e-6;      // Width (typically larger for PMOS)
    double L = 1e-6;       // Length
    double Vth = -0.7;     // Threshold voltage (negative for PMOS)
    double Kp = 50e-6;     // Process parameter (typically smaller than NMOS)
    double lambda = 0.02;  // Channel length modulation
    
    PMOSFET(const std::string& name, const std::string& model = "PMOS") 
        : CircuitElement(name), model(model) {
        pins.emplace_back("drain", 0, -15);
        pins.emplace_back("gate", -20, 0);
        pins.emplace_back("source", 0, 15);
        pins.emplace_back("bulk", 20, 0);
    }
    
    std::string getType() const override { return "pmosfet"; }
    std::string getValue() const override { 
        return model + " W=" + std::to_string(W*1e6) + "u L=" + std::to_string(L*1e6) + "u"; 
    }
    void setValue(const std::string& value) override { model = value; }
    
    std::string toSpiceLine() const override {
        return name + " " + std::to_string(pins[0].node_id) + " " + 
               std::to_string(pins[1].node_id) + " " + std::to_string(pins[2].node_id) + " " + 
               std::to_string(pins[3].node_id) + " " + model;
    }
    
    // For PMOS, all voltages are typically referenced with opposite polarity
    double getDrainCurrent(double Vsg, double Vsd) const {  // Note: source-gate, source-drain
        if (Vsg < -Vth) return 0.0;  // Cutoff (Vth is negative)
        
        double beta = Kp * (W / L);
        
        if (Vsd < (Vsg + Vth)) {  // Vth is negative, so this is (Vsg - |Vth|)
            // Triode region
            return beta * ((Vsg + Vth) * Vsd - 0.5 * Vsd * Vsd) * (1 + lambda * Vsd);
        } else {
            // Saturation region
            return 0.5 * beta * (Vsg + Vth) * (Vsg + Vth) * (1 + lambda * Vsd);
        }
    }
};

#endif
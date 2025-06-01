#ifndef TRANSIENT_ANALYSIS_H
#define TRANSIENT_ANALYSIS_H

#include <vector>
#include <map>
#include <memory>
#include "parser/circuit_element.h"

struct TransientSettings {
    double stepTime;    // Time step (e.g., 1e-9 for 1ns)
    double stopTime;    // Total simulation time
    double startTime = 0.0;
    
    TransientSettings(double step, double stop, double start = 0.0) 
        : stepTime(step), stopTime(stop), startTime(start) {}
};

struct TimePoint {
    double time;
    std::vector<double> nodeVoltages;
    std::map<std::string, double> branchCurrents;
};

class TransientAnalysis {
private:
    std::vector<std::unique_ptr<CircuitElement>>& elements;
    int numNodes;
    TransientSettings settings;
    
    // Current time step matrices
    std::vector<std::vector<double>> G;  // Conductance matrix
    std::vector<double> b;               // Right-hand side
    std::vector<double> x;               // Current solution
    std::vector<double> x_prev;          // Previous time step solution
    
    int matrixSize;
    std::map<std::string, int> voltageSourceIndex;
    
    // Results storage
    std::vector<TimePoint> results;
    
    // Integration method
    enum class IntegrationMethod {
        BACKWARD_EULER,
        TRAPEZOIDAL
    };
    IntegrationMethod method = IntegrationMethod::BACKWARD_EULER;
    
public:
    TransientAnalysis(std::vector<std::unique_ptr<CircuitElement>>& elems, 
                     int nodes, const TransientSettings& settings);
    
    void solve();
    void printResults();
    void exportResults(const std::string& filename);
    
    // Getters for specific time points
    std::vector<double> getNodeVoltageHistory(int node) const;
    std::vector<double> getTimePoints() const;
    
private:
    void initializeIC();  // Initial conditions
    void buildMNAMatrix(double currentTime);
    void timeStep();
    bool gaussianElimination();
    
    // Device stamps for transient analysis
    void addResistor(const Resistor* resistor);
    void addVoltageSource(const VoltageSource* vsource, double currentTime);
    void addCapacitor(const Capacitor* capacitor);
    
    void addMatrixEntry(int row, int col, double value);
    void saveTimePoint(double currentTime);
    
    // Integration methods
    double getCapacitorCurrent(const Capacitor* cap, double v_current, double v_previous);
    double getCapacitorEquivalentConductance(const Capacitor* cap);
    double getCapacitorEquivalentCurrentSource(const Capacitor* cap, double v_previous);
};

#endif
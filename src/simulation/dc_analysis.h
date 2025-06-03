#ifndef DC_ANALYSIS_H
#define DC_ANALYSIS_H

#include <vector>
#include <map>
#include <memory>
#include "parser/circuit_element.h"
#include "simulation/dc_analysis.h"


class DCAnalysis {
private:
    std::vector<std::unique_ptr<CircuitElement>>& elements;
    int numNodes;
    int matrixSize;
    
    // Use standard C++ containers instead of Eigen
    std::vector<std::vector<double>> G;  // Conductance matrix
    std::vector<double> b;               // Right-hand side vector
    std::vector<double> x;               // Solution vector
    
    std::map<std::string, int> voltageSourceIndex;
    
public:
    DCAnalysis(std::vector<std::unique_ptr<CircuitElement>>& elems, int nodes);
    
    void buildMNAMatrix();
    void solve();
    void printResults();
    
    double getNodeVoltage(int node) const;
    double getVoltagSourceCurrent(const std::string& vsourceName) const;
    
private:
    void addResistor(const Resistor* resistor);
    void addVoltageSource(const VoltageSource* vsource);
    void addCapacitor(const Capacitor* capacitor);
    void addInductor(const Inductor* inductor);
    void addDiode(const Diode* diode);
    void addMOSFET(const NMOSFET* mosfet);
    
    void addMatrixEntry(int row, int col, double value);
    bool gaussianElimination();
    void printMatrix() const;
};

#endif
#include "dc_analysis.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

DCAnalysis::DCAnalysis(std::vector<std::unique_ptr<CircuitElement>>& elems, int nodes) 
    : elements(elems), numNodes(nodes) {
    
    // Count voltage sources
    int numVoltageSources = 0;
    voltageSourceIndex.clear();
    for (const auto& element : elements) {
        if (element->name.empty()) continue;
        char type = std::tolower(element->name[0]);
        if (type == 'v') {
            voltageSourceIndex[element->name] = (numNodes - 1) + numVoltageSources;
            numVoltageSources++;
        }
    }
    
    matrixSize = (numNodes - 1) + numVoltageSources;
    
    // Initialize matrices
    G.assign(matrixSize, std::vector<double>(matrixSize, 0.0));
    b.assign(matrixSize, 0.0);
    x.assign(matrixSize, 0.0);
    
    std::cout << "DC Analysis initialized: " << numNodes << " nodes, " 
              << numVoltageSources << " voltage sources" << std::endl;
    std::cout << "Matrix size: " << matrixSize << "x" << matrixSize << std::endl;
}

void DCAnalysis::addMatrixEntry(int row, int col, double value) {
    if (row >= 0 && row < matrixSize && col >= 0 && col < matrixSize) {
        G[row][col] += value;
    }
}

void DCAnalysis::addResistor(const Resistor* resistor) {
    int n1 = resistor->node1;
    int n2 = resistor->node2;
    double conductance = 1.0 / resistor->r;
    
    std::cout << "Adding resistor " << resistor->name << ": nodes " << n1 << "-" << n2 
              << ", G=" << conductance << " S" << std::endl;
    
    // Handle ground connections properly
    // Ground (node 0) doesn't appear in the matrix
    
    if (n1 == 0 && n2 == 0) {
        std::cout << "  Resistor between ground nodes - ignored" << std::endl;
        return;
    }
    
    if (n1 == 0) {
        // Resistor from ground to n2: only affects n2's diagonal
        int node_idx = n2 - 1;  // Convert to 0-based
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            std::cout << "  Ground to node " << n2 << ": Adding G[" << node_idx << "][" << node_idx << "] += " << conductance << std::endl;
            addMatrixEntry(node_idx, node_idx, conductance);
        }
        return;
    }
    
    if (n2 == 0) {
        // Resistor from n1 to ground: only affects n1's diagonal
        int node_idx = n1 - 1;  // Convert to 0-based
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            std::cout << "  Node " << n1 << " to ground: Adding G[" << node_idx << "][" << node_idx << "] += " << conductance << std::endl;
            addMatrixEntry(node_idx, node_idx, conductance);
        }
        return;
    }
    
    // Both nodes are non-ground: standard resistor stamp
    int idx1 = n1 - 1;  // Convert to 0-based indexing
    int idx2 = n2 - 1;
    
    std::cout << "  Between nodes " << n1 << "-" << n2 << " (indices " << idx1 << "-" << idx2 << ")" << std::endl;
    
    if (idx1 >= 0 && idx1 < (numNodes-1)) {
        std::cout << "  Adding G[" << idx1 << "][" << idx1 << "] += " << conductance << std::endl;
        addMatrixEntry(idx1, idx1, conductance);
        
        if (idx2 >= 0 && idx2 < (numNodes-1)) {
            std::cout << "  Adding G[" << idx1 << "][" << idx2 << "] += " << -conductance << std::endl;
            addMatrixEntry(idx1, idx2, -conductance);
        }
    }
    
    if (idx2 >= 0 && idx2 < (numNodes-1)) {
        std::cout << "  Adding G[" << idx2 << "][" << idx2 << "] += " << conductance << std::endl;
        addMatrixEntry(idx2, idx2, conductance);
        
        if (idx1 >= 0 && idx1 < (numNodes-1)) {
            std::cout << "  Adding G[" << idx2 << "][" << idx1 << "] += " << -conductance << std::endl;
            addMatrixEntry(idx2, idx1, -conductance);
        }
    }
    
    std::cout << "  Resistor stamp completed." << std::endl;
}

void DCAnalysis::addVoltageSource(const VoltageSource* vsource) {
    int n1 = vsource->node1;
    int n2 = vsource->node2;
    double voltage = vsource->v;
    
    auto it = voltageSourceIndex.find(vsource->name);
    if (it == voltageSourceIndex.end()) {
        std::cerr << "Error: Voltage source " << vsource->name << " not found in index map" << std::endl;
        return;
    }
    int vsIndex = it->second;
    
    std::cout << "Adding voltage source " << vsource->name << ": nodes " << n1 << "-" << n2 
              << ", V=" << voltage << " V, index=" << vsIndex << std::endl;
    
    // For voltage source from n1 to n2, the voltage constraint is: V(n1) - V(n2) = voltage
    // We only stamp non-ground nodes (ground = node 0)
    
    if (n1 != 0) {  // n1 is not ground
        int node_idx = n1 - 1;  // Convert to 0-based indexing
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            std::cout << "  Adding G[" << node_idx << "][" << vsIndex << "] = +1.0" << std::endl;
            std::cout << "  Adding G[" << vsIndex << "][" << node_idx << "] = +1.0" << std::endl;
            addMatrixEntry(node_idx, vsIndex, 1.0);   // Current flows out of positive node
            addMatrixEntry(vsIndex, node_idx, 1.0);   // Voltage constraint equation
        }
    }
    
    if (n2 != 0) {  // n2 is not ground
        int node_idx = n2 - 1;  // Convert to 0-based indexing
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            std::cout << "  Adding G[" << node_idx << "][" << vsIndex << "] = -1.0" << std::endl;
            std::cout << "  Adding G[" << vsIndex << "][" << node_idx << "] = -1.0" << std::endl;
            addMatrixEntry(node_idx, vsIndex, -1.0);  // Current flows into negative node
            addMatrixEntry(vsIndex, node_idx, -1.0);  // Voltage constraint equation
        }
    }
    
    // Set the voltage constraint: V(n1) - V(n2) = voltage
    if (vsIndex >= 0 && vsIndex < matrixSize) {
        std::cout << "  Setting b[" << vsIndex << "] = " << voltage << std::endl;
        b[vsIndex] = voltage;
    }
    
    std::cout << "  Voltage source stamp completed." << std::endl;
}

void DCAnalysis::addCapacitor(const Capacitor* capacitor) {
    // For DC analysis, capacitors are open circuits
    std::cout << "Capacitor " << capacitor->name << " ignored in DC analysis (open circuit)" << std::endl;
}

void DCAnalysis::buildMNAMatrix() {
    std::cout << "Building MNA matrix..." << std::endl;
    
    // Clear matrix
    for (int i = 0; i < matrixSize; i++) {
        for (int j = 0; j < matrixSize; j++) {
            G[i][j] = 0.0;
        }
        b[i] = 0.0;
    }
    
    // Add all circuit elements
    for (const auto& element : elements) {
        if (element->name.empty()) continue;
        
        char type = std::tolower(element->name[0]);
        
        switch(type) {
            case 'r': {
                const Resistor* resistor = static_cast<const Resistor*>(element.get());
                addResistor(resistor);
                break;
            }
            case 'v': {
                const VoltageSource* vsource = static_cast<const VoltageSource*>(element.get());
                addVoltageSource(vsource);
                break;
            }
            case 'c': {
                const Capacitor* capacitor = static_cast<const Capacitor*>(element.get());
                addCapacitor(capacitor);
                break;
            }
            default:
                std::cout << "Unknown element type: " << element->name << std::endl;
        }
    }
    
    std::cout << "MNA matrix built successfully." << std::endl;
}

bool DCAnalysis::gaussianElimination() {
    const double EPSILON = 1e-12;
    
    std::cout << "Solving system using Gaussian elimination..." << std::endl;
    
    // Forward elimination
    for (int i = 0; i < matrixSize; i++) {
        // Find pivot
        int maxRow = i;
        for (int k = i + 1; k < matrixSize; k++) {
            if (std::abs(G[k][i]) > std::abs(G[maxRow][i])) {
                maxRow = k;
            }
        }
        
        // Swap rows
        if (maxRow != i) {
            std::swap(G[i], G[maxRow]);
            std::swap(b[i], b[maxRow]);
        }
        
        // Check for singular matrix
        if (std::abs(G[i][i]) < EPSILON) {
            std::cerr << "Singular matrix detected at row " << i 
                      << " (pivot = " << G[i][i] << ")" << std::endl;
            return false;
        }
        
        // Eliminate
        for (int k = i + 1; k < matrixSize; k++) {
            double factor = G[k][i] / G[i][i];
            b[k] -= factor * b[i];
            for (int j = i; j < matrixSize; j++) {
                G[k][j] -= factor * G[i][j];
            }
        }
    }
    
    // Back substitution
    for (int i = matrixSize - 1; i >= 0; i--) {
        x[i] = b[i];
        for (int j = i + 1; j < matrixSize; j++) {
            x[i] -= G[i][j] * x[j];
        }
        x[i] /= G[i][i];
    }
    
    return true;
}

void DCAnalysis::solve() {
    buildMNAMatrix();
    
    if (matrixSize == 0) {
        std::cout << "No equations to solve!" << std::endl;
        return;
    }
    
    printMatrix();  // Debug output
    
    if (gaussianElimination()) {
        std::cout << "DC analysis completed successfully!" << std::endl;
        printResults();
    } else {
        std::cerr << "DC analysis failed - singular matrix" << std::endl;
    }
}

double DCAnalysis::getNodeVoltage(int node) const {
    if (node == 0) return 0.0;  // Ground
    if (node > 0 && node < numNodes && (node-1) < static_cast<int>(x.size())) {
        return x[node - 1];
    }
    return 0.0;
}

double DCAnalysis::getVoltagSourceCurrent(const std::string& vsourceName) const {
    auto it = voltageSourceIndex.find(vsourceName);
    if (it != voltageSourceIndex.end() && it->second < static_cast<int>(x.size())) {
        return x[it->second];
    }
    return 0.0;
}

void DCAnalysis::printResults() {
    std::cout << "\n=== DC Analysis Results ===" << std::endl;
    
    // Print node voltages
    std::cout << "Node Voltages:" << std::endl;
    std::cout << "Node 0 (ground): 0.000 V" << std::endl;
    for (int i = 1; i < numNodes; i++) {
        std::cout << "Node " << i << ": " << std::fixed << std::setprecision(3) 
                  << getNodeVoltage(i) << " V" << std::endl;
    }
    
    // Print voltage source currents
    if (!voltageSourceIndex.empty()) {
        std::cout << "\nVoltage Source Currents:" << std::endl;
        for (const auto& vs : voltageSourceIndex) {
            std::cout << vs.first << ": " << std::fixed << std::setprecision(6) 
                      << getVoltagSourceCurrent(vs.first) << " A" << std::endl;
        }
    }
    std::cout << "=========================" << std::endl;
}

void DCAnalysis::printMatrix() const {
    std::cout << "\nMNA Matrix (G):" << std::endl;
    for (int i = 0; i < matrixSize; i++) {
        for (int j = 0; j < matrixSize; j++) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(3) << G[i][j] << " ";
        }
        std::cout << "| " << std::setw(8) << std::fixed << std::setprecision(3) << b[i] << std::endl;
    }
    std::cout << std::endl;
}
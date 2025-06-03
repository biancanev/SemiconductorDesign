#include "transient_analysis.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <algorithm>

TransientAnalysis::TransientAnalysis(std::vector<std::unique_ptr<CircuitElement>>& elems, 
                                   int nodes, const TransientSettings& settings)
    : elements(elems), numNodes(nodes), settings(settings) {
    
    // Count voltage sources for matrix sizing
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
    x_prev.assign(matrixSize, 0.0);
    
    std::cout << "Transient Analysis initialized:" << std::endl;
    std::cout << "  Time: " << settings.startTime << "s to " << settings.stopTime 
              << "s, step: " << settings.stepTime << "s" << std::endl;
    std::cout << "  Matrix size: " << matrixSize << "x" << matrixSize << std::endl;
    std::cout << "  Integration method: Backward Euler" << std::endl;
}

void TransientAnalysis::solve() {
    std::cout << "\n=== Starting Transient Analysis ===" << std::endl;
    
    // Initialize with DC operating point
    initializeIC();
    
    double currentTime = settings.startTime;
    int timeStep = 0;
    
    // Save initial conditions
    saveTimePoint(currentTime);
    
    // Time stepping loop
    while (currentTime < settings.stopTime) {
        currentTime += settings.stepTime;
        timeStep++;
        
        if (timeStep % 100 == 0 || timeStep < 10) {
            std::cout << "Time step " << timeStep << ": t = " 
                      << std::scientific << std::setprecision(3) << currentTime << "s" << std::endl;
        }
        
        // Build matrix for current time step
        buildMNAMatrix(currentTime);
        
        // Solve linear system
        if (!gaussianElimination()) {
            std::cerr << "Transient analysis failed at time " << currentTime << std::endl;
            break;
        }
        
        // Save results and prepare for next time step
        saveTimePoint(currentTime);
        this->timeStep();
    }
    
    std::cout << "Transient analysis completed! " << results.size() << " time points saved." << std::endl;
    printResults();
}

void TransientAnalysis::initializeIC() {
    std::cout << "Initializing with DC operating point..." << std::endl;
    
    // For initial conditions, capacitors act as open circuits
    // This gives us the DC operating point
    
    // Clear matrices
    for (int i = 0; i < matrixSize; i++) {
        for (int j = 0; j < matrixSize; j++) {
            G[i][j] = 0.0;
        }
        b[i] = 0.0;
    }
    
    // Add resistors and voltage sources (capacitors ignored for DC)
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
                addVoltageSource(vsource, 0.0);  // DC value
                break;
            }
            // Capacitors ignored for initial DC solution
        }
    }
    
    // Solve for initial conditions
    if (gaussianElimination()) {
        x_prev = x;  // Store as previous solution
        std::cout << "Initial conditions established." << std::endl;
    } else {
        std::cerr << "Failed to establish initial conditions!" << std::endl;
    }
}

void TransientAnalysis::buildMNAMatrix(double currentTime) {
    // Clear matrices
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
            case 'c': {
                const Capacitor* capacitor = static_cast<const Capacitor*>(element.get());
                addCapacitor(capacitor);
                break;
            }
            case 'l': {
                const Inductor* inductor = static_cast<const Inductor*>(element.get());
                addInductor(inductor);
                break;
            }
            case 'v': {
                const VoltageSource* vsource = static_cast<const VoltageSource*>(element.get());
                addVoltageSource(vsource, currentTime);
                break;
            }
            case 'd': {
                // Diodes in transient analysis - linearized around operating point
                const Diode* diode = static_cast<const Diode*>(element.get());
                addDiodeTransient(diode);
                break;
            }
            case 'm': {
                // MOSFET handling
                if (const NMOSFET* nmos = dynamic_cast<const NMOSFET*>(element.get())) {
                    addMOSFETTransient(nmos);
                } else if (const PMOSFET* pmos = dynamic_cast<const PMOSFET*>(element.get())) {
                    addPMOSFETTransient(pmos);
                }
                break;
            }
        }
    }
}

void TransientAnalysis::addResistor(const Resistor* resistor) {
    int n1 = resistor->pins[0].node_id;;
    int n2 = resistor->pins[1].node_id;;
    double conductance = 1.0 / resistor->r;
    
    // Same stamp as DC analysis
    if (n1 == 0 && n2 == 0) return;
    
    if (n1 == 0) {
        int node_idx = n2 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, conductance);
        }
        return;
    }
    
    if (n2 == 0) {
        int node_idx = n1 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, conductance);
        }
        return;
    }
    
    // Both nodes non-ground
    int idx1 = n1 - 1;
    int idx2 = n2 - 1;
    
    if (idx1 >= 0 && idx1 < (numNodes-1)) {
        addMatrixEntry(idx1, idx1, conductance);
        if (idx2 >= 0 && idx2 < (numNodes-1)) {
            addMatrixEntry(idx1, idx2, -conductance);
        }
    }
    
    if (idx2 >= 0 && idx2 < (numNodes-1)) {
        addMatrixEntry(idx2, idx2, conductance);
        if (idx1 >= 0 && idx1 < (numNodes-1)) {
            addMatrixEntry(idx2, idx1, -conductance);
        }
    }
}

void TransientAnalysis::addVoltageSource(const VoltageSource* vsource, double currentTime) {
    int n1 = vsource->pins[0].node_id;;
    int n2 = vsource->pins[1].node_id;;
    double voltage = vsource->v;  // For now, assume DC. Later add time-dependent sources
    
    auto it = voltageSourceIndex.find(vsource->name);
    if (it == voltageSourceIndex.end()) return;
    int vsIndex = it->second;
    
    // Same stamp as DC analysis
    if (n1 != 0) {
        int node_idx = n1 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, vsIndex, 1.0);
            addMatrixEntry(vsIndex, node_idx, 1.0);
        }
    }
    
    if (n2 != 0) {
        int node_idx = n2 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, vsIndex, -1.0);
            addMatrixEntry(vsIndex, node_idx, -1.0);
        }
    }
    
    if (vsIndex >= 0 && vsIndex < matrixSize) {
        b[vsIndex] = voltage;
    }
}

void TransientAnalysis::addCapacitor(const Capacitor* capacitor) {
    int n1 = capacitor->pins[0].node_id;;
    int n2 = capacitor->pins[1].node_id;;
    
    // Backward Euler: C * dv/dt ≈ C * (v_current - v_previous) / dt
    // This creates an equivalent conductance: Geq = C/dt
    // And an equivalent current source: Ieq = Geq * v_previous
    
    double equiv_conductance = getCapacitorEquivalentConductance(capacitor);
    
    // Get previous voltages across capacitor
    double v1_prev = (n1 == 0) ? 0.0 : ((n1-1 < x_prev.size()) ? x_prev[n1-1] : 0.0);
    double v2_prev = (n2 == 0) ? 0.0 : ((n2-1 < x_prev.size()) ? x_prev[n2-1] : 0.0);
    double v_cap_prev = v1_prev - v2_prev;
    
    double equiv_current = getCapacitorEquivalentCurrentSource(capacitor, v_cap_prev);
    
    // Stamp equivalent conductance (like a resistor)
    if (n1 == 0 && n2 == 0) return;
    
    if (n1 == 0) {
        int node_idx = n2 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, equiv_conductance);
            b[node_idx] += equiv_current;  // Current source
        }
        return;
    }
    
    if (n2 == 0) {
        int node_idx = n1 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, equiv_conductance);
            b[node_idx] -= equiv_current;  // Current source (opposite direction)
        }
        return;
    }
    
    // Both nodes non-ground
    int idx1 = n1 - 1;
    int idx2 = n2 - 1;
    
    if (idx1 >= 0 && idx1 < (numNodes-1)) {
        addMatrixEntry(idx1, idx1, equiv_conductance);
        b[idx1] -= equiv_current;
        
        if (idx2 >= 0 && idx2 < (numNodes-1)) {
            addMatrixEntry(idx1, idx2, -equiv_conductance);
        }
    }
    
    if (idx2 >= 0 && idx2 < (numNodes-1)) {
        addMatrixEntry(idx2, idx2, equiv_conductance);
        b[idx2] += equiv_current;
        
        if (idx1 >= 0 && idx1 < (numNodes-1)) {
            addMatrixEntry(idx2, idx1, -equiv_conductance);
        }
    }
}

double TransientAnalysis::getCapacitorEquivalentConductance(const Capacitor* cap) {
    // Backward Euler: Geq = C / dt
    return cap->c / settings.stepTime;
}

double TransientAnalysis::getCapacitorEquivalentCurrentSource(const Capacitor* cap, double v_previous) {
    // Backward Euler: Ieq = (C / dt) * v_previous
    return getCapacitorEquivalentConductance(cap) * v_previous;
}

void TransientAnalysis::addInductor(const Inductor* inductor) {
    int n1 = inductor->pins[0].node_id;
    int n2 = inductor->pins[1].node_id;
    
    // Backward Euler: L * di/dt ≈ L * (i_current - i_previous) / dt
    // This creates voltage equation: V = L * (i_current - i_previous) / dt
    // Rearranging: i_current = (V * dt / L) + i_previous
    // This is equivalent to a resistor with R = dt/L in parallel with current source
    
    double equiv_resistance = settings.stepTime / inductor->l;
    double equiv_conductance = 1.0 / equiv_resistance;
    
    // Get previous current through inductor (stored separately)
    double i_prev = getInductorPreviousCurrent(inductor);
    double equiv_current = i_prev;  // Current source to maintain continuity
    
    std::cout << "Inductor " << inductor->name << ": Geq=" << equiv_conductance 
              << " S, Ieq=" << equiv_current << " A" << std::endl;
    
    // Stamp like a resistor with current source
    if (n1 == 0 && n2 == 0) return;
    
    if (n1 == 0) {
        int node_idx = n2 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, equiv_conductance);
            b[node_idx] -= equiv_current;
        }
        return;
    }
    
    if (n2 == 0) {
        int node_idx = n1 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, equiv_conductance);
            b[node_idx] += equiv_current;
        }
        return;
    }
    
    // Both nodes non-ground
    int idx1 = n1 - 1;
    int idx2 = n2 - 1;
    
    if (idx1 >= 0 && idx1 < (numNodes-1)) {
        addMatrixEntry(idx1, idx1, equiv_conductance);
        b[idx1] += equiv_current;
        
        if (idx2 >= 0 && idx2 < (numNodes-1)) {
            addMatrixEntry(idx1, idx2, -equiv_conductance);
        }
    }
    
    if (idx2 >= 0 && idx2 < (numNodes-1)) {
        addMatrixEntry(idx2, idx2, equiv_conductance);
        b[idx2] -= equiv_current;
        
        if (idx1 >= 0 && idx1 < (numNodes-1)) {
            addMatrixEntry(idx2, idx1, -equiv_conductance);
        }
    }
}

void TransientAnalysis::addDiodeTransient(const Diode* diode) {
    int n1 = diode->pins[0].node_id;  // anode
    int n2 = diode->pins[1].node_id;  // cathode
    
    // Get previous voltage across diode for linearization
    double v1_prev = (n1 == 0) ? 0.0 : ((n1-1 < x_prev.size()) ? x_prev[n1-1] : 0.0);
    double v2_prev = (n2 == 0) ? 0.0 : ((n2-1 < x_prev.size()) ? x_prev[n2-1] : 0.0);
    double v_diode_prev = v1_prev - v2_prev;
    
    // Linearize around previous operating point
    double conductance = diode->getConductance(v_diode_prev);
    double current_prev = diode->getCurrent(v_diode_prev);
    double current_source = current_prev - conductance * v_diode_prev;
    
    std::cout << "Diode " << diode->name << " linearized: G=" << conductance 
              << " S, Is=" << current_source << " A" << std::endl;
    
    // Stamp conductance and current source
    if (n1 == 0 && n2 == 0) return;
    
    if (n1 == 0) {
        int node_idx = n2 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, conductance);
            b[node_idx] += current_source;
        }
        return;
    }
    
    if (n2 == 0) {
        int node_idx = n1 - 1;
        if (node_idx >= 0 && node_idx < (numNodes-1)) {
            addMatrixEntry(node_idx, node_idx, conductance);
            b[node_idx] -= current_source;
        }
        return;
    }
    
    // Both nodes non-ground
    int idx1 = n1 - 1;
    int idx2 = n2 - 1;
    
    if (idx1 >= 0 && idx1 < (numNodes-1)) {
        addMatrixEntry(idx1, idx1, conductance);
        b[idx1] -= current_source;
        
        if (idx2 >= 0 && idx2 < (numNodes-1)) {
            addMatrixEntry(idx1, idx2, -conductance);
        }
    }
    
    if (idx2 >= 0 && idx2 < (numNodes-1)) {
        addMatrixEntry(idx2, idx2, conductance);
        b[idx2] += current_source;
        
        if (idx1 >= 0 && idx1 < (numNodes-1)) {
            addMatrixEntry(idx2, idx1, -conductance);
        }
    }
}

void TransientAnalysis::addMOSFETTransient(const NMOSFET* mosfet) {
    int nd = mosfet->pins[0].node_id;  // drain
    int ng = mosfet->pins[1].node_id;  // gate  
    int ns = mosfet->pins[2].node_id;  // source
    int nb = mosfet->pins[3].node_id;  // bulk
    
    // Get previous voltages for linearization
    double vd_prev = (nd == 0) ? 0.0 : ((nd-1 < x_prev.size()) ? x_prev[nd-1] : 0.0);
    double vg_prev = (ng == 0) ? 0.0 : ((ng-1 < x_prev.size()) ? x_prev[ng-1] : 0.0);
    double vs_prev = (ns == 0) ? 0.0 : ((ns-1 < x_prev.size()) ? x_prev[ns-1] : 0.0);
    
    double vgs_prev = vg_prev - vs_prev;
    double vds_prev = vd_prev - vs_prev;
    
    // Calculate small-signal parameters at operating point
    double gm = mosfet->getTransconductance(vgs_prev, vds_prev);
    double gds = mosfet->getOutputConductance(vgs_prev, vds_prev);
    double id_prev = mosfet->getDrainCurrent(vgs_prev, vds_prev);
    
    // Current source for linearization: Is = Id - gm*Vgs - gds*Vds
    double current_source = id_prev - gm * vgs_prev - gds * vds_prev;
    
    std::cout << "NMOSFET " << mosfet->name << " linearized: gm=" << gm 
              << " S, gds=" << gds << " S, Is=" << current_source << " A" << std::endl;
    
    // Stamp transconductance (gate-source to drain-source current)
    // Id = gm * Vgs, so drain gets +gm*Vg - gm*Vs, source gets opposite
    if (nd != 0 && ng != 0) {
        int idx_d = nd - 1;
        int idx_g = ng - 1;
        
        if (idx_d >= 0 && idx_d < (numNodes-1) && idx_g >= 0 && idx_g < (numNodes-1)) {
            addMatrixEntry(idx_d, idx_g, gm);   // gm * Vg contributes to Id
        }
    }
    
    if (nd != 0 && ns != 0) {
        int idx_d = nd - 1;
        int idx_s = ns - 1;
        
        if (idx_d >= 0 && idx_d < (numNodes-1) && idx_s >= 0 && idx_s < (numNodes-1)) {
            addMatrixEntry(idx_d, idx_s, -gm);  // -gm * Vs contributes to Id
        }
    }
    
    // Stamp output conductance (drain-source)
    if (nd != 0 && ns != 0) {
        int idx_d = nd - 1;
        int idx_s = ns - 1;
        
        if (idx_d >= 0 && idx_d < (numNodes-1)) {
            addMatrixEntry(idx_d, idx_d, gds);
            b[idx_d] -= current_source;  // Drain gets positive current
            
            if (idx_s >= 0 && idx_s < (numNodes-1)) {
                addMatrixEntry(idx_d, idx_s, -gds);
            }
        }
        
        if (idx_s >= 0 && idx_s < (numNodes-1)) {
            addMatrixEntry(idx_s, idx_s, gds);
            b[idx_s] += current_source;  // Source gets negative current
            
            if (idx_d >= 0 && idx_d < (numNodes-1)) {
                addMatrixEntry(idx_s, idx_d, -gds);
            }
        }
    }
    
    // Add transconductance contribution to source node
    if (ns != 0 && ng != 0) {
        int idx_s = ns - 1;
        int idx_g = ng - 1;
        
        if (idx_s >= 0 && idx_s < (numNodes-1) && idx_g >= 0 && idx_g < (numNodes-1)) {
            addMatrixEntry(idx_s, idx_g, -gm);  // Source gets opposite of drain
        }
    }
    
    if (ns != 0) {
        int idx_s = ns - 1;
        if (idx_s >= 0 && idx_s < (numNodes-1)) {
            addMatrixEntry(idx_s, idx_s, gm);   // Source self-contribution
        }
    }
}

void TransientAnalysis::addPMOSFETTransient(const PMOSFET* mosfet) {
    // Similar to NMOS but with opposite polarity considerations
    int nd = mosfet->pins[0].node_id;  // drain
    int ng = mosfet->pins[1].node_id;  // gate  
    int ns = mosfet->pins[2].node_id;  // source
    int nb = mosfet->pins[3].node_id;  // bulk
    
    // For PMOS, we typically work with source-gate and source-drain voltages
    double vd_prev = (nd == 0) ? 0.0 : ((nd-1 < x_prev.size()) ? x_prev[nd-1] : 0.0);
    double vg_prev = (ng == 0) ? 0.0 : ((ng-1 < x_prev.size()) ? x_prev[ng-1] : 0.0);
    double vs_prev = (ns == 0) ? 0.0 : ((ns-1 < x_prev.size()) ? x_prev[ns-1] : 0.0);
    
    double vsg_prev = vs_prev - vg_prev;  // Source-gate for PMOS
    double vsd_prev = vs_prev - vd_prev;  // Source-drain for PMOS
    
    // For simplicity, use similar small-signal model as NMOS
    // In practice, you'd implement the specific PMOS equations
    double gm = 50e-6;  // Simplified transconductance
    double gds = 1e-6;  // Simplified output conductance
    
    std::cout << "PMOSFET " << mosfet->name << " using simplified model" << std::endl;
    
    // Similar stamping as NMOS but consider PMOS polarity
    // Implementation details would follow PMOS device equations
}

double TransientAnalysis::getInductorPreviousCurrent(const Inductor* inductor) {
    // In a complete implementation, you would maintain a separate data structure
    // to track inductor currents at each time step
    // For now, return 0.0 as a placeholder
    
    // TODO: Implement inductor current history tracking
    return 0.0;
}

void TransientAnalysis::addMatrixEntry(int row, int col, double value) {
    if (row >= 0 && row < matrixSize && col >= 0 && col < matrixSize) {
        G[row][col] += value;
    }
}

bool TransientAnalysis::gaussianElimination() {
    // Same Gaussian elimination as DC analysis
    const double EPSILON = 1e-12;
    
    for (int i = 0; i < matrixSize; i++) {
        int maxRow = i;
        for (int k = i + 1; k < matrixSize; k++) {
            if (std::abs(G[k][i]) > std::abs(G[maxRow][i])) {
                maxRow = k;
            }
        }
        
        if (maxRow != i) {
            std::swap(G[i], G[maxRow]);
            std::swap(b[i], b[maxRow]);
        }
        
        if (std::abs(G[i][i]) < EPSILON) {
            return false;
        }
        
        for (int k = i + 1; k < matrixSize; k++) {
            double factor = G[k][i] / G[i][i];
            b[k] -= factor * b[i];
            for (int j = i; j < matrixSize; j++) {
                G[k][j] -= factor * G[i][j];
            }
        }
    }
    
    for (int i = matrixSize - 1; i >= 0; i--) {
        x[i] = b[i];
        for (int j = i + 1; j < matrixSize; j++) {
            x[i] -= G[i][j] * x[j];
        }
        x[i] /= G[i][i];
    }
    
    return true;
}

void TransientAnalysis::timeStep() {
    // Prepare for next time step
    x_prev = x;
}

void TransientAnalysis::saveTimePoint(double currentTime) {
    TimePoint point;
    point.time = currentTime;
    
    // Save node voltages
    point.nodeVoltages.resize(numNodes);
    point.nodeVoltages[0] = 0.0;  // Ground
    
    for (int i = 1; i < numNodes; i++) {
        if ((i-1) < static_cast<int>(x.size())) {
            point.nodeVoltages[i] = x[i-1];
        } else {
            point.nodeVoltages[i] = 0.0;
        }
    }
    
    // Save voltage source currents
    for (const auto& vs : voltageSourceIndex) {
        if (vs.second < static_cast<int>(x.size())) {
            point.branchCurrents[vs.first] = x[vs.second];
        }
    }
    
    results.push_back(point);
}

void TransientAnalysis::printResults() {
    std::cout << "\n=== Transient Analysis Results ===" << std::endl;
    std::cout << "Total time points: " << results.size() << std::endl;
    
    // Print first few and last few points
    int printCount = std::min(5, static_cast<int>(results.size()));
    
    std::cout << "\nFirst " << printCount << " time points:" << std::endl;
    std::cout << std::setw(12) << "Time(s)";
    for (int i = 1; i < numNodes; i++) {
        std::cout << std::setw(12) << ("Node" + std::to_string(i));
    }
    std::cout << std::endl;
    
    for (int i = 0; i < printCount; i++) {
        const auto& point = results[i];
        std::cout << std::scientific << std::setprecision(3) << std::setw(12) << point.time;
        for (int j = 1; j < numNodes; j++) {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12) << point.nodeVoltages[j];
        }
        std::cout << std::endl;
    }
    
    if (results.size() > 10) {
        std::cout << "..." << std::endl;
        std::cout << "Last " << printCount << " time points:" << std::endl;
        
        for (int i = results.size() - printCount; i < static_cast<int>(results.size()); i++) {
            const auto& point = results[i];
            std::cout << std::scientific << std::setprecision(3) << std::setw(12) << point.time;
            for (int j = 1; j < numNodes; j++) {
                std::cout << std::fixed << std::setprecision(6) << std::setw(12) << point.nodeVoltages[j];
            }
            std::cout << std::endl;
        }
    }
}

void TransientAnalysis::exportResults(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open " << filename << " for writing." << std::endl;
        return;
    }
    
    // Write header
    file << "Time";
    for (int i = 1; i < numNodes; i++) {
        file << ",Node" << i;
    }
    file << std::endl;
    
    // Write data
    for (const auto& point : results) {
        file << std::scientific << std::setprecision(6) << point.time;
        for (int i = 1; i < numNodes; i++) {
            file << "," << std::fixed << std::setprecision(6) << point.nodeVoltages[i];
        }
        file << std::endl;
    }
    
    file.close();
    std::cout << "Results exported to " << filename << std::endl;
}

std::vector<double> TransientAnalysis::getNodeVoltageHistory(int node) const {
    std::vector<double> history;
    for (const auto& point : results) {
        if (node < static_cast<int>(point.nodeVoltages.size())) {
            history.push_back(point.nodeVoltages[node]);
        }
    }
    return history;
}

std::vector<double> TransientAnalysis::getTimePoints() const {
    std::vector<double> times;
    for (const auto& point : results) {
        times.push_back(point.time);
    }
    return times;
}
#ifndef CIRCUIT_ELEMENT_H
#define CIRCUIT_ELEMENT_H

#include <vector>
#include <string>

class CircuitElement{
    public:
        std::string name;
        int node1, node2;
        std::vector<std::string> nodes;
        CircuitElement(const std::string& name, int n1, int n2);
        virtual ~CircuitElement() = default;
};

class Resistor: public CircuitElement{
    public:
        double r;
        Resistor(const std::string& name, int n1, int n2, double r);
};

class Capacitor: public CircuitElement{
    public:
        double c;
        Capacitor(const std::string& name, int n1, int n2, double c);
};

class VoltageSource: public CircuitElement{
    public:
        double v;
        VoltageSource(const std::string& name, int n1, int n2, double v);
};

#endif
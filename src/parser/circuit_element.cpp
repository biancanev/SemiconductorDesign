#include "circuit_element.h"

CircuitElement::CircuitElement(const std::string& name, int n1, int n2){
    node1=n1;
    node2=n2;
    this->name=name;
}

Resistor::Resistor(const std::string& name, int n1, int n2, double r) : CircuitElement(name, n1, n2){
    this->r=r;
}

Capacitor::Capacitor(const std::string& name, int n1, int n2, double c) : CircuitElement(name, n1, n2){
    this->c=c;
}

VoltageSource::VoltageSource(const std::string& name, int n1, int n2, double v) : CircuitElement(name, n1, n2){
    this->v=v;
}
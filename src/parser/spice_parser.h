#ifndef SPICE_PARSER_H
#define SPICE_PARSER_H

#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "circuit_element.h"
#include "simulation/dc_analysis.h"
#include "simulation/transient_analysis.h"

class SPICEParser{
    private:
        std::vector<std::unique_ptr<CircuitElement>> elements;
        std::map<std::string, int> nodeMap;
        int numNodes = 0;
        TransientSettings* transientSettings = nullptr;

    public:
        void parseFile(const std::string& filename);
        void parseCommand(const std::vector<std::string>& tokens);
        void parseComponent(const std::vector<std::string>& tokens);
        double parseValue(const std::string& valueStr);
        int getNodeNumber(const std::string& nodeName);
        void parseResistor(const std::vector<std::string>& tokens);
        void parseCapacitor(const std::vector<std::string>& tokens);
        void parseVoltageSource(const std::vector<std::string>& tokens);
        void parseInductor(const std::vector<std::string>& tokens);
        void parseDiode(const std::vector<std::string>& tokens);
        void parseMOSFET(const std::vector<std::string>& tokens);
        void printParsedElements();
        //void parseCurrentSource(const std::vector<std::string>& tokens);
        //void parseMOSFET(const std::vector<std::string>& tokens);

};

class SPICETokenizer{
    private:
        std::vector<std::string> lines;
        size_t current_line;
    public:
        std::vector<std::string> tokenizeLine(const std::string& line);
        bool hasMoreLines();
        std::string getNextLine();
        void loadFile(const std::string& filename);
};

#endif
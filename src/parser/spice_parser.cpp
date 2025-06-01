#include "spice_parser.h"
std::vector<std::string> SPICETokenizer::tokenizeLine(const std::string& line){
    std::vector<std::string> tokens;
    std::string cleanLine = line;

    size_t commentPos = cleanLine.find(';');
    if (commentPos != std::string::npos) {
        cleanLine = cleanLine.substr(0, commentPos);
    }

    if (!cleanLine.empty() && cleanLine[0] == '*') {
        return tokens; // Return empty vector for comment lines
    }

    cleanLine.erase(0, cleanLine.find_first_not_of(" \t\r\n"));
    cleanLine.erase(cleanLine.find_last_not_of(" \t\r\n") + 1);
    
    if (cleanLine.empty()) {
        return tokens;
    }

    std::istringstream iss(cleanLine);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    
    return tokens;
}

bool SPICETokenizer::hasMoreLines() {
    return current_line < lines.size();
}

std::string SPICETokenizer::getNextLine() {
    if (hasMoreLines()) {
        return lines[current_line++];
    }
    return "";
}

void SPICETokenizer::loadFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    lines.clear();
    current_line = 0;
    
    std::string line;
    while (std::getline(file, line)) {
        // Handle line continuation (lines starting with '+')
        if (!line.empty() && line[0] == '+' && !lines.empty()) {
            // Append to previous line (remove the '+')
            lines.back() += " " + line.substr(1);
        } else {
            lines.push_back(line);
        }
    }
    file.close();
}

void SPICEParser::parseFile(const std::string& filename) {
    SPICETokenizer tokenizer;
    tokenizer.loadFile(filename);
    
    // Initialize ground node (node 0)
    nodeMap["0"] = 0;
    nodeMap["gnd"] = 0;
    nodeMap["ground"] = 0;
    numNodes = 1; // Start with 1 because we have ground
    
    while (tokenizer.hasMoreLines()) {
        std::string line = tokenizer.getNextLine();
        auto tokens = tokenizer.tokenizeLine(line);
        
        if (tokens.empty()) continue;
        
        // Check if it's a command (starts with '.')
        if (tokens[0][0] == '.') {
            parseCommand(tokens);
        } else {
            parseComponent(tokens);
        }
    }
    
    std::cout << "Parsed " << elements.size() << " components with " 
              << numNodes << " unique nodes." << std::endl;
}

void SPICEParser::parseCommand(const std::vector<std::string>& tokens) {
    std::string command = tokens[0];
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);
    
    if (command == ".end") {
        std::cout << "End of netlist reached." << std::endl;
    } else if (command == ".tran") {
        if (tokens.size() >= 3) {
            std::cout << "Transient analysis: step=" << tokens[1] 
                        << ", stop=" << tokens[2] << std::endl;
        }else{
            std::cerr << "Invalid .tran command. Usage: .tran <step> <stop> [start]" << std::endl;
            return;
        }

        double stepTime = parseValue(tokens[1]);
        double stopTime = parseValue(tokens[2]);
        double startTime = 0.0;
        
        if (tokens.size() >= 4) {
            startTime = parseValue(tokens[3]);
        }
        
        std::cout << "Transient analysis: step=" << stepTime 
                << "s, stop=" << stopTime << "s, start=" << startTime << "s" << std::endl;
        
        // Store transient settings
        if (transientSettings) {
            delete transientSettings;
        }
        this->transientSettings = new TransientSettings(stepTime, stopTime, startTime);

        if (!transientSettings) {
            std::cout << "No transient analysis specified (.tran command required)" << std::endl;
            return;
        }
        
        std::cout << "\n=== Starting Transient Analysis ===" << std::endl;
        
        if (elements.empty()) {
            std::cout << "No circuit elements found. Cannot run transient analysis." << std::endl;
            return;
        }
        
        TransientAnalysis transientAnalysis(elements, numNodes, *transientSettings);
        transientAnalysis.solve();
        transientAnalysis.exportResults("transient_results.csv");
    } else if (command == ".dc") {
        std::cout << "DC analysis specified" << std::endl;
        if (elements.empty()) {
            std::cout << "No circuit elements found. Cannot run DC analysis." << std::endl;
            return;
        }
        
        DCAnalysis dcAnalysis(elements, numNodes);
        dcAnalysis.solve();

    } else {
        std::cout << "Unknown command: " << command << std::endl;
    }
}

int SPICEParser::getNodeNumber(const std::string& nodeName) {
    // Convert to lowercase for case-insensitive comparison
    std::string lowerNode = nodeName;
    std::transform(lowerNode.begin(), lowerNode.end(), lowerNode.begin(), ::tolower);
    
    auto it = nodeMap.find(lowerNode);
    if (it != nodeMap.end()) {
        return it->second;
    } else {
        // New node, assign next number
        nodeMap[lowerNode] = numNodes;
        return numNodes++;
    }
}

void SPICEParser::parseResistor(const std::vector<std::string>& tokens) {
    if (tokens.size() < 4) {
        std::cerr << "Invalid resistor specification" << std::endl;
        return;
    }
    
    std::string name = tokens[0];
    std::string node1 = tokens[1];
    std::string node2 = tokens[2];
    std::string value = tokens[3];
    
    // Map nodes to numbers
    int n1 = getNodeNumber(node1);
    int n2 = getNodeNumber(node2);
    
    // Parse resistance value
    double resistance = parseValue(value);
    
    std::cout << "Resistor " << name << ": " << n1 << " to " << n2 
                << ", R=" << resistance << " ohms" << std::endl;
    
    auto resistor = std::make_unique<Resistor>(name, n1, n2, resistance);
    elements.push_back(std::move(resistor));
}

void SPICEParser::parseCapacitor(const std::vector<std::string>& tokens) {
    std::string name = tokens[0];
    std::string node1 = tokens[1];
    std::string node2 = tokens[2];
    std::string value = tokens[3];

    int n1 = getNodeNumber(node1);
    int n2 = getNodeNumber(node2);

    double capacitance = parseValue(value);

    std::cout << "Capacitance " << name << ": " << n1 << " to " << n2 
            << ", C=" << capacitance << " farads" << std::endl;

    auto capacitor = std::make_unique<Capacitor>(name, n1, n2, capacitance);
    elements.push_back(std::move(capacitor));
}

void SPICEParser::parseVoltageSource(const std::vector<std::string>& tokens) {
    std::string name = tokens[0];
    std::string node1 = tokens[1];
    std::string node2 = tokens[2];
    std::string value = tokens[3];

    int n1 = getNodeNumber(node1);
    int n2 = getNodeNumber(node2);

    double voltage = parseValue(value);

    std::cout << "Voltage Source " << name << ": " << n1 << " to " << n2 
            << ", V=" << voltage << " volts" << std::endl;
    
    auto vsource = std::make_unique<VoltageSource>(name, n1, n2, voltage);
    elements.push_back(std::move(vsource));
}

void SPICEParser::parseComponent(const std::vector<std::string>& tokens) {
    if (tokens.size() < 3) {
        std::cerr << "Invalid component line (too few tokens)" << std::endl;
        return;
    }
    
    std::string name = tokens[0];
    char componentType = std::tolower(name[0]);
    
    switch (componentType) {
        case 'r': // Resistor
            parseResistor(tokens);
            break;
        case 'c': // Capacitor
            parseCapacitor(tokens);
            break;
        case 'v': // Voltage source
            parseVoltageSource(tokens);
            break;
        case 'i': // Current source
            //parseCurrentSource(tokens);
            break;
        case 'm': // MOSFET
            //parseMOSFET(tokens);
            break;
        default:
            std::cout << "Unknown component type: " << componentType 
                        << " in " << name << std::endl;
    }
}
double SPICEParser::parseValue(const std::string& valueStr) {
    if (valueStr.empty()) return 0.0;
    
    // Handle engineering notation
    std::string str = valueStr;
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    
    double multiplier = 1.0;
    char lastChar = str.back();
    
    switch (lastChar) {
        case 't': multiplier = 1e12; str.pop_back(); break;
        case 'g': multiplier = 1e9;  str.pop_back(); break;
        case 'k': multiplier = 1e3;  str.pop_back(); break;
        case 'm': 
            // Could be 'meg' (1e6) or 'm' (1e-3)
            if (str.length() >= 3 && str.substr(str.length()-3) == "meg") {
                multiplier = 1e6;
                str = str.substr(0, str.length()-3);
            } else {
                multiplier = 1e-3;
                str.pop_back();
            }
            break;
        case 'u': multiplier = 1e-6; str.pop_back(); break;
        case 'n': multiplier = 1e-9; str.pop_back(); break;
        case 'p': multiplier = 1e-12; str.pop_back(); break;
        case 'f': multiplier = 1e-15; str.pop_back(); break;
    }
    
    try {
        double value = std::stod(str);
        return value * multiplier;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing value: " << valueStr << std::endl;
        return 0.0;
    }
}

void SPICEParser::printParsedElements() {
    std::cout << "\n=== Parsed Elements Debug ===" << std::endl;
    std::cout << "Total elements: " << elements.size() << std::endl;
    std::cout << "Number of nodes: " << numNodes << std::endl;
    
    for (size_t i = 0; i < elements.size(); i++) {
        const auto& element = elements[i];
        std::cout << "Element " << i << ": " << element->name 
                  << " (nodes: " << element->node1 << "-" << element->node2 << ")" << std::endl;
        
        // Try to identify the type
        if (!element->name.empty()) {
            char type = std::tolower(element->name[0]);
            std::cout << "  Type: " << type << std::endl;
            
            // Try casting to see what type it really is
            if (auto* r = dynamic_cast<const Resistor*>(element.get())) {
                std::cout << "  -> Resistor: " << r->r << " ohms" << std::endl;
            } else if (auto* v = dynamic_cast<const VoltageSource*>(element.get())) {
                std::cout << "  -> Voltage Source: " << v->v << " V" << std::endl;
            } else if (auto* c = dynamic_cast<const Capacitor*>(element.get())) {
                std::cout << "  -> Capacitor: " << c->c << " F" << std::endl;
            } else {
                std::cout << "  -> Unknown type!" << std::endl;
            }
        }
    }
    std::cout << "=========================" << std::endl;
}
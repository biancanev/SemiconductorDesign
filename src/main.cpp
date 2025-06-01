#include <iostream>
#include "parser\spice_parser.h"

int main(){
    try{
        SPICEParser parser;
        parser.parseFile("rc_tran.cir");
        parser.printParsedElements();
    }catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
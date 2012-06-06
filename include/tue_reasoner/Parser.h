/*
 * Parser.h
 *
 *  Created on: June 6, 2012
 *      Author: sdries
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include <stdio.h>
#include <stdlib.h>

#include "tue_reasoner/Compound.h"
#include "tue_reasoner/Value.h"

#include <problib/pdfs/PMF.h>

class Parser {
public:
	Parser(std::string filename);

	~Parser();

	bool parse(std::map<std::string, Compound*> facts, std::stringstream& error);

protected:

	std::string filename_;

	Compound* parseCompound(const std::string& line, std::string& error);

};

#endif /* PARSER_H_ */

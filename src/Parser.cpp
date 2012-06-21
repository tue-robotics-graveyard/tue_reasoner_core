/*
 * Parser.cpp
 *
 *  Created on: June 6, 2012
 *      Author: sdries
 */


#include "tue_reasoner/Parser.h"

using namespace std;

Parser::Parser(std::string filename) : filename_(filename) {

}

Parser::~Parser() {
}

bool Parser::parse(map<std::string, vector<Compound*> >& facts, stringstream& error) {
	ifstream input(filename_.c_str());

	if (!input.is_open()) {
		error << "Could not open file: " << filename_ << endl;
		return false;
	}

	int line_nr = 0;

	while (!input.eof()) {
		std::string line;
		getline(input, line);

		string line_error = "";
		Compound* C = parseCompound(line, line_error);

		if (line_error != "") {
			error << "    Line " << line_nr << ": " << line_error << endl;
		} else if (C) {
			facts[C->getPredicate()].push_back(C);
		}

		++line_nr;
	}

	return error.str() == "";
}

void Parser::parseList(const string& str, vector<string>& args, string& error) {
	size_t i_arg_start = 0;
	size_t i_arg_end = str.find_first_of(',', i_arg_start);

	while(i_arg_end != string::npos && i_arg_start < i_arg_end) {
		args.push_back(str.substr(i_arg_start, i_arg_end - i_arg_start));

		i_arg_start = i_arg_end + 1;
		size_t i_bracket_open = str.find_first_of('[', i_arg_start);
		size_t i_comma = str.find_first_of(',', i_arg_start);

		if (i_bracket_open != string::npos && i_bracket_open < i_comma) {
			i_arg_end = str.find_first_of(']', i_bracket_open) + 1;
			if (i_arg_end >= str.size()) {
				i_arg_end = string::npos;
			}
		} else {
			i_arg_end = str.find_first_of(',', i_arg_start);
		}
	}
	args.push_back(str.substr(i_arg_start));
}

Compound* Parser::parseCompound(const string& line, string& error) {

	string line_no_white = line;
	line_no_white.erase(std::remove(line_no_white.begin(), line_no_white.end(), ' '), line_no_white.end());

	if (line.empty()) {
		return 0;
	}

	if (line[0] == '#' || line[0] == '/') {
		return 0;
	}

	size_t i_open = line_no_white.find_first_of('(');
	size_t i_close = line_no_white.find_last_of(')');

	if (i_open == string::npos || i_close == string::npos) {
		error = "Malformed fact.";
		return 0;
	}

	string predicate = line_no_white.substr(0, i_open);

	Compound* C = new Compound(predicate);

	vector<string> args;
	parseList(line_no_white.substr(i_open + 1, i_close - i_open - 1), args, error);

	for(vector<string>::iterator it_arg = args.begin(); it_arg != args.end(); ++it_arg) {
		string& arg = *it_arg;

		size_t i_arg_open = arg.find_first_of('[');
		if (i_arg_open != string::npos) {
			// pdf
			size_t i_arg_close = arg.find_last_of(']');

			vector<string> values;
			parseList(arg.substr(i_arg_open + 1, i_arg_close - i_arg_open - 1), values, error);

			pbl::Vector vec(values.size());
			for(unsigned int i = 0; i < values.size(); ++i) {
				vec(i) = atof(values[i].c_str());
			}

			pbl::Gaussian G(vec, arma::zeros(values.size(), values.size()));

			C->addArgument(Value(G));

		} else {
			// pmf
			pbl::PMF pmf;
			pmf.setExact(arg);
			C->addArgument(Value(pmf));
		}
	}

	size_t i_prob = line_no_white.find_first_of(':', i_close);
	if (i_prob != string::npos) {
		string prob_str = line_no_white.substr(i_prob + 1);
		double prob = atof(prob_str.c_str());
		C->setProbability(prob);
	} else {
		C->setProbability(1.0);
	}

	printf("%s\n",C->toString().c_str());

	return C;
}

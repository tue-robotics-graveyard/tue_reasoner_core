/*
 * Term.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Term.h"

std::string Term::EMPTY_STRING = "";

Term::Term(Term::Type type) : type_(type) {
}

Term::Term(const Term& orig) : type_(orig.type_) {
}

Term::~Term() {

}

bool Term::isVariable() const {
	return type_ == VARIABLE;
}

bool Term::isValue() const {
	return type_ == VALUE;
}

bool Term::isCompound() const {
	return type_ == COMPOUND;
}

Term::Type Term::type() {
	return type_;
}

const std::string& Term::getName() const {
	return EMPTY_STRING;
}

pbl::PDF* Term::getValue() const {
	return 0;
}

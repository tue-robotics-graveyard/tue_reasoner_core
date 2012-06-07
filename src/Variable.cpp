/*
 * Variable.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Variable.h"

Variable::Variable(const std::string& name) : Term(VARIABLE), name_(name) {

}

Variable::Variable(const Variable& orig) : Term(orig), name_(orig.name_) {

}

Variable::~Variable() {

}

Variable* Variable::clone() const {
	return new Variable(*this);
}

const std::string& Variable::getName() const {
	return name_;
}

BindingSet* Variable::match(const Term& term) const {
	printf("Variable::match - not yet supported\n");
	return 0;
}

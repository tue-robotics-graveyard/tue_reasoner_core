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

const std::string& Variable::getName() const {
	return name_;
}

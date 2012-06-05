/*
 * Term.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Term.h"

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

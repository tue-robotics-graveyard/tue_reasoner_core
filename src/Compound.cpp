/*
 * Predicate.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Compound.h"

Compound::Compound(const std::string& predicate) : Term(COMPOUND), predicate_(predicate) {

}

Compound::Compound(const std::string& predicate, const std::vector<Term>& arguments)
	: Term(COMPOUND), predicate_(predicate), arguments_(arguments) {

}

Compound::Compound(const Compound& orig)
	: Term(orig), predicate_(orig.predicate_), arguments_(orig.arguments_) {

}

Compound::~Compound() {

}

void Compound::addArgument(const Term& term) {

}

std::string Compound::getPredicate() {
	return predicate_;
}

const std::vector<Term>& Compound::getArguments() {
	return arguments_;
}

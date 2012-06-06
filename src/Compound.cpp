/*
 * Predicate.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Compound.h"

Compound::Compound(const std::string& predicate) : Term(COMPOUND), probability_(1.0), predicate_(predicate) {

}

Compound::Compound(const std::string& predicate, const std::vector<Term*>& arguments)
	: Term(COMPOUND), probability_(1.0), predicate_(predicate) {
	for (std::vector<Term*>::const_iterator it = arguments.begin(); it != arguments.end(); ++it) {
		arguments_.push_back(*it);
	}
}

Compound::Compound(const Compound& orig)
	: Term(orig), probability_(orig.probability_), predicate_(orig.predicate_), arguments_(orig.arguments_) {

}

Compound::~Compound() {
	for (std::vector<Term*>::iterator it = arguments_.begin(); it != arguments_.end(); ++it) {
		delete *it;
	}
}

Compound* Compound::clone() const {
	return new Compound(*this);
}

void Compound::addArgument(const Term& term) {
	arguments_.push_back(term.clone());
}

void Compound::setProbability(double probability) {
	probability_ = probability;
}

std::string Compound::getPredicate() const {
	return predicate_;
}

const std::vector<Term*>& Compound::getArguments() const {
	return arguments_;
}

std::string Compound::toString() const {
	std::stringstream ss;
	ss << predicate_ << "(";
	for(std::vector<Term*>::const_iterator it = arguments_.begin(); it !=  arguments_.end(); ++it) {
		const Term* term = *it;

		if (term->isVariable()) {
			ss << "_" << term->getName();
		} else if (term->isValue()) {
			ss << term->getValue()->toString();
		}
		if ((it + 1) != arguments_.end()) {
			ss << ", ";
		}
	}
	ss << ") : " << probability_;
	return ss.str();
}

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

BindingSet* Compound::match(const Term& term) const {
	if (!term.isCompound()) {
		printf("Compound::match - can only match between compounds\n");
		return 0;
	}

	const Compound* C = static_cast<const Compound*>(&term);

	BindingSet* binding_set = new BindingSet();
	double prob = C->probability_ * this->probability_;

	if (C->arguments_.size() == arguments_.size()) {
		for(unsigned int i = 0; i < arguments_.size(); ++i) {
			Term* arg1 = arguments_[i];
			Term* arg2 = C->arguments_[i];

			if (arg1->isValue() && arg2->isValue()) {
				prob *= arg1->getValue()->getLikelihood(*arg2->getValue());
				if (prob == 0) {
					delete binding_set;
					return 0;
				}
			} else {
				Term* var;
				Term* value;

				if (arg1->isValue() && arg2->isVariable()) {
					var = arg2;
					value = arg1;
				} else if (arg1->isVariable() && arg2->isValue()) {
					var = arg1;
					value = arg2;
				} else {
					printf("Compound::match - can not match two variables");
					delete binding_set;
					return 0;
				}

				binding_set->addBinding(var->getName(), value->getValue());
			}
		}
	}

	binding_set->setProbability(prob);
	return binding_set;
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

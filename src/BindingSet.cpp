/*
 * BindingSet.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/BindingSet.h"

BindingSet::BindingSet() : probability_(0) {

}

BindingSet::BindingSet(const BindingSet& orig) : probability_(orig.probability_), bindings_(orig.bindings_) {

}

BindingSet::~BindingSet() {

}

void BindingSet::addBinding(const std::string& variable_name, const pbl::PDF* pdf) {
	bindings_[variable_name] = pdf;
}

const pbl::PDF* BindingSet::getBinding(const std::string& variable_name) const {
	std::map<std::string, const pbl::PDF*>::const_iterator it = bindings_.find(variable_name);

	if (it == bindings_.end()) {
		return 0;
	}
	return it->second;
}

void BindingSet::setProbability(double probability) {
	probability_ = probability;
}

double BindingSet::getProbability() const {
	return probability_;
}

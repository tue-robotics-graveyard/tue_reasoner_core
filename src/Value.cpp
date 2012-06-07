/*
 * Value.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Value.h"

Value::Value(const pbl::PDF& pdf) : Term(VALUE), pdf_(pdf.clone()) {
}

Value::Value(const Value& orig) : Term(orig), pdf_(orig.pdf_->clone()) {
}

Value::~Value() {
	delete pdf_;
}

Value* Value::clone() const {
	return new Value(*this);
}

pbl::PDF* Value::getValue() const {
	return pdf_;
}

BindingSet* Value::match(const Term& term) const {
	printf("Value::match - not yet supported\n");
	return 0;
}

/*
 * Value.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include "tue_reasoner/Value.h"

Value::Value(const pbl::PDF& pdf) : Term(VALUE), pdf_(pdf.clone()) {

}

Value::Value(const Value& orig) : Term(orig), pdf_(pdf_->clone()) {

}

Value::~Value() {
	delete pdf_;
}

pbl::PDF* Value::getValue() const {
	return pdf_;
}

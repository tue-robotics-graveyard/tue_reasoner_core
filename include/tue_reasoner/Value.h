/*
 * Value.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef VALUE_H_
#define VALUE_H_

#include "tue_reasoner/Term.h"

class Value : public Term {

public:

	Value(const pbl::PDF& pdf);

	Value(const Value& orig);

	virtual Value* clone() const;

	virtual ~Value();

	virtual BindingSet* match(const Term& term) const;

	pbl::PDF* getValue() const;

protected:

	pbl::PDF* pdf_;

};

#endif /* VALUE_H_ */

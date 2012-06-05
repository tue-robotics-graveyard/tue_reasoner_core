/*
 * Value.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef VALUE_H_
#define VALUE_H_

#include "tue_reasoner/Term.h"

#include <problib/pdfs/PDF.h>

class Value : public Term {

public:

	Value(const pbl::PDF& pdf);

	Value(const Value& orig);

	virtual ~Value();

protected:

	pbl::PDF* pdf_;

};

#endif /* VALUE_H_ */

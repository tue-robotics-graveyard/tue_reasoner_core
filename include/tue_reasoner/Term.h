/*
 * Term.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef TERM_H_
#define TERM_H_

#include <problib/pdfs/PDF.h>

#include "tue_reasoner/BindingSet.h"

class Term {

public:

	enum Type {
		COMPOUND,
		VARIABLE,
		VALUE
	};

	Term(Type type);

	Term(const Term& orig);

	virtual ~Term();

	virtual Term* clone() const = 0;

	virtual BindingSet* match(const Term& term) const = 0;

	bool isVariable() const;

	bool isValue() const;

	bool isCompound() const;

	virtual const std::string& getName() const;

	virtual pbl::PDF* getValue() const;

	Type type();

protected:

	Type type_;

	static std::string EMPTY_STRING;

};

#endif /* TERM_H_ */

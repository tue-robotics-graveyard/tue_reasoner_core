/*
 * Term.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef TERM_H_
#define TERM_H_

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

	bool isVariable() const;

	bool isValue() const;

	bool isCompound() const;

	Type type();

protected:

	Type type_;

};

#endif /* TERM_H_ */

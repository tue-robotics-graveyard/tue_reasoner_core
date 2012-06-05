/*
 * Variable.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef PREDICATE_H_
#define PREDICATE_H_

#include "tue_reasoner/Term.h"

#include <string>
#include <vector>

class Compound : public Term {

public:

	Compound(const std::string& predicate);

	Compound(const std::string& predicate, const std::vector<Term>& arguments);

	Compound(const Compound& orig);

	void addArgument(const Term& term);

	virtual ~Compound();

	std::string getPredicate();

	const std::vector<Term>& getArguments();


protected:

	std::string predicate_;

	std::vector<Term> arguments_;

};

#endif /* PREDICATE_H_ */

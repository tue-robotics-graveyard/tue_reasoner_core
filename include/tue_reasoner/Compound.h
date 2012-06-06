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

	Compound(const std::string& predicate, const std::vector<Term*>& arguments);

	Compound(const Compound& orig);

	virtual ~Compound();

	virtual Compound* clone() const;

	void addArgument(const Term& term);

	void setProbability(double probability);

	std::string getPredicate() const;

	const std::vector<Term*>& getArguments() const;

	std::string toString() const;


protected:

	double probability_;

	std::string predicate_;

	std::vector<Term*> arguments_;

};

#endif /* PREDICATE_H_ */

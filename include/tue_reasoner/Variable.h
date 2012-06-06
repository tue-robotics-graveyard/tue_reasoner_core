/*
 * Variable.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef VARIABLE_H_
#define VARIABLE_H_

#include "tue_reasoner/Term.h"

#include <string>

class Variable : public Term {

public:

	Variable(const std::string& name);

	Variable(const Variable& orig);

	virtual Variable* clone() const;

	virtual ~Variable();

	const std::string& getName() const;

protected:

	std::string name_;

};

#endif /* VARIABLE_H_ */

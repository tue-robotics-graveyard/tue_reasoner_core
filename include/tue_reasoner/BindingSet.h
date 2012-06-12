/*
 * BindingSet.h
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#ifndef BINDINGSET_H_
#define BINDINGSET_H_

#include <string>
#include <map>

#include <problib/pdfs/PDF.h>

class BindingSet {

public:

	BindingSet();

	BindingSet(const BindingSet& orig);

	virtual ~BindingSet();

	virtual BindingSet* clone() const;

	void addBinding(const std::string& variable_name, const pbl::PDF& pdf);

	const pbl::PDF* getBinding(const std::string& variable_name) const;

	const std::map<std::string, const pbl::PDF*>& getBindings() const;

	void setProbability(double probability);

	double getProbability() const;

protected:

	double probability_;

	std::map<std::string, const pbl::PDF*> bindings_;

};

#endif /* BINDINGSET_H_ */

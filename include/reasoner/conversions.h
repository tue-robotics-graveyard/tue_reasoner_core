#ifndef REASONER_CONVERSIONS_H_
#define REASONER_CONVERSIONS_H_

#include <psi/psi.h>
#include <psi/BindingSet.h>

class PlTerm;
class PlEngine;

PlTerm psiToProlog(const psi::Term& term);

PlTerm psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var);

psi::Term prologToPsi(const PlTerm& pl_term);

psi::Term prologToPsi(const PlTerm& pl_term, std::map<std::string, PlTerm>& str_to_var);

#endif

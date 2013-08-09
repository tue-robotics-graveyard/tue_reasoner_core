#ifndef _Reasoner_H_
#define _Reasoner_H_

#include <psi/psi.h>
#include <psi/BindingSet.h>

class PlTerm;
class PlEngine;

class Reasoner {

public:

    Reasoner();

    virtual ~Reasoner();

    std::vector<psi::BindingSet> query(const psi::Term& query);

protected:

    PlEngine* prolog_engine_;

    static void sighandler(int signo);

    PlTerm psiToProlog(const psi::Term& term) const;

    PlTerm psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var) const;

    psi::Term prologToPsi(const PlTerm& pl_term) const;

};

#endif

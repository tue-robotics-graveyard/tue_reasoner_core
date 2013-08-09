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

    bool loadDatabase(const std::string& filename);

protected:

    PlEngine* prolog_engine_;

    static void sighandler(int signo);

};

#endif

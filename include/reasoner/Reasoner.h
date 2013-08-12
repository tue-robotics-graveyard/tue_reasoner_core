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

    static Reasoner& getInstance();

    std::vector<psi::BindingSet> query(const psi::Term& query);

    bool loadDatabase(const std::string& filename);

    virtual bool customPredicate(const psi::Term& goal, std::vector<psi::BindingSet>& result);

protected:

    PlEngine* prolog_engine_;

    static void sighandler(int signo);

private:

    static Reasoner* instance_;

};

#endif

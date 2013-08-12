#ifndef _Reasoner_H_
#define _Reasoner_H_

#include <psi/psi.h>
#include <psi/Server.h>
#include <psi/BindingSet.h>

class PlTerm;
class PlEngine;

class Reasoner : public psi::Server {

public:

    Reasoner();

    virtual ~Reasoner();

    static Reasoner& getInstance();

    std::vector<psi::BindingSet> query(const psi::Term& query);

    bool loadDatabase(const std::string& filename);

    virtual bool customPredicate(const psi::Term& goal, std::vector<psi::BindingSet>& result);


    // PSI COMMUNICATION

    std::vector<psi::BindingSet> processQuery(const psi::Term& query);

    bool processAssert(const std::vector<psi::Term>& facts);

    bool processRetract(const std::vector<psi::Term>& facts);

protected:

    PlEngine* prolog_engine_;

    static void sighandler(int signo);

private:

    static Reasoner* instance_;

};

#endif

#include "reasoner/Reasoner.h"

#include "swi-cpp/SWI-cpp.h"

#include <iostream>
#include <sstream>

// Needed for specifying custom signal handler
#include <signal.h>

Reasoner::Reasoner() {
    // Initialize Prolog Engine
    putenv((char*)"SWI_HOME_DIR=/usr/lib/swi-prolog");
    //PlEngine prolog_engine(argv[0]);

    char** args = new char*[1];
    args[0] = new char[1000];
    strcpy(args[0], "/");

    prolog_engine_ = new PlEngine(1, args);

    // Prolog has its own signal handler which seems to interfere with ROS' handler
    // Therefore, make sure ROS is shutdown if Prolog receives an interrupt signal
    PL_signal(SIGINT, &Reasoner::sighandler);
}

Reasoner::~Reasoner() {
    delete prolog_engine_;
}

void Reasoner::sighandler(int signo) {
    //ros::shutdown();
}

std::vector<psi::BindingSet> Reasoner::processQuery(const psi::Term& query) {
    PlTermv av(1);
    std::map<std::string, PlTerm> str_to_var;
    av[0] = psiToProlog(query, str_to_var);

    std::stringstream print_out;
    print_out << "?- ";
    print_out << (char*)av[0];

    std::vector<psi::BindingSet> result;

    try {
        PlQuery q("call", av);
        while(q.next_solution()) {
            psi::BindingSet binding_set;

            for(std::map<std::string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {
                binding_set.add(it->first, prologToPsi(it->second));
            }

            result.push_back(binding_set);
        }
    } catch ( PlException &ex ) {
        std::cerr << (char *)ex << std::endl;
    }

    return result;
}

PlTerm Reasoner::psiToProlog(const psi::Term& term) const {
    std::map<std::string, PlTerm> str_to_var;
    return psiToProlog(term, str_to_var);
}

PlTerm Reasoner::psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var) const {
    if (term.isVariable()) {
        // term is a variable
        std::map<std::string, PlTerm>::iterator it_var = str_to_var.find(term.toString());
        if (it_var != str_to_var.end()) {
            // known variable
            return it_var->second;
        } else {
            // unknown variable, so add to map
            PlTerm var;
            str_to_var[term.toString()] = var;
            return var;
        }
    } else if (term.isConstant()) {
        const psi::Constant* con = static_cast<const psi::Constant*>(&term);
        if (con->isNumber()) {
            return PlTerm(con->getNumber());
        } else {
            return PlTerm(con->toString().c_str());
        }

    } else if (term.isSequence()) {
        const psi::Sequence* seq = static_cast<const psi::Sequence*>(&term);

        PlTerm pl_list;
        PlTail l(pl_list);

        for(unsigned int i = 0; i < seq->getSize(); ++i) {
            l.append(psiToProlog(seq->get(i), str_to_var));
        }

        l.close();
        return pl_list;
    } else if (term.isCompound()) {
        // term is a compound
        PlTermv args(term.getSize());

        for(unsigned int i = 0; i < term.getSize(); ++i) {
            args[i] = psiToProlog(term.get(i), str_to_var);
        }

        return PlCompound(term.getFunctor().c_str(), args);
    }

    return PlTerm("UNKNOWN");
}

psi::Term Reasoner::prologToPsi(const PlTerm& pl_term) const {
    try {
        // try if pl_term is a number
        double num = (double)pl_term;
        return psi::Constant(num);
    } catch(const PlTypeError& e) {
    }

    try {
        if (pl_term.arity() == 0) {
            // constant (number was already tested, so it is a string)
            return psi::Constant((char*)pl_term);
        } else {
            // compound

            if (std::string(pl_term.name()) == ".") {
                // special case: list

                psi::Sequence seq;

                PlTail pl_list(pl_term);
                PlTerm elem;
                while(pl_list.next(elem)) {
                    seq.add(prologToPsi(elem));
                    // TODO: fix memory leak
                }
                return seq;
            } else {
                psi::Compound c(pl_term.name());

                for(int i = 0; i < pl_term.arity(); ++i) {
                    c.addArgument(prologToPsi(pl_term[i+1]));
                    // TODO: fix memory leak
                }
                return c;
            }

        }
    } catch (const PlTypeError& e) {
        // pl_term is a variable
        return psi::Variable((char*)pl_term);
    }
}

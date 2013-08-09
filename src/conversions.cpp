#include "reasoner/conversions.h"

#include <swi-cpp/SWI-cpp.h>

PlTerm psiToProlog(const psi::Term& term) {
    std::map<std::string, PlTerm> str_to_var;
    return psiToProlog(term, str_to_var);
}

PlTerm psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var) {
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

psi::Term prologToPsi(const PlTerm& pl_term, std::map<std::string, PlTerm>& str_to_var) {
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
                    seq.add(prologToPsi(elem, str_to_var));
                    // TODO: fix memory leak
                }
                return seq;
            } else {
                psi::Compound c(pl_term.name());

                for(int i = 0; i < pl_term.arity(); ++i) {
                    c.addArgument(prologToPsi(pl_term[i+1], str_to_var));
                    // TODO: fix memory leak
                }
                return c;
            }

        }
    } catch (const PlTypeError& e) {
        // pl_term is a variable
        str_to_var[(char*)pl_term] = pl_term;
        return psi::Variable((char*)pl_term);

    }
}

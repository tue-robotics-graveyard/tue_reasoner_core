#include "reasoner/Reasoner.h"
#include "reasoner/conversions.h"

#include <psi/Client.h>

#include <SWI-Prolog.h>
#include <swi-cpp/SWI-cpp.h>

using namespace std;

map<string, psi::Client*> psi_clients_;

PREDICATE(extern, 2) {
    std::map<std::string, PlTerm> str_to_var;
    psi::Term q = prologToPsi(A1, str_to_var);
    string server_name = (char*)A2;

    psi::Client* client;

    map<string, psi::Client*>::iterator it = psi_clients_.find(server_name);
    if (it == psi_clients_.end()) {
        client = new psi::Client(server_name);
        psi_clients_[server_name] = client;
    } else {
        client = it->second;
    }

    vector<psi::BindingSet> binding_sets = client->query(q);
    for(vector<psi::BindingSet>::iterator it = binding_sets.begin(); it != binding_sets.end(); ++it) {
        const psi::BindingSet& binding_set = *it;
        const map<string, psi::Term> bindings = binding_set.getAllBindings();
        for(map<string, psi::Term>::const_iterator it2 = bindings.begin(); it2 != bindings.end(); ++it2) {
            std::string var_name = it2->first;
            psi::Term result = it2->second;

            PlTerm var = str_to_var[var_name];
            var = psiToProlog(result);
        }

        return true;
    }

    return true;
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "reasoner_repl");

    Reasoner r;

    PL_install_readline();

    for(;;) {
        int status = PL_toplevel() ? 0 : 1;
        PL_halt(status);
    }
}

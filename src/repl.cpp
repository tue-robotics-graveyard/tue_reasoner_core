#include "reasoner/Reasoner.h"
#include "reasoner/conversions.h"

#include <psi/Client.h>

#include <SWI-Prolog.h>
#include <swi-cpp/SWI-cpp.h>

using namespace std;

map<string, psi::Client*> psi_clients_;

PREDICATE(extern, 3) {
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

    PlTail binding_list_list(A3);

    vector<psi::BindingSet> binding_sets = client->query(q);
    for(vector<psi::BindingSet>::iterator it = binding_sets.begin(); it != binding_sets.end(); ++it) {

        PlTerm t_binding_list;
        PlTail binding_list(t_binding_list);

        const psi::BindingSet& binding_set = *it;
        const map<string, psi::Term> bindings = binding_set.getAllBindings();
        for(map<string, psi::Term>::const_iterator it2 = bindings.begin(); it2 != bindings.end(); ++it2) {
            PlTermv binding_args(2);
            binding_args[0] = str_to_var[it2->first];
            binding_args[1] = psiToProlog(it2->second, str_to_var);
            binding_list.append(PlCompound("=", binding_args));
        }

        binding_list.close();
        binding_list_list.append(t_binding_list);
    }

    binding_list_list.close();

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

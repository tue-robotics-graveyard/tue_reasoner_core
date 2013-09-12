#include "reasoner/Reasoner.h"
#include "reasoner/conversions.h"

#include <psi/Client.h>

#include "swi-cpp/SWI-cpp.h"

#include <iostream>
#include <sstream>

// Needed for specifying custom signal handler
#include <signal.h>

#include <ros/package.h>

Reasoner* Reasoner::instance_ = 0;

Reasoner::Reasoner() : psi::Server(ros::NodeHandle("~").getNamespace() + "/reasoner") {
    if (instance_) {
        // throw error
        printf("ERROR: Only one Reasoner object may exist per process.\n");
        return;
    }

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

    // load std.pl
    std::string package_path = ros::package::getPath("tue_reasoner_core");
    loadDatabase(package_path + "/prolog/std.pl");

    instance_ = this;
}

Reasoner::~Reasoner() {
    delete prolog_engine_;
    instance_ = 0;
}

Reasoner& Reasoner::getInstance() {
    if (!instance_) {
        instance_ = new Reasoner();
    }
    return *instance_;
}

void Reasoner::sighandler(int signo) {
    ros::shutdown();
}

std::vector<psi::BindingSet> Reasoner::query(const psi::Term& query) {
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

bool Reasoner::loadDatabase(const std::string& filename) {
    return PlCall("consult", PlTermv(filename.c_str()));
}

bool Reasoner::customPredicate(const psi::Term& goal, std::vector<psi::BindingSet>& result) {
    return false;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                PSI COMMUNICATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

std::vector<psi::BindingSet> Reasoner::processQuery(const psi::Term& q) {
    return this->query(q);
}

bool Reasoner::processAssert(const std::vector<psi::Term>& facts) {
    return false;
}

bool Reasoner::processRetract(const std::vector<psi::Term>& facts) {
    return false;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                    PREDICATES
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

std::map<std::string, psi::Client*> psi_clients_;

PREDICATE(extern, 3) {
    std::map<std::string, PlTerm> str_to_var;
    psi::Term q = prologToPsi(A1, str_to_var);
    std::string server_name = (char*)A2;

    psi::Client* client;

    std::map<std::string, psi::Client*>::iterator it = psi_clients_.find(server_name);
    if (it == psi_clients_.end()) {
        client = new psi::Client(server_name);
        psi_clients_[server_name] = client;
    } else {
        client = it->second;
    }

    PlTail binding_list_list(A3);

    std::vector<psi::BindingSet> binding_sets = client->query(q);
    for(std::vector<psi::BindingSet>::iterator it = binding_sets.begin(); it != binding_sets.end(); ++it) {

        PlTerm t_binding_list;
        PlTail binding_list(t_binding_list);

        const psi::BindingSet& binding_set = *it;
        const std::map<std::string, psi::Term> bindings = binding_set.getAllBindings();
        for(std::map<std::string, psi::Term>::const_iterator it2 = bindings.begin(); it2 != bindings.end(); ++it2) {
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

PREDICATE(rosparam, 2) {
    std::string param_name = (char*)A1;

    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(param_name, v)) {
        return false;
    }

    if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
        A2 = PlAtom(((std::string)v).c_str());
    }

    return true;
}

PREDICATE(custom, 1) {
    std::map<std::string, PlTerm> str_to_var;
    psi::Term t = prologToPsi(A1, str_to_var);
    std::vector<psi::BindingSet> result;
    bool success = Reasoner::getInstance().customPredicate(t, result);

    if (!success || result.empty()) {
        return success;
    }

    // for now, only look at first binding:
    std::map<std::string, psi::Term> bindings = result.front().getAllBindings();

    for(std::map<std::string, psi::Term>::iterator it = bindings.begin(); it != bindings.end(); ++it) {
        std::map<std::string, PlTerm>::iterator it_var = str_to_var.find(it->first);
        if (it_var != str_to_var.end()) {
            it_var->second = psiToProlog(it->second, str_to_var);
        }
    }

    return true;
}

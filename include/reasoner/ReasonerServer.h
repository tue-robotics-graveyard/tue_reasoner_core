#ifndef TUE_REASONER_H_
#define TUE_REASONER_H_

#include <psi/Server.h>
#include <psi/Client.h>

#include <tf/transform_listener.h>

//#include <wire/ServerROS.h>
//#include <wire_interface/Property.h>

#include "swi-cpp/SWI-cpp.h"

// Needed for specifying custom signal handler
#include <signal.h>

class ReasonerServer : public psi::Server {

public:

    ReasonerServer(const std::string& service_name, const std::string& wm_type);

    virtual ~ReasonerServer();

    void start();

    std::vector<psi::BindingSet> processQuery(const psi::Term& query);

    bool processAssert(const std::vector<psi::Term>& facts);

    bool processRetract(const std::vector<psi::Term>& facts);

    bool loadDatabase(const std::string& filename);

    // predicates

    bool pred_property_list(PlTerm a1, PlTerm a2, PlTerm a3, PlTerm a4);

    bool pred_property_expected_list(PlTerm a1, PlTerm a2, PlTerm a3, PlTerm a4);

    bool pred_add_evidence(PlTerm a1);

    bool pred_lookup_transform(PlTerm a1, PlTerm a2, PlTerm a3);

    bool pred_transform_point(PlTerm frame_in, PlTerm point_in, PlTerm frame_out, PlTerm point_out);

    bool pred_quaternion_to_rpy(PlTerm quat, PlTerm rpy);

protected:

    //wire::ServerROS* wire_server_;

    psi::Client* wire_client_;

    tf::TransformListener* tf_listener_;

    ros::ServiceClient ed_client_;

    static void sighandler(int signo);

    PlTerm psiToProlog(const psi::Term& term) const;

    PlTerm psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var) const;

    psi::Term prologToPsi(const PlTerm& pl_term) const;

};

#endif

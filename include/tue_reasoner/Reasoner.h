#ifndef TUE_REASONER_H_
#define TUE_REASONER_H_

#include <psi/Server.h>

#include <tf/transform_listener.h>

#include <wire/ServerROS.h>
#include <wire_interface/Property.h>

#include "swi-cpp/SWI-cpp.h"

class Reasoner : public psi::Server {

public:

    Reasoner(const std::string& service_query, const std::string& service_assert);

    virtual ~Reasoner();

    void start();

    std::vector<psi::BindingSet> processQuery(const psi::Term& query);

    bool processAssert(const std::vector<psi::Term>& facts);

    bool processRetract(const std::vector<psi::Term>& facts);

    bool loadDatabase(const std::string& filename);

    // predicates

    bool pred_property_list(PlTerm a1, PlTerm a2, PlTerm a3);

    bool pred_add_evidence(PlTerm a1);

    bool pred_lookup_transform(PlTerm a1, PlTerm a2, PlTerm a3);

    bool pred_transform_point(PlTerm frame_in, PlTerm point_in, PlTerm frame_out, PlTerm point_out);

protected:

    wire::ServerROS* wire_server_;

    ros::Publisher pub_evidence_;

    tf::TransformListener* tf_listener_;

    PlTerm psiToProlog(const psi::Term& term) const;

    PlTerm psiToProlog(const psi::Term& term, std::map<std::string, PlTerm>& str_to_var) const;

    psi::Term prologToPsi(const PlTerm& pl_term) const;

};

#endif

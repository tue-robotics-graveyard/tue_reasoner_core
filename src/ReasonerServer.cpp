#include <ros/ros.h>

#include "reasoner/ReasonerServer.h"

#include <psi/Constant.h>
#include <psi/Compound.h>
#include <psi/Sequence.h>

//#include <problib/conversions.h>
//#include <problib/conversions_psi.h>

#include <ed/SimpleQuery.h>

#include <ros/package.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

ReasonerServer::ReasonerServer(const std::string& service_name, const std::string& wm_type)
    : psi::Server(service_name), wire_client_(0)
{
    ros::NodeHandle nh_private("~");

    tf_listener_ = new tf::TransformListener();

    // create world model
    //wire_server_ = new wire::ServerROS();
    //wire_server_->registerEvidenceTopic("/world_evidence");

    if (wm_type == "wire")
    {
        wire_client_ = new psi::Client("/wire", service_name + "/wire/query_result");
    }
    else if (wm_type == "ed")
    {
        ros::NodeHandle nh_;
        ed_client_ = nh_.serviceClient<ed::SimpleQuery>("/ed/simple_query");
        ed_client_.waitForExistence();
    }

    // Prolog has its own signal handler which seems to interfere with ROS' handler
    // Therefore, make sure ROS is shutdown if Prolog receives an interrupt signal
    PL_signal(SIGINT, &ReasonerServer::sighandler);
}

// ----------------------------------------------------------------------------------------------------

ReasonerServer::~ReasonerServer()
{
    delete wire_client_;
    //delete wire_server_;
}

// ----------------------------------------------------------------------------------------------------

void ReasonerServer::start()
{
    //wire_server_->start();
    ros::spin();
}

// ----------------------------------------------------------------------------------------------------

vector<psi::BindingSet> ReasonerServer::processQuery(const psi::Term& query) {

    //    if (query.getFunctor() == "property") {
    //        ROS_DEBUG("processQuery: Rerouting to WIRE: %s", query.toString().c_str());
    //        return wire_client_->query(query);
    //    }

    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);

    PlTermv av(1);
    map<string, PlTerm> str_to_var;
    av[0] = psiToProlog(query, str_to_var);

    stringstream print_out;
    print_out << "?- ";
    print_out << (char*)av[0];

    ROS_DEBUG("%s", print_out.str().c_str());

    vector<psi::BindingSet> result;

    try {
        PlQuery q("call", av);
        while(q.next_solution()) {
            psi::BindingSet binding_set;

            for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {
                binding_set.add(it->first, prologToPsi(it->second));
            }

            result.push_back(binding_set);
        }
    } catch ( PlException &ex ) {
        std::cerr << (char *)ex << std::endl;
    }

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
    ROS_DEBUG("       Reasoner: query took  %f seconds.", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);

    return result;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::processAssert(const vector<psi::Term>& facts) {

    if (facts.empty()) {
        ROS_ERROR("Empty received empty fact list");
        return false;
    }

    stringstream errors;

    for(vector<psi::Term>::const_iterator it = facts.begin(); it != facts.end(); ++it) {
        const psi::Term& term = *it;

        PlTermv av(1);
        map<string, PlTerm> str_to_var;
        av[0] = psiToProlog(term, str_to_var);

        ROS_DEBUG("?- \033[1m%s(%s)\033[0m", "assertz", (char*)av[0]);
        PlQuery q("assertz", av);

        try {
            while (q.next_solution()) {
            }
        } catch ( PlException &ex ) {
            errors << (char *)ex << std::endl;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::processRetract(const std::vector<psi::Term>& facts)
{

    if (facts.empty()) {
        ROS_ERROR("Empty received empty retract list");
        return false;
    }

    stringstream errors;

    for(vector<psi::Term>::const_iterator it = facts.begin(); it != facts.end(); ++it) {
        const psi::Term& term = *it;

        PlTermv av(1);
        map<string, PlTerm> str_to_var;
        av[0] = psiToProlog(term, str_to_var);

        ROS_DEBUG("?- \033[1m%s(%s)\033[0m", "retractall", (char*)av[0]);
        PlQuery q("retractall", av);

        try {
            while (q.next_solution()) {
            }
        } catch ( PlException &ex ) {
            errors << (char *)ex << std::endl;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

PlTerm ReasonerServer::psiToProlog(const psi::Term& term) const
{
    map<string, PlTerm> str_to_var;
    return psiToProlog(term, str_to_var);
}

// ----------------------------------------------------------------------------------------------------

PlTerm ReasonerServer::psiToProlog(const psi::Term& term, map<string, PlTerm>& str_to_var) const
{
    if (term.isVariable()) {
        // term is a variable
        map<string, PlTerm>::iterator it_var = str_to_var.find(term.toString());
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

            // TODO: convert number array
            /*
            PlTerm term;
            PlTail list(term);

            for(unsigned int i = 0; i < msg.constant.num_array.size(); ++i) {
                list.append(PlTerm(msg.constant.num_array[i]));
            }
            list.close();
            return term;

        */

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

    ROS_ERROR_STREAM("INVALID TERM");
    return PlTerm("UNKNOWN");
}

// ----------------------------------------------------------------------------------------------------

psi::Term ReasonerServer::prologToPsi(const PlTerm& pl_term) const
{
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

            if (string(pl_term.name()) == ".") {
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

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::pred_property_expected_list(PlTerm a1, PlTerm a2, PlTerm a3, PlTerm a4)
{
    PlTail property_list(a4);

    PlTermv binding(3);
    binding[0] = "a1";
    binding[1] = "a2";
    binding[2] = a3;

    property_list.append(PlCompound("binding", binding));

    property_list.close();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::pred_property_list(PlTerm a1, PlTerm a2, PlTerm a3, PlTerm a4)
{

    PlTail property_list(a4);

    psi::Term obj_id_psi = prologToPsi(a1);
    psi::Term attr_psi = prologToPsi(a2);

    if (wire_client_)
    {
        ROS_DEBUG("pred_property_list: Rerouting to WIRE: property(%s, %s, %s)", (char*)a1, (char*)a2, (char*)a3);

        vector<psi::BindingSet> result = wire_client_->query(psi::Compound("property",
                                                                           obj_id_psi,
                                                                           prologToPsi(a2),
                                                                           psi::Variable("Value"),
                                                                           prologToPsi(a3)));

        for(vector<psi::BindingSet>::iterator it_res = result.begin(); it_res != result.end(); ++it_res) {
            psi::BindingSet b = *it_res;
            psi::Term val = b.get(psi::Variable("Value"));
            if (val.isValid()) {
                PlTermv binding(3);

                if (obj_id_psi.isVariable()) {
                    binding[0] = psiToProlog(b.get(obj_id_psi.toVariable()));
                } else {
                    binding[0] = obj_id_psi.toString().c_str();
                }

                if (attr_psi.isVariable()) {
                    binding[1] = psiToProlog(b.get(attr_psi.toVariable()));
                } else {
                    binding[1] = attr_psi.toString().c_str();
                }

                binding[2] = psiToProlog(val);
                property_list.append(PlCompound("binding", binding));
            }
        }
    }
    else if (ed_client_.exists())
    {
        psi::Term frame_id_psi = prologToPsi(a3);
        if (!frame_id_psi.isVariable() && frame_id_psi.toString() != "/map" && frame_id_psi.toString() != "map")
        {
            ROS_ERROR("ED Queries have to be in map frame. eceived query: 'property(%s, %s, %s, %s)'", (char*)a1, (char*)a2, "Value", (char*)a3);
        }
        else
        {
            ed::SimpleQuery srv;

            if (!obj_id_psi.isVariable())
                srv.request.id = obj_id_psi.toString();

            if (ed_client_.call(srv))
            {
                for(std::vector<ed::EntityInfo>::const_iterator it = srv.response.entities.begin(); it != srv.response.entities.end(); ++it)
                {
                    const ed::EntityInfo& e = *it;

                    if (attr_psi.isVariable() || attr_psi.toString() == "class_label")
                    {
                        if (!e.type.empty())
                        {
                            PlTermv binding(3);
                            binding[0] = e.id.c_str();
                            binding[1] = "class_label";

                            psi::Sequence probs;
                            probs.add(psi::Compound("p", psi::Constant(1.0), psi::Constant(e.type)));

                            binding[2] = psiToProlog(psi::Compound("discrete", psi::Constant(1), probs));

                            property_list.append(PlCompound("binding", binding));
                        }
                    }

                    if (attr_psi.isVariable() || attr_psi.toString() == "position")
                    {
                        PlTermv binding(3);
                        binding[0] = e.id.c_str();
                        binding[1] = "position";

                        psi::Sequence pos;
                        pos.add(psi::Constant(e.pose.position.x));
                        pos.add(psi::Constant(e.pose.position.y));
                        pos.add(psi::Constant(e.pose.position.z));

                        psi::Sequence cov;
                        cov.add(psi::Constant(1));
                        cov.add(psi::Constant(0));
                        cov.add(psi::Constant(0));
                        cov.add(psi::Constant(1));
                        cov.add(psi::Constant(0));
                        cov.add(psi::Constant(1));

                        binding[2] = psiToProlog(psi::Compound("gaussian", psi::Compound("vector", pos), psi::Compound("symmetric_matrix", cov)));

                        property_list.append(PlCompound("binding", binding));
                    }

                    // TODO: property 'name' (for Cocktail party)
                }
            }

            else
            {
                ROS_ERROR("Failed to call service %s", ed_client_.getService().c_str());
            }
        }
    }

    property_list.close();

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::pred_lookup_transform(PlTerm a1, PlTerm a2, PlTerm a3) {
    tf::StampedTransform transform;

    try {
        tf_listener_->lookupTransform((string)a1, (string)a2, ros::Time(), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    PlTermv vec_args(3);
    vec_args[0] = transform.getOrigin().getX();
    vec_args[1] = transform.getOrigin().getY();
    vec_args[2] = transform.getOrigin().getZ();

    PlTermv quat_args(4);
    quat_args[0] = transform.getRotation().getX();
    quat_args[1] = transform.getRotation().getY();
    quat_args[2] = transform.getRotation().getZ();
    quat_args[3] = transform.getRotation().getW();

    PlTermv tf_args(2);
    tf_args[0] = PlCompound("vector", vec_args);
    tf_args[1] = PlCompound("quaternion", quat_args);

    a3 = PlCompound("transform", tf_args);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::pred_transform_point(PlTerm frame_in, PlTerm point_in, PlTerm frame_out, PlTerm point_out) {

    psi::Term point_in_psi = prologToPsi(point_in);
    double x = point_in_psi.get(0).getNumber();
    double y = point_in_psi.get(1).getNumber();
    double z = point_in_psi.get(2).getNumber();

    tf::Stamped<tf::Point> point_stamped(tf::Point(x, y, z), ros::Time(), (string)frame_in);

    try{
        tf::Stamped<tf::Point> point_stamped_out;
        tf_listener_->transformPoint((string)frame_out, point_stamped, point_stamped_out);

        psi::Sequence seq;
        seq.add(psi::Constant(point_stamped_out.getX()));
        seq.add(psi::Constant(point_stamped_out.getY()));
        seq.add(psi::Constant(point_stamped_out.getZ()));

        point_out = psiToProlog(seq);
        return true;
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::pred_quaternion_to_rpy(PlTerm quat, PlTerm rpy)
{
    psi::Term quat_psi = prologToPsi(quat);
    tf::Quaternion q(quat_psi.get(0).getNumber(), quat_psi.get(1).getNumber(), quat_psi.get(2).getNumber(), quat_psi.get(3).getNumber());

    tf::Matrix3x3 m(q);
    double yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);

    rpy = psiToProlog(psi::Compound("rpy", psi::Constant(yaw), psi::Constant(pitch), psi::Constant(roll)));
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReasonerServer::loadDatabase(const string& filename)
{
    return PlCall("consult", PlTermv(filename.c_str()));
}

// ----------------------------------------------------------------------------------------------------

void ReasonerServer::sighandler(int signo)
{
    ros::shutdown();
}

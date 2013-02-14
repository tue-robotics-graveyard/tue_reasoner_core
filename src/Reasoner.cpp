#include <ros/ros.h>

#include "tue_reasoner/Reasoner.h"

#include <psi/Constant.h>
#include <psi/Compound.h>
#include <psi/Sequence.h>

#include <problib/conversions.h>
#include <problib/conversions_psi.h>

#include <wire_msgs/WorldEvidence.h>

using namespace std;

Reasoner* REASONER;

Reasoner::Reasoner(const std::string& service_query, const std::string& service_assert) : psi::Server(service_query, service_assert) {
    ros::NodeHandle nh_private("~");

    tf_listener_ = new tf::TransformListener();

    // create world model
    wire_server_ = new wire::ServerROS();
    wire_server_->registerEvidenceTopic("/world_evidence");

    pub_evidence_ = nh_private.advertise<wire_msgs::WorldEvidence>("/world_evidence", 10);
}

Reasoner::~Reasoner() {
    delete wire_server_;
}

void Reasoner::start() {
    wire_server_->start();
}

vector<psi::BindingSet> Reasoner::processQuery(const psi::Term& query) {

    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);

    PlTermv av(1);
    map<string, PlTerm> str_to_var;
    av[0] = psiToProlog(query, str_to_var);

    stringstream print_out;
    print_out << "?- ";
    print_out << (char*)av[0];

    ROS_INFO("%s", print_out.str().c_str());

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
    ROS_INFO("       Reasoner: query took  %f seconds.", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);

    return result;
}

bool Reasoner::processAssert(const vector<psi::Term>& facts) {

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

        ROS_INFO("?- \033[1m%s(%s)\033[0m", "assertz", (char*)av[0]);
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

bool Reasoner::processRetract(const std::vector<psi::Term>& facts) {

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

        ROS_INFO("?- \033[1m%s(%s)\033[0m", "retractall", (char*)av[0]);
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

PlTerm Reasoner::psiToProlog(const psi::Term& term) const {
    map<string, PlTerm> str_to_var;
    return psiToProlog(term, str_to_var);
}

PlTerm Reasoner::psiToProlog(const psi::Term& term, map<string, PlTerm>& str_to_var) const {
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
        PlTermv args(term.getArity());

        for(unsigned int i = 0; i < term.getArity(); ++i) {
            args[i] = psiToProlog(term.getArgument(i), str_to_var);
        }

        return PlCompound(term.getFunctor().c_str(), args);
    }

    ROS_ERROR_STREAM("INVALID TERM");
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

string Reasoner::IDIntToString(int ID) {
    stringstream ss;
    ss << "id-" << ID;
    return ss.str();
}

int Reasoner::IDStringToInt(const string& ID) {
    if(ID.substr(0, 3) == "id-") {
        return atoi(ID.substr(3).c_str());
    }
    return -1;
}

bool Reasoner::pred_property_list(PlTerm a1, PlTerm a2, PlTerm a3) {
    int obj_id = IDStringToInt((char*)a1);
    string property = (char*)a2;
    PlTail property_list(a3);

    const list<wire::Object*>& objs = wire_server_->getMAPObjects(ros::Time::now());

    for(list<wire::Object*>::const_iterator it_obj = objs.begin(); it_obj != objs.end(); ++it_obj) {
        const wire::Object& obj = **it_obj;

        if (obj_id < 0 || obj_id == obj.getID()) {
            const wire::Property* prop = obj.getProperty(property);

            if (prop) {
                PlTermv binding(2);
                binding[0] = IDIntToString(obj.getID()).c_str();
                binding[1] = psiToProlog(pbl::toPSI(prop->getValue()));
                property_list.append(PlCompound("binding", binding));
            }
        }
    }

    property_list.close();

    return true;
}

PREDICATE(property_list, 3) {
    return REASONER->pred_property_list(A1, A2, A3);
}

bool Reasoner::pred_add_evidence(PlTerm a1) {
    PlTail tail(a1);
    PlTerm e;

    map<string, wire_msgs::ObjectEvidence> id_to_evidence;

    while(tail.next(e)) {
        string functor = (string)e.name();
        if (functor != "property" || e.arity() != 3) {
            return false;
        }

        string id = (string)e[1];
        string attribute = (string)e[2];
        psi::Term pdf_psi = prologToPsi(e[3]);

        pbl::PDF* pdf = pbl::toPDF(pdf_psi);

        wire_msgs::Property prop;
        prop.attribute = attribute;

        if (pdf) {
            prop.pdf = pbl::PDFtoMsg(*pdf);
            delete pdf;
        } else {
            pbl::PMF pmf;
            pmf.setExact(pdf_psi.toString());
            prop.pdf = pbl::PDFtoMsg(pmf);
        }

        map<string, wire_msgs::ObjectEvidence>::iterator it_id = id_to_evidence.find(id);
        if (it_id == id_to_evidence.end()) {
            wire_msgs::ObjectEvidence ev_obj;
            ev_obj.certainty = 1.0;
            ev_obj.properties.push_back(prop);
            id_to_evidence[id] = ev_obj;
        } else {
            it_id->second.properties.push_back(prop);
        }
    }

    wire_msgs::WorldEvidence ev_world;
    ev_world.header.frame_id = "/map";
    ev_world.header.stamp = ros::Time::now();

    for(map<string, wire_msgs::ObjectEvidence>::iterator it_id = id_to_evidence.begin(); it_id != id_to_evidence.end(); ++it_id) {
        ev_world.object_evidence.push_back(it_id->second);
    }

    pub_evidence_.publish(ev_world);

    return true;
}

PREDICATE(add_evidence, 1) {
    return REASONER->pred_add_evidence(A1);
}


bool Reasoner::pred_lookup_transform(PlTerm a1, PlTerm a2, PlTerm a3) {
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

PREDICATE(lookup_transform, 3) {
    return REASONER->pred_lookup_transform(A1, A2, A3);
}

bool Reasoner::loadDatabase(const string& filename) {
    return PlCall("consult", PlTermv(filename.c_str()));
}


int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "reasoner");
    ros::NodeHandle nh_private("~");

    // Initialize Prolog Engine
    putenv("SWI_HOME_DIR=/usr/lib/swi-prolog");
    //PlEngine prolog_engine(argv[0]);

    PlEngine prolog_engine(argc, argv);

    REASONER = new Reasoner("/reasoner/query", "/reasoner/assert");

    for(int i_database = 1; true; ++i_database) {
        stringstream s_param;
        s_param << "database" << i_database;

        XmlRpc::XmlRpcValue database_name;
        if (nh_private.getParam(s_param.str(), database_name)) {
            if (database_name.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("Each database name must be a string.");
                return -1;
            }

            string database_name_str = (string)database_name;
            if (!REASONER->loadDatabase(database_name_str)) {
                ROS_ERROR("Failed to load database: %s", (database_name_str).c_str());
                return -1;
            }
        } else {
            break;
        }
    }

    REASONER->start();

    delete REASONER;

    return 0;
}

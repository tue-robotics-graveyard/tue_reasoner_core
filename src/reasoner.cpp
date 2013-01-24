/*
 * reasoner.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "tue_reasoner/Variable.h"
#include "tue_reasoner/Value.h"
#include "tue_reasoner/Compound.h"
#include "tue_reasoner/BindingSet.h"
#include "tue_reasoner/Parser.h"

#include <tue_reasoner_msgs/Query.h>
#include <tue_reasoner_msgs/Assert.h>
#include <tue_reasoner_msgs/LoadDatabase.h>
#include <tue_reasoner_msgs/BindingSet.h>

#include <problib/conversions.h>

#include <world_model/WorldModelROS.h>
#include <world_model/storage/SemanticObject.h>
#include <world_model/core/Property.h>

#include <vector>
#include <string>

#include "swi-cpp/SWI-cpp.h"

using namespace std;

//map<string, vector<Compound*> > facts_;

mhf::WorldModelROS* world_model_;

string IDIntToString(int ID) {
    stringstream ss;
    ss << "id-" << ID;
    return ss.str();
}

int IDStringToInt(const string& ID) {
    if(ID.substr(0, 3) == "id-") {
        return atoi(ID.substr(3).c_str());
    }
    return -1;
}

/*
 * Get the most probable Gaussian from a pdf
 */
const pbl::Gaussian* getBestGaussian(const pbl::PDF& pdf, double min_weight = 0) {
    if (pdf.type() == pbl::PDF::GAUSSIAN) {
        return pbl::PDFtoGaussian(pdf);
    } else if (pdf.type() == pbl::PDF::MIXTURE) {
        const pbl::Mixture* mix = pbl::PDFtoMixture(pdf);

        if (mix){
            const pbl::Gaussian* G_best = 0;
            double w_best = min_weight;
            for(int i = 0; i < mix->components(); ++i) {
                const pbl::PDF& pdf = mix->getComponent(i);
                const pbl::Gaussian* G = pbl::PDFtoGaussian(pdf);
                double w = mix->getWeight(i);
                if (G && w > w_best) {
                    G_best = G;
                    w_best = w;
                }
            }
            return G_best;
        }
    }
    return 0;
}

PREDICATE(position_list, 2) {
    int obj_id = IDStringToInt((char*)A1);
    PlTail position_list(A2);

    const list<mhf::SemanticObject*> objs = world_model_->getMAPObjects();
    for(list<mhf::SemanticObject*>::const_iterator it_obj = objs.begin(); it_obj != objs.end(); ++it_obj) {
        const mhf::SemanticObject& obj = **it_obj;
        if (obj_id < 0 || obj_id == obj.getID()) {
            const mhf::Property* pos_prop = obj.getProperty("position");
            const pbl::Gaussian* pos_gauss = getBestGaussian(pos_prop->getValue());
            if (pos_gauss) {
                const pbl::Vector& mean = pos_gauss->getMean();
                PlTermv xyz(3);
                xyz[0] = mean(0);
                xyz[1] = mean(1);
                xyz[2] = mean(2);

                PlTermv binding(2);
                binding[0] = IDIntToString(obj.getID()).c_str();
                binding[1] = PlCompound("point", xyz);

                position_list.append(PlCompound("binding", binding));
            }
        }
    }

    position_list.close();
    return TRUE;
}

PREDICATE(type_list, 2) {
    int obj_id = IDStringToInt((char*)A1);
    PlTail type_list(A2);

    const list<mhf::SemanticObject*> objs = world_model_->getMAPObjects();
    for(list<mhf::SemanticObject*>::const_iterator it_obj = objs.begin(); it_obj != objs.end(); ++it_obj) {
        const mhf::SemanticObject& obj = **it_obj;
        if (obj_id < 0 || obj_id == obj.getID()) {
            const mhf::Property* class_prop = obj.getProperty("class_label");
            if (class_prop) {
                const pbl::PMF* class_pmf = pbl::PDFtoPMF(class_prop->getValue());
                if (class_pmf) {
                    string class_label;
                    class_pmf->getExpectedValue(class_label);

                    PlTermv binding(2);
                    binding[0] = IDIntToString(obj.getID()).c_str();
                    binding[1] = class_label.c_str();

                    type_list.append(PlCompound("binding", binding));
                }
            }
        }
    }

    type_list.close();
    return TRUE;
}

PREDICATE(object_property_list, 3) {
    int obj_id = IDStringToInt((char*)A1);
    string property = (char*)A2;
    PlTail property_list(A3);

    const list<mhf::SemanticObject*> objs = world_model_->getMAPObjects();
    for(list<mhf::SemanticObject*>::const_iterator it_obj = objs.begin(); it_obj != objs.end(); ++it_obj) {
        const mhf::SemanticObject& obj = **it_obj;
        if (obj_id < 0 || obj_id == obj.getID()) {
            const mhf::Property* class_prop = obj.getProperty(property);
            if (class_prop) {
                const pbl::PMF* class_pmf = pbl::PDFtoPMF(class_prop->getValue());
                if (class_pmf) {
                    string class_label;
                    class_pmf->getExpectedValue(class_label);

                    PlTermv binding(2);
                    binding[0] = IDIntToString(obj.getID()).c_str();
                    binding[1] = class_label.c_str();

                    property_list.append(PlCompound("binding", binding));
                }
            }
        }
    }

    property_list.close();
    return TRUE;
}

PlTerm MsgToPrologTerm(const tue_reasoner_msgs::TermImpl& msg, const tue_reasoner_msgs::Term& full_term_msg, map<string, PlTerm>& str_to_var) {
    if (msg.type == tue_reasoner_msgs::TermImpl::VARIABLE) {
        // term is a variable
        map<string, PlTerm>::iterator it_var = str_to_var.find(msg.variable);
        if (it_var != str_to_var.end()) {
            // known variable
            return it_var->second;
        } else {
            // unknown variable, so add to map
            PlTerm var;
            str_to_var[msg.variable] = var;
            return var;
        }
    } else if (msg.type == tue_reasoner_msgs::TermImpl::CONSTANT) {
        // term is a constant
        if (msg.constant.type == tue_reasoner_msgs::Constant::STRING) {
            return PlTerm(msg.constant.str.c_str());
        } else if (msg.constant.type == tue_reasoner_msgs::Constant::NUMBER) {
            return PlTerm(msg.constant.num);
        } else if (msg.constant.type == tue_reasoner_msgs::Constant::NUMBER_ARRAY) {
            PlTerm term;
            PlTail list(term);

            for(unsigned int i = 0; i < msg.constant.num_array.size(); ++i) {
                list.append(PlTerm(msg.constant.num_array[i]));
            }
            list.close();
            return term;
        } else if (msg.constant.type == tue_reasoner_msgs::Constant::PDF) {
            ROS_ERROR("CANNOT ADD PDF's TO THE PROLOG DATABASE YET.");
            return PlTerm("PDF");
        }
    } else if (msg.type == tue_reasoner_msgs::TermImpl::COMPOUND) {
        // term is a compound
        PlTermv args(msg.sub_term_ptrs.size());

        for(unsigned int i = 0; i < msg.sub_term_ptrs.size(); ++i) {
            args[i] = MsgToPrologTerm(full_term_msg.sub_terms[msg.sub_term_ptrs[i]], full_term_msg, str_to_var);
        }

        return PlCompound(msg.functor.c_str(), args);
    }

    ROS_ERROR_STREAM("UNKNOWN TERM MESSAGE: " << msg);
    return PlTerm("UNKNOWN");
}

void prologTermToMsg(const PlTerm& term, const map<string, PlTerm>& str_to_var, tue_reasoner_msgs::TermImpl& msg, tue_reasoner_msgs::Term& full_term_msg) {
    try {
        msg.constant.num = (double)term;
        msg.constant.type = tue_reasoner_msgs::Constant::NUMBER;
        msg.type = tue_reasoner_msgs::TermImpl::CONSTANT;
        return;
    } catch(const PlTypeError& e) {
    }

    try {
    if (term.arity() == 0) {
        // constant (either a number or string)
        msg.type = tue_reasoner_msgs::TermImpl::CONSTANT;

        msg.constant.str = (char*)term;
        msg.constant.type = tue_reasoner_msgs::Constant::STRING;
    } else {
        // compound
        msg.type = tue_reasoner_msgs::TermImpl::COMPOUND;

        msg.functor = term.name();

        for(int i = 0; i < term.arity(); ++i) {
            tue_reasoner_msgs::TermImpl sub_term;
            prologTermToMsg(term[i+1], str_to_var, sub_term, full_term_msg);
            msg.sub_term_ptrs.push_back(full_term_msg.sub_terms.size());
            full_term_msg.sub_terms.push_back(sub_term);
        }
    }
    } catch (const PlTypeError& e) {
        msg.type = tue_reasoner_msgs::TermImpl::VARIABLE;
        msg.variable = (char*)term;
        return;
    }
}

tue_reasoner_msgs::BindingSet prologToBindingSetMsg(const map<string, PlTerm>& str_to_var) {
    tue_reasoner_msgs::BindingSet binding_set;

	for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {

        tue_reasoner_msgs::Binding binding;
        binding.variable = it->first;

		const PlTerm& term = it->second;

        prologTermToMsg(term, str_to_var, binding.value.root, binding.value);

        binding_set.bindings.push_back(binding);
	}

	return binding_set;
}

bool proccessQuery(tue_reasoner_msgs::Query::Request& req, tue_reasoner_msgs::Query::Response& res) {

    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);


    PlTermv av(1);
    map<string, PlTerm> str_to_var;
    av[0] = MsgToPrologTerm(req.term.root, req.term, str_to_var);

    stringstream print_out;    
    print_out << "?- ";
    print_out << (char*)av[0];

    ROS_INFO("%s", print_out.str().c_str());

    try {        
        PlQuery q("call", av);
        while( q.next_solution() ) {
            res.binding_sets.push_back(prologToBindingSetMsg(str_to_var));

            stringstream s_bindings;
            for(map<string, PlTerm>::iterator it_bind = str_to_var.begin(); it_bind != str_to_var.end(); ++it_bind) {
                s_bindings << it_bind->first << " = " << (char*)(it_bind->second) << " ";
            }
            ROS_INFO("   %s", s_bindings.str().c_str());

            // todo PRINT ANSWER!
        }
    } catch ( PlException &ex ) {
        std::cerr << (char *)ex << std::endl;
    }

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
    ROS_INFO("       Reasoner: query took  %f seconds.", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);

	return true;
}

bool proccessAssert(tue_reasoner_msgs::Assert::Request& req, tue_reasoner_msgs::Assert::Response& res) {

    if (req.facts.empty()) {
        ROS_ERROR("Empty received empty query");
        return false;
    }

    string action;
    if (req.action == tue_reasoner_msgs::Assert::Request::ASSERT) {
        action = "assert";
    } else if (req.action == tue_reasoner_msgs::Assert::Request::ASSERTA) {
        action = "asserta";
    } else if (req.action == tue_reasoner_msgs::Assert::Request::ASSERTZ) {
        action = "assertz";
    } else if (req.action == tue_reasoner_msgs::Assert::Request::RETRACT) {
        action = "retractall";
    }

    stringstream errors;

    for(vector<tue_reasoner_msgs::Term>::const_iterator it = req.facts.begin(); it != req.facts.end(); ++it) {
        const tue_reasoner_msgs::Term& term_msg = *it;

        PlTermv av(1);
        map<string, PlTerm> str_to_var;
        av[0] = MsgToPrologTerm(term_msg.root, term_msg, str_to_var);

        ROS_INFO("?- \033[1m%s(%s)\033[0m", action.c_str(), (char*)av[0]);
        PlQuery q(action.c_str(), av);

        try {
            while (q.next_solution()) {

            }
        } catch ( PlException &ex ) {
            errors << (char *)ex << std::endl;
        }
    }

    res.error = errors.str();
    return true;
}

bool loadDatabase(const string& filename) {
    return PlCall("consult", PlTermv(filename.c_str()));
}

bool loadDatabase(tue_reasoner_msgs::LoadDatabase::Request& req, tue_reasoner_msgs::LoadDatabase::Response& resp) {
    if (!loadDatabase(req.db_filename)) {
        resp.result = "Failed to load database: " + req.db_filename;
        ROS_ERROR("%s", resp.result.c_str());
    }
    return true;
}

int main(int argc, char **argv) {

	// Initialize node
	ros::init(argc, argv, "reasoner");
    ros::NodeHandle nh_private("~");

	// Initialize Prolog Engine
    putenv("SWI_HOME_DIR=/usr/lib/swi-prolog");
	//PlEngine prolog_engine(argv[0]);

    PlEngine prolog_engine(argc, argv);

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
            if (!loadDatabase(database_name_str)) {
                ROS_ERROR("Failed to load database: %s", (database_name_str).c_str());
                return -1;
            }
        } else {
            break;
        }
    }

    // create world model
    world_model_ = new mhf::WorldModelROS();
    world_model_->registerEvidenceTopic("/world_evidence");
    world_model_->startThreaded();

    ros::ServiceServer query_service = nh_private.advertiseService("query", proccessQuery);
    ros::ServiceServer assert_service = nh_private.advertiseService("assert", proccessAssert);
    ros::ServiceServer load_db_service = nh_private.advertiseService("load_database", loadDatabase);

    ros::spin();

    delete world_model_;

	return 0;
}

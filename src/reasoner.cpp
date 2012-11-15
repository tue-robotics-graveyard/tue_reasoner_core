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

#include <reasoning_msgs/Query.h>
#include <reasoning_msgs/Assert.h>
#include <reasoning_msgs/LoadDatabase.h>
#include <reasoning_msgs/BindingSet.h>

#include <problib/conversions.h>

#include <world_model/WorldModelROS.h>

#include <vector>
#include <string>

#include "swi-cpp/SWI-cpp.h"

using namespace std;

//map<string, vector<Compound*> > facts_;

mhf::WorldModelROS* world_model_;

PlTerm MsgToPrologTerm(const reasoning_msgs::TermImpl& msg, const reasoning_msgs::Term& full_term_msg, map<string, PlTerm>& str_to_var) {
    if (msg.type == reasoning_msgs::TermImpl::VARIABLE) {
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
    } else if (msg.type == reasoning_msgs::TermImpl::CONSTANT) {
        // term is a constant
        if (msg.constant.type == reasoning_msgs::Constant::STRING) {
            return PlTerm(msg.constant.str.c_str());
        } else if (msg.constant.type == reasoning_msgs::Constant::NUMBER) {
            return PlTerm(msg.constant.num);
        } else if (msg.constant.type == reasoning_msgs::Constant::NUMBER_ARRAY) {
            PlTerm term;
            PlTail list(term);

            for(unsigned int i = 0; i < msg.constant.num_array.size(); ++i) {
                list.append(PlTerm(msg.constant.num_array[i]));
            }
            list.close();
            return term;
        } else if (msg.constant.type == reasoning_msgs::Constant::PDF) {
            ROS_ERROR("CANNOT ADD PDF's TO THE PROLOG DATABASE YET.");
            return PlTerm("PDF");
        }
    } else if (msg.type == reasoning_msgs::TermImpl::COMPOUND) {
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

void prologTermToMsg(const PlTerm& term, const map<string, PlTerm>& str_to_var, reasoning_msgs::TermImpl& msg, reasoning_msgs::Term& full_term_msg) {
    try {
        msg.constant.num = (double)term;
        msg.constant.type = reasoning_msgs::Constant::NUMBER;
        msg.type = reasoning_msgs::TermImpl::CONSTANT;
        return;
    } catch(const PlTypeError& e) {
    }

    if (term.arity() == 0) {
        // constant (either a number or string)
        msg.type = reasoning_msgs::TermImpl::CONSTANT;

        msg.constant.str = (char*)term;
        msg.constant.type = reasoning_msgs::Constant::STRING;
    } else {
        // compound
        msg.type = reasoning_msgs::TermImpl::COMPOUND;

        msg.functor = term.name();

        for(int i = 0; i < term.arity(); ++i) {
            reasoning_msgs::TermImpl sub_term;
            prologTermToMsg(term[i+1], str_to_var, sub_term, full_term_msg);
            msg.sub_term_ptrs.push_back(full_term_msg.sub_terms.size());
            full_term_msg.sub_terms.push_back(sub_term);
        }
    }
}

reasoning_msgs::BindingSet prologToBindingSetMsg(const map<string, PlTerm>& str_to_var) {
    cout << "prologToBindingSetMsg" << endl;

    reasoning_msgs::BindingSet binding_set;

	for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {

        reasoning_msgs::Binding binding;
        binding.variable = it->first;

		const PlTerm& term = it->second;

        prologTermToMsg(term, str_to_var, binding.value.root, binding.value);

        binding_set.bindings.push_back(binding);
	}

    cout << "prologToBindingSetMsg - end" << endl;

	return binding_set;
}

bool proccessQuery(reasoning_msgs::Query::Request& req, reasoning_msgs::Query::Response& res) {

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

bool proccessAssert(reasoning_msgs::Assert::Request& req, reasoning_msgs::Assert::Response& res) {

    if (req.facts.empty()) {
        ROS_ERROR("Empty received empty query");
        return false;
    }

    string action;
    if (req.action == reasoning_msgs::Assert::Request::ASSERT) {
        action = "assert";
    } else if (req.action == reasoning_msgs::Assert::Request::ASSERTA) {
        action = "asserta";
    } else if (req.action == reasoning_msgs::Assert::Request::ASSERTZ) {
        action = "assertz";
    } else if (req.action == reasoning_msgs::Assert::Request::RETRACT) {
        action = "retractall";
    }

    stringstream errors;

    for(vector<reasoning_msgs::Term>::const_iterator it = req.facts.begin(); it != req.facts.end(); ++it) {
        const reasoning_msgs::Term& term_msg = *it;

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

bool loadDatabase(reasoning_msgs::LoadDatabase::Request& req, reasoning_msgs::LoadDatabase::Response& resp) {
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

    string std_db_filename = "";
    nh_private.getParam("std_database", std_db_filename);
    if (!loadDatabase(std_db_filename)) {
        ROS_ERROR("Failed to load database: %s", std_db_filename.c_str());
        return 1;
    }

    string db_filename = "";
    nh_private.getParam("database", db_filename);
    if (!loadDatabase(db_filename)) {
        ROS_ERROR("Failed to load database: %s", db_filename.c_str());
        return 1;
    }

    // create world model
    //world_model_ = new mhf::WorldModelROS();
    //world_model_->startThreaded();

    ros::ServiceServer query_service = nh_private.advertiseService("query", proccessQuery);
    ros::ServiceServer assert_service = nh_private.advertiseService("assert", proccessAssert);
    ros::ServiceServer load_db_service = nh_private.advertiseService("load_database", loadDatabase);

    //world_model_ = new WorldModelROS();
    //world_model_->registerEvidenceTopic("/world_evidence");
    //world_model_->startThreaded();

    ros::spin();

    //delete world_model_;

	return 0;
}

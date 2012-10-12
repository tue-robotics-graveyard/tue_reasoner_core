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

#include <reasoning_srvs/Query.h>
#include <reasoning_srvs/Assert.h>

#include <problib/conversions.h>

#include <WorldModelROS.h>
#include <mht_old/MHTObject.h>
#include <storage/SemanticObject.h>
#include <core/PropertySet.h>
#include <core/Property.h>

#include <vector>
#include <string>

#include "swi-cpp/SWI-cpp.h"

using namespace std;

//map<string, vector<Compound*> > facts_;

//WorldModelROS* world_model_;

PlCompound msgToProlog(const reasoning_msgs::CompoundTerm& msg, map<string, PlTerm>& str_to_var) {
	PlTermv args_prolog(msg.arguments.size());

	for(unsigned int i = 0; i < msg.arguments.size(); ++i) {

		const reasoning_msgs::Argument& arg_msg = msg.arguments[i];

		if (arg_msg.variable != "") {
			// argument is a variable

            map<string, PlTerm>::iterator it_var = str_to_var.find(arg_msg.variable);
			if (it_var != str_to_var.end()) {
				// known variable
				args_prolog[i] = it_var->second;
			} else {
				// unknown variable, so add to map
				str_to_var[arg_msg.variable] = args_prolog[i];
			}
		} else {
			// argument is a value
            if (arg_msg.constant.str != "") {
                args_prolog[i] = arg_msg.constant.str.c_str();
            }
		}
	}
	return PlCompound(msg.predicate.c_str(), args_prolog);
}

reasoning_msgs::BindingSet prologToBindingSetMsg(const map<string, PlTerm>& str_to_var) {
    reasoning_msgs::BindingSet binding_set;

	for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {

        reasoning_msgs::Binding binding;
        binding.variable = it->first;

		const PlTerm& term = it->second;

		if ((string)term.name() == ".") {

			vector<double> vec;

			PlTail list(term);

			PlTerm e;
			while(list.next(e)) {
                binding.value.num_array.push_back((double)e);
			}

		} else {
            binding.value.str = term.name();
		}

        binding_set.bindings.push_back(binding);
	}

	return binding_set;
}

bool proccessQuery(reasoning_srvs::Query::Request& req, reasoning_srvs::Query::Response& res) {

    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);

    if (req.conjuncts.empty()) {
        ROS_ERROR("Empty received empty query");
        return false;
    }

    PlTermv av(1);
    PlTail goal_list(av[0]);

    map<string, PlTerm> str_to_var;
    for(vector<reasoning_msgs::CompoundTerm>::const_iterator it = req.conjuncts.begin(); it != req.conjuncts.end(); ++it) {
        const reasoning_msgs::CompoundTerm& term_msg = *it;

        cout << term_msg.predicate << "/" << term_msg.arguments.size() << endl;

        goal_list.append(msgToProlog(term_msg, str_to_var));
	}

    goal_list.close();

    try {
        PlQuery q("complex_query", av);
        while( q.next_solution() ) {
            res.binding_sets.push_back(prologToBindingSetMsg(str_to_var));
        }
    } catch ( PlException &ex ) {
        std::cerr << (char *)ex << std::endl;
    }

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
    printf("Reasoner: query took  %f seconds.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);

	return true;
}

bool proccessAssert(reasoning_srvs::Assert::Request& req, reasoning_srvs::Assert::Response& res) {

    if (req.facts.empty()) {
        ROS_ERROR("Empty received empty query");
        return false;
    }

    string action;
    if (req.action == reasoning_srvs::Assert::Request::ASSERT) {
        action = "assert";
    } else if (req.action == reasoning_srvs::Assert::Request::ASSERTA) {
        action = "asserta";
    } else if (req.action == reasoning_srvs::Assert::Request::ASSERTZ) {
        action = "assertz";
    } else if (req.action == reasoning_srvs::Assert::Request::RETRACT) {
        action = "retractall";
    }

    cout << action << endl;

    stringstream errors;

    for(vector<reasoning_msgs::CompoundTerm>::const_iterator it = req.facts.begin(); it != req.facts.end(); ++it) {
        const reasoning_msgs::CompoundTerm& term_msg = *it;

        PlTermv av(1);
        map<string, PlTerm> str_to_var;
        av[0] = msgToProlog(term_msg, str_to_var);

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

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "reasoner");
	ros::NodeHandle nh_private("~");

	// Initialize Prolog Engine
    putenv("SWI_HOME_DIR=/usr/lib/swi-prolog");
	//PlEngine prolog_engine(argv[0]);

    PlEngine prolog_engine(argc, argv);

    string std_database = "NONE";
    nh_private.getParam("std_database", std_database);
    PlQuery q_consult("consult", PlTermv(std_database.c_str()));
    try {
    if (!q_consult.next_solution()) {
        ROS_ERROR("Failed to load standard database: %s", std_database.c_str());
        return 0;
    }
    } catch ( PlException &ex ) {
        std::cerr << (char *)ex << std::endl;
        return 0;
    }

	string db_filename = "";
	nh_private.getParam("database_filename", db_filename);
    PlTermv filename_term(db_filename.c_str());

    if (!PlCall("consult", filename_term)) {
        ROS_ERROR("Failed to parse knowledge file: %s", db_filename.c_str());
        return 0;
    }

    /*
	// load the knowledge base into prolog
	PlTermv filename_term(db_filename.c_str());
	PlQuery q_consult("consult", filename_term);
	try {
		if (!q_consult.next_solution()) {
			ROS_ERROR("Failed to parse knowledge file: %s", db_filename.c_str());
			return 0;
		}
	} catch ( PlException &ex ) {
		std::cerr << (char *)ex << std::endl;
		return 0;
	}
    */

    ros::ServiceServer query_service = nh_private.advertiseService("query", proccessQuery);
    ros::ServiceServer assert_service = nh_private.advertiseService("assert", proccessAssert);

    //world_model_ = new WorldModelROS();
    //world_model_->registerEvidenceTopic("/world_evidence");
    //world_model_->startThreaded();

    ros::spin();

    //delete world_model_;

	return 0;
}

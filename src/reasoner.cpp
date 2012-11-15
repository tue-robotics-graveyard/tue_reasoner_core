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

/*
#include <WorldModelROS.h>
#include <mht_old/MHTObject.h>
#include <storage/SemanticObject.h>
#include <core/PropertySet.h>
#include <core/Property.h>
*/

#include <vector>
#include <string>

#include "swi-cpp/SWI-cpp.h"

using namespace std;

//map<string, vector<Compound*> > facts_;

//WorldModelROS* world_model_;

PlCompound msgToProlog(const reasoning_msgs::TermImpl& msg, const reasoning_msgs::Term& full_term_msg, map<string, PlTerm>& str_to_var) {
    PlTermv args_prolog(msg.arguments.size());

    for(unsigned int i = 0; i < msg.arguments.size(); ++i) {
        const reasoning_msgs::Argument& arg_msg = msg.arguments[i];
        if (arg_msg.type == reasoning_msgs::Argument::VARIABLE) {
            // argument is a variable

            map<string, PlTerm>::iterator it_var = str_to_var.find(arg_msg.variable);
            if (it_var != str_to_var.end()) {
                // known variable
                args_prolog[i] = it_var->second;
            } else {
                // unknown variable, so add to map
                str_to_var[arg_msg.variable] = args_prolog[i];
            }
        } else if (arg_msg.type == reasoning_msgs::Argument::CONSTANT) {
            // argument is a value

            if (arg_msg.constant.type == reasoning_msgs::Constant::STRING) {
                args_prolog[i] = arg_msg.constant.str.c_str();
            } else if (arg_msg.constant.type == reasoning_msgs::Constant::NUMBER) {
                args_prolog[i] = arg_msg.constant.num;
            } else if (arg_msg.constant.type == reasoning_msgs::Constant::NUMBER_ARRAY) {
                PlTail list(args_prolog[i]);

                for(unsigned int i = 0; i < arg_msg.constant.num_array.size(); ++i) {
                    list.append(PlTerm(arg_msg.constant.num_array[i]));
                }
                list.close();

            } else if (arg_msg.constant.type == reasoning_msgs::Constant::PDF) {
                cout << "CANNOT ADD PDF's TO THE PROLOG DATABASE YET." << endl;
            }
        } else if (arg_msg.type == reasoning_msgs::Argument::COMPOUND) {
            args_prolog[i] = msgToProlog(full_term_msg.sub_terms[arg_msg.term_ptr], full_term_msg, str_to_var);
        } else {
            ROS_ERROR_STREAM("Received argument with unspecified type:\n" << arg_msg);
        }
    }
    return PlCompound(msg.functor.c_str(), args_prolog);
}

reasoning_msgs::BindingSet prologToBindingSetMsg(const map<string, PlTerm>& str_to_var) {
    reasoning_msgs::BindingSet binding_set;

	for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {

        reasoning_msgs::Binding binding;
        binding.variable = it->first;

		const PlTerm& term = it->second;

		if ((string)term.name() == ".") {
            binding.value.type = reasoning_msgs::Constant::NUMBER_ARRAY;

            vector<double> vec;

			PlTail list(term);

			PlTerm e;
			while(list.next(e)) {
                binding.value.num_array.push_back((double)e);
			}

		} else {
            binding.value.type = reasoning_msgs::Constant::STRING;
            binding.value.str = term.name();
		}

        binding_set.bindings.push_back(binding);
	}

	return binding_set;
}

bool proccessQuery(reasoning_msgs::Query::Request& req, reasoning_msgs::Query::Response& res) {

    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);


    PlTermv av(1);
    map<string, PlTerm> str_to_var;
    av[0] = msgToProlog(req.term.root, req.term, str_to_var);

    stringstream print_out;    
    print_out << "?- ";
    print_out << (char*)av[0];

    /*
    for(vector<reasoning_msgs::CompoundTerm>::const_iterator it = req.conjuncts.begin(); it != req.conjuncts.end(); ++it) {
        const reasoning_msgs::CompoundTerm& term_msg = *it;
        PlTerm term = msgToProlog(term_msg, str_to_var);
        print_out << (char*)term << ", ";
        goal_list.append(term);
    }
    goal_list.close();
    */

    ROS_INFO("%s", print_out.str().c_str());

    try {        
        PlQuery q("call", av);
        while( q.next_solution() ) {
            res.binding_sets.push_back(prologToBindingSetMsg(str_to_var));

            stringstream s_bindings;
            for(map<string, PlTerm>::iterator it_bind = str_to_var.begin(); it_bind != str_to_var.end(); ++it_bind) {
                s_bindings << it_bind->first << " = " << (char*)it_bind->second << " ";
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
        av[0] = msgToProlog(term_msg.root, term_msg, str_to_var);

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

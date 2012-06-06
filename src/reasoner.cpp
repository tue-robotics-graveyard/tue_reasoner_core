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

#include <problib/conversions.h>


#include <vector>
#include <string>

using namespace std;

map<string, vector<Compound*> > facts_;

void match(const Compound& C, vector<BindingSet*>& binding_sets) {
	map<string, vector<Compound*> >::iterator it_fact_set = facts_.find(C.getPredicate());
	if (it_fact_set != facts_.end()) {
		const vector<Compound*>& fact_set = it_fact_set->second;
		for(vector<Compound*>::const_iterator it_fact = fact_set.begin(); it_fact != fact_set.end(); ++it_fact) {
			const Compound& fact = *it_fact;

			if (fact.getArguments().size() == C.getArguments().size()) {

			}

		}
	}
}

Compound* msgToCompound(const reasoning_msgs::Query& msg) {
	Compound* C = new Compound(msg.predicate);

	for(vector<reasoning_msgs::Argument>::const_iterator it = msg.arguments.begin(); it != msg.arguments.end(); ++it) {
		if (it->variable != "") {
			// argument is a variable
			C->addArgument(Variable(it->variable));
		} else {
			// argument is a value
			pbl::PDF* pdf = pbl::msgToPDF(it->value);
			if (pdf) {
				C->addArgument(Value(*pdf));
				delete pdf;
			}
		}
	}

	return C;
}

bool proccessQuery(reasoning_srvs::Query::Request& req, reasoning_srvs::Query::Response& res) {

	if (req.query.conjuncts.empty()) {
		ROS_ERROR("Empty received empty query");
		return false;
	}

	if (req.query.conjuncts.size() > 1 ) {
		ROS_WARN("Reasoner can currently only process single queries (not conjunctions of queries).");
	}

	vector<Compound*> conjuncts;

	for(vector<reasoning_msgs::Query>::const_iterator it = req.query.conjuncts.begin(); it != req.query.conjuncts.end(); ++it) {
		conjuncts.push_back(msgToCompound(*it));
	}

	Compound& C = **conjuncts.begin();

	vector<BindingSet*> binding_sets;

	match(C, binding_sets);

	for(vector<Compound*>::iterator it = conjuncts.begin(); it != conjuncts.end(); ++it) {
		delete *it;
	}

	return true;
}

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "WorldModel");
	ros::NodeHandle nh_private("~");

	string db_filename = "";
	nh_private.getParam("database_filename", db_filename);

	Parser P(db_filename);
	stringstream parse_error;
	if (!P.parse(facts_, parse_error)) {
		ROS_ERROR_STREAM("Error(s) while parsing " << db_filename << ": " << endl << parse_error.str());
	}

	ros::ServiceServer service = nh_private.advertiseService("query", proccessQuery);

	return 0;
}

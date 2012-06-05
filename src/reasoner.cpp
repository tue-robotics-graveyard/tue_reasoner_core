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

#include <reasoning_srvs/Query.h>

#include <problib/conversions.h>


#include <vector>
#include <string>

using namespace std;

void predicateIsInRoom(const vector<Term>& args, const vector<BindingSet*>& binding_sets) {
	if (args.size() != 2) {
		return;
	}

	if (args[0].isValue() && args[1].isVariable()) {
		std::string object_class;
		args[0].getValue()->getExpectedValue(object_class);

		if (object_class == "cup") {
			BindingSet* set = new BindingSet();
			set->addBinding();
		}

	} else {
		ROS_WARN("Predicate 'is-in-room': first argument should be a value, second argument should be a variable.");
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

	if (C.getPredicate() == "is-in-room") {
		predicateIsInRoom(C.getArguments(), binding_sets);
	}

	for(vector<Compound*>::iterator it = conjuncts.begin(); it != conjuncts.end(); ++it) {
		delete *it;
	}

	return true;
}

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "WorldModel");
	ros::NodeHandle nh_private("~");

	ros::ServiceServer service = nh_private.advertiseService("query", proccessQuery);

	return 0;
}

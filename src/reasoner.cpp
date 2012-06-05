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

#include <reasoning_srvs/Query.h>

#include <problib/conversions.h>


#include <vector>
#include <string>

using namespace std;

void predicateIsA(const vector<Term>& args) {
	if (args.size() != 2) {
		return;
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

	if (C.getPredicate() == "is-a") {
		predicateIsA(C.getArguments());
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

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

#include <WorldModelROS.h>

#include <vector>
#include <string>

using namespace std;

map<string, vector<Compound*> > facts_;

WorldModelROS* world_model_;

void predicate_isInstanceOf(const Compound& C, vector<BindingSet*>& binding_sets) {
	const Term& instanceTerm = C.getArgument(0);
	const Term& classTerm = C.getArgument(1);

	if (classTerm.isVariable()) {
		printf("Predicate 'is-instance-of': 2nd argument (class name) must be given.\n");
		return;
	} else {
		PropertySet P;
		P.addProperty("class_label", *classTerm.getValue());

		vector<MHTObject<SemanticObject, Measurement>*> matches;
		vector<double> probabilities;
		world_model_->query(P, matches, probabilities);

		if (instanceTerm.isVariable()) {
			map<int, double> ID_to_prob;
			for(unsigned int i = 0; i < matches.size(); ++i) {
				if (probabilities[i] > 0.0001) {  // todo
					int ID = matches[i]->getUserInfo().ID_;
					map<int, double>::iterator it_ID = ID_to_prob.find(ID);
					if (it_ID == ID_to_prob.end()) {
						ID_to_prob[ID] = probabilities[i];
					} else {
						it_ID->second += probabilities[i];
					}
				}
			}

			for(map<int, double>::iterator it_ID = ID_to_prob.begin(); it_ID != ID_to_prob.end(); ++it_ID) {
				stringstream ss;
				ss << "ID-" << it_ID->first;

				pbl::PMF pmf;
				pmf.setExact(ss.str());

				BindingSet* binding_set = new BindingSet();
				binding_set->addBinding(instanceTerm.getName(), pmf);
				binding_set->setProbability(it_ID->second);
				binding_sets.push_back(binding_set);
			}
		} else {
			string ID_str;
			if (instanceTerm.getValue()->getExpectedValue(ID_str)) {
				int ID = atoi(ID_str.c_str());

				double prob = 0;
				for(unsigned int i = 0; i < matches.size(); ++i) {
					if (ID == matches[i]->getUserInfo().ID_) {
						prob += probabilities[i];
					}
				}

				if (prob > 0) {
					BindingSet* binding_set = new BindingSet();
					binding_set->setProbability(prob);
					binding_sets.push_back(binding_set);
				}
			}
		}
	}

}

void match(const Compound& C, vector<BindingSet*>& binding_sets) {
	cout << "Matching against: " << C.toString() << endl << endl;

	map<string, vector<Compound*> >::iterator it_fact_set = facts_.find(C.getPredicate());

	if (it_fact_set != facts_.end()) {
		const vector<Compound*>& fact_set = it_fact_set->second;
		for(vector<Compound*>::const_iterator it_fact = fact_set.begin(); it_fact != fact_set.end(); ++it_fact) {
			const Compound* fact = *it_fact;

			BindingSet* binding_set = fact->match(C);
			if (binding_set) {
				binding_sets.push_back(binding_set);
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

	vector<BindingSet*> binding_sets;

	Compound& C = **conjuncts.begin();
	if (C.getPredicate() == "is-instance-of" && C.getArguments().size() == 2) {
		predicate_isInstanceOf(C, binding_sets);
	} else {
		match(C, binding_sets);
	}

	for(vector<BindingSet*>::iterator it = binding_sets.begin(); it != binding_sets.end(); ++it) {

		reasoning_msgs::VariableBindingSet binding_set_msg;
		binding_set_msg.probability = (*it)->getProbability();

		const map<string, const pbl::PDF*>& bindings = (*it)->getBindings();
		for(map<string, const pbl::PDF*>::const_iterator it2 = bindings.begin(); it2 != bindings.end(); ++it2) {
			reasoning_msgs::VariableBinding binding_msg;
			binding_msg.variable = it2->first;
			pbl::PDFtoMsg(*(it2->second), binding_msg.value);
			binding_set_msg.bindings.push_back(binding_msg);

		}
		res.response.binding_sets.push_back(binding_set_msg);
	}

	for(vector<Compound*>::iterator it = conjuncts.begin(); it != conjuncts.end(); ++it) {
		delete *it;
	}

	return true;
}

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "reasoner");
	ros::NodeHandle nh_private("~");

	string db_filename = "";
	nh_private.getParam("database_filename", db_filename);

	Parser P(db_filename);
	stringstream parse_error;
	if (!P.parse(facts_, parse_error)) {
		ROS_ERROR_STREAM("Error(s) while parsing " << db_filename << ": " << endl << parse_error.str());
	}

	ros::ServiceServer service = nh_private.advertiseService("query", proccessQuery);

	world_model_ = new WorldModelROS();

	ros::spin();

	delete world_model_;

	return 0;
}

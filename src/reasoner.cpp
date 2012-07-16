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

#include "swi-cpp/SWI-cpp.h"

using namespace std;

map<string, vector<Compound*> > facts_;

WorldModelROS* world_model_;

// map from predicate names to function pointers
map<string, void(*)(const Compound&, vector<BindingSet*>&)>  computable_predicates_;

string IDIntToString(int ID) {
	stringstream ss;
	ss << "id_" << ID;
	return ss.str();
}

int IDStringToInt(const string& ID) {
	if(ID.substr(0, 3) == "id_") {
		return atoi(ID.substr(3).c_str());
	}
	return -1;
}

PREDICATE(hello, 1) {
	cout << "Hello " << (char *)A1 << endl;
	return TRUE;
}

PREDICATE(comp_property, 3) {
	int ID = IDStringToInt((char*)A1);
	PropertySet P(ID);

	string attribute = (char*)A2;

	cout << "Querying for attribute '" << attribute << "' for object with ID " << ID << endl;

	vector<MHTObject*> matches;
	vector<double> probabilities;
	world_model_->query(P, matches, probabilities);

	cout << "Quering done" << endl;

	PlTail results(A3);
	for(unsigned int i = 0; i < matches.size(); ++i) {
		const pbl::PDF& pdf = matches[i]->getObject()->getProperty(attribute)->getPDF();
		results.append(PlCompound("pdf", PlTerm((long)(&pdf))));
	}
	results.close();
	return TRUE;
}

// has-property(ID, ATTRIBUTE, VALUE)
void predicate_hasProperty(const Compound& C, vector<BindingSet*>& binding_sets) {
	const Term& idTerm = C.getArgument(0);
	const Term& attributeTerm = C.getArgument(1);
	const Term& valueTerm = C.getArgument(2);

	if (idTerm.isVariable() || attributeTerm.isVariable() || !valueTerm.isVariable()) {
		printf("Predicate 'has-property': 1st and 2nd argument (instance ID and attribute name) must be given, 3rd argument (value) must be a variable.\n");
		return;
	}

	// determine ID
	string ID_str;
	idTerm.getValue()->getExpectedValue(ID_str);
	int ID = IDStringToInt(ID_str);
	PropertySet P(ID);

	// determine attribute
	string attribute;
	attributeTerm.getValue()->getExpectedValue(attribute);

	cout << "Querying for attribute '" << attribute << "' for object with ID " << ID << endl;

	vector<MHTObject*> matches;
	vector<double> probabilities;
	world_model_->query(P, matches, probabilities);

	cout << "Quering done" << endl;

	for(unsigned int i = 0; i < matches.size(); ++i) {
		const pbl::PDF& pdf = matches[i]->getObject()->getProperty(attribute)->getPDF();
		BindingSet* binding_set = new BindingSet();
		binding_set->addBinding(valueTerm.getName(), pdf);
		binding_set->setProbability(probabilities[i]);
		binding_sets.push_back(binding_set);
	}

}

void predicate_isInstanceAtCoordinates(const Compound& C, vector<BindingSet*>& binding_sets) {
	const Term& instanceTerm = C.getArgument(0);
	const Term& coordinatesTerm = C.getArgument(1);

	if (instanceTerm.isVariable()) {
		printf("Predicate 'is-instance-at-coordinates': 1st argument (instance ID) must be given.\n");
		return;
	}

	if (!coordinatesTerm.isVariable()) {
		printf("Predicate 'is-instance-at-coordinates': 2nd argument (coordinates) must be a variable.\n");
		return;
	}

	string ID_str;
	instanceTerm.getValue()->getExpectedValue(ID_str);
	int ID = IDStringToInt(ID_str);

	if (ID < 0) {
		printf("Predicate 'is-instance-at-coordinates': 1st argument (instance ID) is malformed (should be of format: ID-#).\n");
		return;
	}

	PropertySet P(ID);

	vector<MHTObject*> matches;
	vector<double> probabilities;
	world_model_->query(P, matches, probabilities);

	for(unsigned int i = 0; i < matches.size(); ++i) {
		const pbl::PDF& pdf_pos = matches[i]->getObject()->getProperty("position")->getPDF();
		BindingSet* binding_set = new BindingSet();
		binding_set->addBinding(coordinatesTerm.getName(), pdf_pos);
		binding_set->setProbability(probabilities[i]);
		binding_sets.push_back(binding_set);
	}

}

void predicate_isInstanceAtRoom(const Compound& C, vector<BindingSet*>& binding_sets) {
	ROS_WARN("Predicate is-instance-at-room not yet implemented.");
}

void predicate_isClassAtRoom(const Compound& C, vector<BindingSet*>& binding_sets) {
	ROS_WARN("Predicate is-class-at-room not yet implemented.");
}

void predicate_isInstanceOf(const Compound& C, vector<BindingSet*>& binding_sets) {
	const Term& instanceTerm = C.getArgument(0);
	const Term& classTerm = C.getArgument(1);

	if (classTerm.isVariable()) {
		printf("Predicate 'is-instance-of': 2nd argument (class name) must be given.\n");
		return;
	}

	PropertySet P;
	P.addProperty("class_label", *classTerm.getValue());

	vector<MHTObject*> matches;
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
			pbl::PMF pmf;
			pmf.setExact(IDIntToString(it_ID->first));

			BindingSet* binding_set = new BindingSet();
			binding_set->addBinding(instanceTerm.getName(), pmf);
			binding_set->setProbability(it_ID->second);
			binding_sets.push_back(binding_set);
		}
	} else {
		string ID_str;
		if (instanceTerm.getValue()->getExpectedValue(ID_str)) {
			int ID = IDStringToInt(ID_str);

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

Compound msgToCompound(const reasoning_msgs::Query& msg) {
	Compound C(msg.predicate);

	for(vector<reasoning_msgs::Argument>::const_iterator it = msg.arguments.begin(); it != msg.arguments.end(); ++it) {
		if (it->variable != "") {
			// argument is a variable
			C.addArgument(Variable(it->variable));
		} else {
			// argument is a value
			pbl::PDF* pdf = pbl::msgToPDF(it->value);
			if (pdf) {
				C.addArgument(Value(*pdf));
				delete pdf;
			}
		}
	}

	return C;
}

PlCompound msgToProlog(const reasoning_msgs::Query& msg, map<string, PlTerm>& str_to_var) {
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
			if (arg_msg.value.exact_value_str != "") {
				args_prolog[i] = arg_msg.value.exact_value_str.c_str();
			} else {
				pbl::PDF* pdf = pbl::msgToPDF(arg_msg.value);
				args_prolog[i] = PlCompound("pdf", PlTerm(pdf));
			}
		}
	}
	return PlCompound(msg.predicate.c_str(), args_prolog);
}

BindingSet* prologToBindingSet(const map<string, PlTerm>& str_to_var) {
	BindingSet* binding_set = new BindingSet();

	binding_set->setProbability(1.0);
	for(map<string, PlTerm>::const_iterator it = str_to_var.begin(); it != str_to_var.end(); ++it) {
		pbl::PMF pmf;
		pmf.setExact((char *)it->second);
		binding_set->addBinding(it->first, pmf);
	}

	return binding_set;
}

bool proccessQuery(reasoning_srvs::Query::Request& req, reasoning_srvs::Query::Response& res) {

	if (req.query.conjuncts.empty()) {
		ROS_ERROR("Empty received empty query");
		return false;
	}

	if (req.query.conjuncts.size() > 1 ) {
		ROS_WARN("Reasoner can currently only process single queries (not conjunctions of queries).");
	}

	vector<BindingSet*> binding_sets;

	for(vector<reasoning_msgs::Query>::const_iterator it = req.query.conjuncts.begin(); it != req.query.conjuncts.end(); ++it) {
		const reasoning_msgs::Query& term_msg = *it;

		stringstream predicate_ss;
		predicate_ss << term_msg.predicate << "/" << term_msg.arguments.size();

		map<string, void(*)(const Compound&, vector<BindingSet*>&)>::iterator it_computable = computable_predicates_.find(predicate_ss.str());
		if (it_computable != computable_predicates_.end()) {
			it_computable->second(msgToCompound(term_msg), binding_sets);
		} else {
			PlTermv av(1);

			map<string, PlTerm> str_to_var;
			PlTail goal_list(av[0]);
			goal_list.append(msgToProlog(*it, str_to_var));
			goal_list.close();

			try {
				PlQuery q("complex_query", av);
				while( q.next_solution() ) {
					binding_sets.push_back(prologToBindingSet(str_to_var));
				}
			} catch ( PlException &ex ) {
				std::cerr << (char *)ex << std::endl;
			}
		}

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

	string db_filename = "";
	nh_private.getParam("database_filename", db_filename);

	/*
	Parser P(db_filename);
	stringstream parse_error;
	if (!P.parse(facts_, parse_error)) {
		ROS_ERROR_STREAM("Error(s) while parsing " << db_filename << ": " << endl << parse_error.str());
	}
	 */

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

	computable_predicates_["is_instance_of/2"] = &predicate_isInstanceOf;
	computable_predicates_["is_instance_at_coordinates/2"] = &predicate_isInstanceAtCoordinates;
	computable_predicates_["is_instance_at_room/2"] = &predicate_isInstanceAtRoom;
	computable_predicates_["is_class_at_room/2"] = &predicate_isClassAtRoom;
	//computable_predicates_["has_property/3"] = &predicate_hasProperty;

	ros::ServiceServer service = nh_private.advertiseService("query", proccessQuery);

	world_model_ = new WorldModelROS();
	world_model_->registerEvidenceTopic("/world_evidence");

	ros::spin();

	delete world_model_;

	return 0;
}

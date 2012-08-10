/*
 * test_reasoner.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "reasoning_srvs/Query.h"

#include <problib/conversions.h>

using namespace std;

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

	ros::ServiceClient client = nh_private.serviceClient<reasoning_srvs::Query>("/reasoner/query");

	client.waitForExistence();

	reasoning_msgs::Query query;
	reasoning_msgs::Argument arg1;
	reasoning_msgs::Argument arg2;
	reasoning_msgs::Argument arg3;

	/*
	query.predicate = "has-property";
	arg1.value.exact_value_str = "ID-2";
	arg2.value.exact_value_str = "class_label";
	arg3.variable = "X";
*/

/*
	query.predicate = "is-instance-at-coordinates";
	arg1.value.exact_value_str = "ID-2";
	arg2.variable = "POS";
*/

	/*
	query.predicate = "is-class-at-coordinates";
	arg1.value.exact_value_str = "coke";
	arg2.variable = "POS";
	*/

/*
	query.predicate = "is-instance-of";
	arg1.variable = "X";
	arg2.value.exact_value_str = "cup";
*/

	/*
	query.predicate = "has_property";
	arg1.value.exact_value_str = "id_1";
	arg2.value.exact_value_str = "position";
	arg3.variable = "X";
	*/

	/*
	query.predicate = "owl_subclass_of";
	arg1.variable = "A";
	//arg2.variable = "B";
	arg2.value.exact_value_str = "http://ias.cs.tum.edu/kb/knowrob.owl#FoodOrDrink";
	 */
	//visualisation_canvas(C).
	query.predicate = "is_instance_of";
	arg1.variable = "A";
	arg2.variable = "B";

	query.arguments.push_back(arg1);
	query.arguments.push_back(arg2);
	//query.arguments.push_back(arg3);

	reasoning_srvs::Query srv;
	srv.request.query.conjuncts.push_back(query);


    ros::Time current_time = ros::Time::now();

	if (client.call(srv)) {

        cout << "test_reasoner: query took " << (ros::Time::now().toSec() - current_time.toSec()) << "seconds" << endl;

		if (!srv.response.response.binding_sets.empty()) {
			for(vector<reasoning_msgs::VariableBindingSet>::const_iterator it = srv.response.response.binding_sets.begin();
					it != srv.response.response.binding_sets.end(); ++it) {

				if (it->bindings.empty()) {
					cout << "Yes (probability = " << it->probability << ")." << endl << endl;
				} else {
					cout << "=== probability: " << it->probability << " === " << endl;
					for(vector<reasoning_msgs::VariableBinding>::const_iterator it_b = it->bindings.begin(); it_b != it->bindings.end(); ++it_b) {
						pbl::PDF* pdf = pbl::msgToPDF(it_b->value);
						cout << it_b->variable << " = " << pdf->toString() << endl;
						delete pdf;
					}
					cout << endl;
				}
			}
		} else {
			cout << "No." << endl << endl;
		}

	} else {
		ROS_ERROR("Failed to call service /reasoner/query");
		return 1;
	}

}

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

reasoning_msgs::Argument varArgument(const string& var) {
    reasoning_msgs::Argument arg;
    arg.variable = var;
    return arg;
}

reasoning_msgs::Argument constArgument(const string& str) {
    reasoning_msgs::Argument arg;
    arg.constant.str = str;
    return arg;
}

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

	ros::ServiceClient client = nh_private.serviceClient<reasoning_srvs::Query>("/reasoner/query");

	client.waitForExistence();

    reasoning_srvs::Query query;

    reasoning_msgs::CompoundTerm term1;
    term1.predicate = "type";
    term1.arguments.push_back(varArgument("X"));
    term1.arguments.push_back(constArgument("drink"));

    query.request.conjuncts.push_back(term1);

    ros::Time current_time = ros::Time::now();

    if (client.call(query)) {

        cout << "test_reasoner: query took " << (ros::Time::now().toSec() - current_time.toSec()) << "seconds" << endl;

        if (!query.response.binding_sets.empty()) {
            for(vector<reasoning_msgs::BindingSet>::const_iterator it = query.response.binding_sets.begin();
                    it != query.response.binding_sets.end(); ++it) {

                const reasoning_msgs::BindingSet& binding_set = *it;

                if (binding_set.bindings.empty()) {
                    cout << "Yes" << endl;
				} else {
                    for(vector<reasoning_msgs::Binding>::const_iterator it_b = binding_set.bindings.begin(); it_b != binding_set.bindings.end(); ++it_b) {
                        const reasoning_msgs::Binding& binding = *it_b;
                        cout << binding.variable << " = " << binding.value.str << "\t" << endl;
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

/*
 * test_reasoner.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "reasoning_srvs/Query.h"

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

	ros::ServiceClient client = nh_private.serviceClient<reasoning_srvs::Query>("/reasoner/query");

	client.waitForExistence();

	reasoning_msgs::Query query;
	query.predicate = "is-a";

	reasoning_msgs::Argument arg1;
	arg1.variable = "X;";
	query.arguments.push_back(arg1);

	reasoning_msgs::Argument arg2;
	arg2.variable = "Y;";
	//arg2.value.exact_value_str = "animal";
	query.arguments.push_back(arg2);

	// query: i-as(X, animal)

	reasoning_srvs::Query srv;
	srv.request.query.conjuncts.push_back(query);

	if (client.call(srv)) {
		std::cout << srv.response.response << std::endl;
	} else {
		ROS_ERROR("Failed to call service /reasoner/query");
		return 1;
	}

}

/*
 * test_reasoner.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "reasoning_srvs/Query.h"
#include "reasoning_srvs/Assert.h"
#include "reasoning_srvs/LoadDatabase.h"

#include <problib/conversions.h>

using namespace std;

ros::ServiceClient assert_client;
ros::ServiceClient query_client;

reasoning_msgs::Argument varArgument(const string& var) {
    reasoning_msgs::Argument arg;
    arg.variable = var;
    return arg;
}

reasoning_msgs::Argument constArgument(const string& str) {
    reasoning_msgs::Argument arg;
    arg.constant.type = reasoning_msgs::Constant::STRING;
    arg.constant.str = str;
    return arg;
}

reasoning_msgs::Argument constArgument(const vector<double>& vec) {
    reasoning_msgs::Argument arg;
    arg.constant.type = reasoning_msgs::Constant::NUMBER_ARRAY;
    arg.constant.num_array = vec;
    return arg;
}

reasoning_msgs::CompoundTerm compoundTerm(const string& pred, const reasoning_msgs::Argument& arg1, const reasoning_msgs::Argument& arg2) {
    reasoning_msgs::CompoundTerm term;
    term.predicate = pred;
    term.arguments.push_back(arg1);
    term.arguments.push_back(arg2);
    return term;
}

bool assertKnowledge(const reasoning_srvs::Assert::Request& req) {
    reasoning_srvs::Assert::Response resp;
    if (assert_client.call(req, resp)) {
        if (resp.error == "") {
            cout << "Assert succesfull" << endl;
        } else {
            cout << "Assert service: " << resp.error << endl;
        }
    } else {
        ROS_ERROR("Failed to call service %s", assert_client.getService().c_str());
        return false;
    }
    return true;
}

bool queryKnowledge(const reasoning_srvs::Query::Request& req) {
    ros::Time current_time = ros::Time::now();

    reasoning_srvs::Query::Response resp;
    if (query_client.call(req, resp)) {

        cout << "test_reasoner: query took " << (ros::Time::now().toSec() - current_time.toSec()) << "seconds" << endl;

        if (!resp.binding_sets.empty()) {
            for(vector<reasoning_msgs::BindingSet>::const_iterator it = resp.binding_sets.begin(); it != resp.binding_sets.end(); ++it) {

                const reasoning_msgs::BindingSet& binding_set = *it;

                if (binding_set.bindings.empty()) {
                    cout << "Yes" << endl;
                } else {
                    for(vector<reasoning_msgs::Binding>::const_iterator it_b = binding_set.bindings.begin(); it_b != binding_set.bindings.end(); ++it_b) {
                        const reasoning_msgs::Binding& binding = *it_b;
                        cout << binding.variable << " = ";

                        if (binding.value.type == reasoning_msgs::Constant::STRING) {
                            cout << binding.value.str;
                        } else if (binding.value.type == reasoning_msgs::Constant::NUMBER) {
                            cout << binding.value.num;
                        } else if (binding.value.type == reasoning_msgs::Constant::NUMBER_ARRAY) {
                            cout << "[";
                            for(unsigned int i = 0; i < binding.value.num_array.size(); ++i) {
                                cout << " " << binding.value.num_array[i];
                            }
                            cout << "]";
                        } else if (binding.value.type == reasoning_msgs::Constant::PDF) {
                            pbl::PDF* pdf = pbl::msgToPDF(binding.value.pdf);
                            if (pdf) {
                                cout << pdf->toString();
                            } else {
                                cout << "INVALID PDF";
                            }
                        }
                        cout << "\t";
                    }
                    cout << endl;
                }
            }
        } else {
            cout << "No." << endl << endl;
        }

    } else {
        ROS_ERROR("Failed to call service %s", query_client.getService().c_str());
        return false;
    }
    return true;
}

int main(int argc, char **argv) {

    if (argc <= 1) {
        cout << "Please specify database filename" << endl;
        return 1;
    }

    // Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

    /* * * * * * * * * INIT CLIENTS * * * * * * * * */

    ros::ServiceClient load_db_client = nh_private.serviceClient<reasoning_srvs::LoadDatabase>("/reasoner/load_database");
    load_db_client.waitForExistence();

    assert_client = nh_private.serviceClient<reasoning_srvs::Assert>("/reasoner/assert");
    assert_client.waitForExistence();

    query_client = nh_private.serviceClient<reasoning_srvs::Query>("/reasoner/query");
    query_client.waitForExistence();

    /* * * * * * * * * LOAD DATABASE * * * * * * * * */

    reasoning_srvs::LoadDatabase load_db;
    load_db.request.db_filename = argv[1];
    load_db_client.call(load_db);

    /* * * * * * * * * TEST * * * * * * * * */

    reasoning_srvs::Query::Request query1;
    query1.conjuncts.push_back(compoundTerm("is_class_at_coordinates", constArgument("Exit"), varArgument("Y")));
    queryKnowledge(query1);

    reasoning_srvs::Assert::Request assert;
    assert.facts.push_back(compoundTerm("type", constArgument("milk"), constArgument("drink")));
    assertKnowledge(assert);

    reasoning_srvs::Assert::Request assert2;
    vector<double> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    assert2.facts.push_back(compoundTerm("type", constArgument("test"), constArgument(vec)));
    assertKnowledge(assert2);

    cout << "- - - - - - - - - - - - - - - - - - " << endl;

    reasoning_srvs::Query::Request query;
    query.conjuncts.push_back(compoundTerm("type", varArgument("X"), varArgument("Y")));
    //query.conjuncts.push_back(compoundTerm("location", varArgument("X"), constArgument("living_room")));
    queryKnowledge(query);

    reasoning_srvs::Assert::Request retract;
    retract.action = reasoning_srvs::Assert::Request::RETRACT;
    retract.facts.push_back(compoundTerm("type", constArgument("coke"), varArgument("Y")));
    assertKnowledge(retract);

    cout << "- - - - - - - - - - - - - - - - - - " << endl;

    queryKnowledge(query);
}

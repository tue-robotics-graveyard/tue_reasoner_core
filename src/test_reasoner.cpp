/*
 * test_reasoner.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "reasoning_msgs/Query.h"
#include "reasoning_msgs/Assert.h"
#include "reasoning_msgs/LoadDatabase.h"

#include <problib/conversions.h>

using namespace std;

ros::ServiceClient assert_client;
ros::ServiceClient query_client;

reasoning_msgs::Term variable(const string& var) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::VARIABLE;
    term.root.variable = var;
    return term;
}

reasoning_msgs::Term constant(const string& str) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::CONSTANT;
    term.root.constant.type = reasoning_msgs::Constant::STRING;
    term.root.constant.str = str;
    return term;
}

reasoning_msgs::Term constant(const vector<double>& vec) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::CONSTANT;
    term.root.constant.type = reasoning_msgs::Constant::NUMBER_ARRAY;
    term.root.constant.num_array = vec;
    return term;
}

reasoning_msgs::Term compound(const string& functor) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::COMPOUND;
    term.root.functor = functor;
    return term;
}

reasoning_msgs::Term compound(const string& functor, const reasoning_msgs::Term& arg1) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::COMPOUND;
    term.root.functor = functor;
    term.sub_terms.push_back(arg1.root);   // only works for non-compound arg1
    term.root.sub_term_ptrs.push_back(0);
    return term;
}

reasoning_msgs::Term compound(const string& functor, const reasoning_msgs::Term& arg1, const reasoning_msgs::Term& arg2) {
    reasoning_msgs::Term term;
    term.root.type = reasoning_msgs::TermImpl::COMPOUND;
    term.root.functor = functor;
    term.sub_terms.push_back(arg1.root);   // only works for non-compound arg1
    term.sub_terms.push_back(arg2.root);   // only works for non-compound arg2
    term.root.sub_term_ptrs.push_back(0);
    term.root.sub_term_ptrs.push_back(1);
    return term;
}

std::string toString(const reasoning_msgs::TermImpl& msg, const reasoning_msgs::Term& full_term_msg) {
    if (msg.type == reasoning_msgs::TermImpl::VARIABLE) {
        return msg.variable;
    } else if (msg.type == reasoning_msgs::TermImpl::CONSTANT) {
        if (msg.constant.type == reasoning_msgs::Constant::STRING) {
            return "'" + msg.constant.str + "'";
        } else if (msg.constant.type == reasoning_msgs::Constant::NUMBER) {
            stringstream ss;
            ss << msg.constant.num;
            return ss.str();
        } else if (msg.constant.type == reasoning_msgs::Constant::NUMBER_ARRAY) {
            stringstream ss;
            ss << "[";
            for(unsigned int i = 0; i < msg.constant.num_array.size(); ++i) {
                ss << " " << msg.constant.num_array[i];
            }
            ss << "]";
            return ss.str();
        }  else if (msg.constant.type == reasoning_msgs::Constant::PDF) {
            pbl::PDF* pdf = pbl::msgToPDF(msg.constant.pdf);
            if (pdf) {
                return pdf->toString();
            } else {
                return "INVALID_PDF";
            }
        }
    } else if (msg.type == reasoning_msgs::TermImpl::COMPOUND) {
        stringstream ss;
        ss << msg.functor << "(";
        for(unsigned int i = 0; i < msg.sub_term_ptrs.size(); ++i) {
            ss << toString(full_term_msg.sub_terms[msg.sub_term_ptrs[i]], full_term_msg);
            if (i + 1 < msg.sub_term_ptrs.size()) {
                ss << ", ";
            }
        }
        ss << ")";
        return ss.str();
    }

    return "_UNKNOWN_";
}

std::string toString(const reasoning_msgs::Term& term_msg) {
    stringstream ss;
    ss << term_msg;
    //return ss.str();
    return toString(term_msg.root, term_msg);
}

bool assertKnowledge(const reasoning_msgs::Assert::Request& req) {
    reasoning_msgs::Assert::Response resp;
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

bool queryKnowledge(const reasoning_msgs::Query::Request& req) {
    ros::Time current_time = ros::Time::now();

    reasoning_msgs::Query::Response resp;
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
                        cout << binding.variable << " = " << toString(binding.value);
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

    // Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

    /* * * * * * * * * INIT CLIENTS * * * * * * * * */

    ros::ServiceClient load_db_client = nh_private.serviceClient<reasoning_msgs::LoadDatabase>("/reasoner/load_database");
    load_db_client.waitForExistence();

    assert_client = nh_private.serviceClient<reasoning_msgs::Assert>("/reasoner/assert");
    assert_client.waitForExistence();

    query_client = nh_private.serviceClient<reasoning_msgs::Query>("/reasoner/query");
    query_client.waitForExistence();

    /* * * * * * * * * LOAD DATABASE * * * * * * * * */

    /*
    if (argc <= 1) {
        cout << "Please specify database filename" << endl;
        return 1;
    }

    reasoning_msgs::LoadDatabase load_db;
    load_db.request.db_filename = argv[1];
    load_db_client.call(load_db);
    */

    /* * * * * * * * * TEST * * * * * * * * */

    reasoning_msgs::Query::Request query1;
    query1.term = compound("type", variable("X"), variable("P"));
    queryKnowledge(query1);

    /*
    reasoning_msgs::Query::Request query1;
    query1.term = compound("is_class_at_coordinates", constant("exit"), variable("Y"));
    queryKnowledge(query1);
    */

    return 0;

    reasoning_msgs::Assert::Request assert;
    assert.facts.push_back(compound("type", constant("milk"), constant("drink")));
    assertKnowledge(assert);

    reasoning_msgs::Assert::Request assert2;
    vector<double> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    assert2.facts.push_back(compound("type", constant("test"), constant(vec)));
    assertKnowledge(assert2);

    cout << "- - - - - - - - - - - - - - - - - - " << endl;

    reasoning_msgs::Query::Request query;
    query.term = compound("type", variable("X"), variable("Y"));
    //query.conjuncts.push_back(compoundTerm("location", varArgument("X"), constArgument("living_room")));
    queryKnowledge(query);

    reasoning_msgs::Assert::Request retract;
    retract.action = reasoning_msgs::Assert::Request::RETRACT;
    retract.facts.push_back(compound("type", constant("coke"), variable("Y")));
    assertKnowledge(retract);

    cout << "- - - - - - - - - - - - - - - - - - " << endl;

    queryKnowledge(query);
}

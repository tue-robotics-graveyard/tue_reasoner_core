/*
 * test_reasoner.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: sdries
 */

#include <ros/ros.h>

#include "reasoning_msgs/LoadDatabase.h"

#include "reasoner_interface/interface.h"

using namespace std;


int main(int argc, char **argv) {

    // Initialize node
	ros::init(argc, argv, "ReasonerTest");
	ros::NodeHandle nh_private("~");

    /* * * * * * * * * INIT CLIENTS * * * * * * * * */

    ros::ServiceClient load_db_client = nh_private.serviceClient<reasoning_msgs::LoadDatabase>("/reasoner/load_database");
    load_db_client.waitForExistence();

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

    interface::query(Compound("position", Variable("X"), Variable("P")));

    interface::query(Compound("is_class_at_coordinates", Constant("exit"), Variable("P")));

    interface::assertFact(Compound("type", Constant("milk"), Constant("drink")));

/*
    vector<double> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    interface::assertFact(Compound("type", Constant("test"), Constant(vec)));
*/
    cout << "- - - - - - - - - - - - - - - - - - " << endl;

    interface::retract(Compound("type", Constant("coke"), Variable("Y")));

    cout << "- - - - - - - - - - - - - - - - - - " << endl;
}

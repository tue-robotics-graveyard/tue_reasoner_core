#include "reasoner/ReasonerServer.h"

#include <psi/conversions.h>

#include <ros/package.h>

using namespace std;

ReasonerServer* REASONER;

PREDICATE(property_list, 4) {
    return REASONER->pred_property_list(A1, A2, A3, A4);
}

PREDICATE(property_expected_list, 4) {
    return REASONER->pred_property_expected_list(A1, A2, A3, A4);
}

PREDICATE(lookup_transform, 3) {
    return REASONER->pred_lookup_transform(A1, A2, A3);
}

PREDICATE(transform_point, 4) {
    return REASONER->pred_transform_point(A1, A2, A3, A4);
}

PREDICATE(quaternion_to_rpy, 2) {
    return REASONER->pred_quaternion_to_rpy(A1, A2);
}

PREDICATE(get_ros_package_path, 2) {
    string package = (char*)A1;
    std::string path = ros::package::getPath(package);
    if (path != "") {
        A2 = PlAtom(path.c_str());
        return true;
    }
    return false;
}

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "reasoner");
    ros::NodeHandle nh_private("~");

    // Initialize Prolog Engine
    putenv((char*)"SWI_HOME_DIR=/usr/lib/swi-prolog");
    //PlEngine prolog_engine(argv[0]);

    PlEngine prolog_engine(argc, argv);

    // Determine world model type (ed or wire)
    std::string wm_type;
    if (!nh_private.getParam("world_model_type", wm_type))
        wm_type = "wire"; // Default

    if (wm_type != "wire" && wm_type != "ed")
    {
        ROS_ERROR("Unknown world model type: %s", wm_type.c_str());
        return 1;
    }

    REASONER = new ReasonerServer("/reasoner", wm_type);

    for(int i_database = 1; true; ++i_database) {
        stringstream s_param;
        s_param << "database" << i_database;

        XmlRpc::XmlRpcValue database_name;
        if (nh_private.getParam(s_param.str(), database_name)) {
            // delete the parameter (otherwise it is maintained, even if the reasoner is stopped - and
            // will be loaded again next time the reasoner is started)
            nh_private.deleteParam(s_param.str());

            if (database_name.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("Each database name must be a string.");
                return -1;
            }

            string database_name_str = (string)database_name;
            if (!REASONER->loadDatabase(database_name_str)) {
                ROS_ERROR("Failed to load database: %s", (database_name_str).c_str());
                return -1;
            }
        } else {
            break;
        }
    }

    string fact_str;
    nh_private.getParam("assert", fact_str);
    psi::Term fact = psi::stringToTerm(fact_str);
    if (fact.isValid()) {
        vector<psi::Term> facts;
        facts.push_back(fact);
        REASONER->processAssert(facts);
    }

    REASONER->start();

    delete REASONER;

    return 0;
}

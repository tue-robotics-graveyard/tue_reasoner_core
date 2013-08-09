#include <reasoner/Reasoner.h>

using namespace std;

int main(int argc, char **argv) {

    Reasoner r;

//    // Initialize node
//	ros::init(argc, argv, "ReasonerTest");
//	ros::NodeHandle nh_private("~");

//    /* * * * * * * * * LOAD DATABASE * * * * * * * * */

//    /*
//    ros::ServiceClient load_db_client = nh_private.serviceClient<tue_reasoner_msgs::LoadDatabase>("/reasoner/load_database");
//    load_db_client.waitForExistence();

//    if (argc <= 1) {
//        cout << "Please specify database filename" << endl;
//        return 1;
//    }

//    tue_reasoner_msgs::LoadDatabase load_db;
//    load_db.request.db_filename = argv[1];
//    load_db_client.call(load_db);
//    */

//    /* * * * * * * * * TEST * * * * * * * * */

//    psi::Client client("/reasoner");

//    /*
//    vector<BindingSet> result =
//            client.query(Compound(",",
//                          Compound("position", Variable("X"), Variable("P")),
//                          Compound("type", Variable("X"), Variable("Y"))
//                         )
//                 );
//    */

//    vector<BindingSet> result = client.query(Compound("mirror", Constant("a"), Variable("X")));

//    if (result.empty()) {
//        cout << "No." << endl;
//    } else {
//        for(unsigned int i = 0; i < result.size(); ++i) {
//            cout << result[i].toString() << endl;
//        }
//    }

//    /*

//    interface::query(Compound("is_class_at_coordinates", Constant("exit"), Variable("P")));

//    interface::assertFact(Compound("type", Constant("milk"), Constant("drink")));

//    interface::retract(Compound("type", Constant("coke"), Variable("Y")));
//    */

}

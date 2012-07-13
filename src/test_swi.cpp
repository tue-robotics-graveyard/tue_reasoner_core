
#include "swi-cpp/SWI-cpp.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

	putenv("SWI_HOME_DIR=/usr/lib/swi-prolog");
	PlEngine e(argv[0]);

	PlTermv file(PlAtom("/home/sdries/ros/tue/dev/sjoerd/tue_reasoner/3rdparty/swi-cpp/reasoner.pl"));
	PlQuery q_consult("consult", file);
	try {
		if (!q_consult.next_solution()) return 0;
	} catch ( PlException &ex ) {
		std::cerr << (char *)ex << std::endl;
	}

	// (likes(john, X), person(X)).
	PlTermv v1(2);
	v1[0] = "john";
	PlCompound likes("likes", v1);

	PlTermv v2(1);
	v2[0] = v1[1];
	PlCompound person("person", v2);

	PlTermv v3(1);

	PlTail l(v3[0]);
	l.append(likes);
	l.append(person);
	l.close();

	try {
		PlQuery q("complex_query", v3);
		while( q.next_solution() ) {
			cout << "\t" << (char *)v1[1] << endl;
		}
	} catch ( PlException &ex ) {
		std::cerr << (char *)ex << std::endl;
	}


}

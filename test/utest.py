#! /usr/bin/env python
import roslib; roslib.load_manifest('psi')
from psi import *

if __name__ == "__main__":
    rospy.init_node('tue_reasoner_test_client_py')

    client = Client("/reasoner")

    print "Asserting foo(bar)"
    client.query(Compound("assert", Compound("foo", "bar")))

    print "?- not(foo(bar)) - must be False"
    if client.query(Compound("not", Compound("foo", "bar"))):
        print "True"
    else:
        print "False"

    print "?- not(foo(bla)) - must be True"
    if client.query(Compound("not", Compound("foo", "bla"))):
        print "True"
    else:
        print "False"

    print "?- not(foo(X)) - must be False"
    if client.query(Compound("not", Compound("foo", "X"))):
        print "True"
    else:
        print "False"	

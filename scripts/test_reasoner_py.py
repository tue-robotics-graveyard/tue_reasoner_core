#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import threading
import reasoning_msgs.msg
import reasoning_msgs.srv

###########################################################################
#                                                                         # 
#                                  TERM                                   #
#                                                                         #
###########################################################################

class Term(object):

    def __repr__(self):
        return "UNKNOWN TERM" 

    def is_variable(self):
        return False       

    def is_constant(self):
        return False       

    def is_compound(self):
        return False       

###########################################################################
#                                                                         # 
#                                COMPOUND                                 #
#                                                                         #
###########################################################################

class Compound(Term):
    def __init__(self, functor, *args):
        Term.__init__(self)
        self.functor = functor
        self.arguments = []        
        for arg in args:
            self.add_argument(arg)

    def add_argument(self, arg):
        if isinstance(arg, str):
            if arg[0].isupper():        
                self.arguments.append(Variable(arg))
            else:
                self.arguments.append(Constant(arg))                
        else:
            self.arguments.append(arg)

    def get_arity(self):
        return len(self.arguments)

    def __getitem__(self, index):
        return self.arguments[index]

    def __repr__(self):
        s = self.functor
        if len(self.arguments) > 0:
            s += "("
            for i in range(len(self.arguments)):
                s+= str(self.arguments[i])
                if i + 1 < len(self.arguments):
                    s += ", "
            s += ")"
        return s

    def is_compound(self):
        return True  

###########################################################################
#                                                                         # 
#                                VARIABLE                                 #
#                                                                         #
###########################################################################

class Variable(Term):
    def __init__(self, var_name):
        Term.__init__(self)
        self.var_name = var_name

    def __repr__(self):
        return self.var_name.title()  # title() makes first letter capital

    def is_variable(self):
        return True  

###########################################################################
#                                                                         # 
#                                CONSTANT                                 #
#                                                                         #
###########################################################################

class Constant(Term):
    def __init__(self, string):
        Term.__init__(self)
        self.string = string

    def __repr__(self):
        return self.string

    def is_constant(self):
        return True

    def is_number(self):
        f = float(self.string)
        return True

    def __float__(self):
        return float(self.string)

###########################################################################
#                                                                         # 
#                         CONJUNCTION AND DISJUNCTION                     #
#                                                                         #
###########################################################################

class Conjunction(Compound):
    def __init__(self, arg1, *args):
        if len(args) == 0:
            Compound.__init__(self, ",", arg1)                    
        elif len(args) == 1:
            Compound.__init__(self, ",", arg1, args[0])
        else:
            Compound.__init__(self, ",", arg1, Conjunction(*args))

class Disjunction(Compound):
    def __init__(self, arg1, *args):
        if len(args) == 0:
            Compound.__init__(self, ";", arg1)                    
        elif len(args) == 1:
            Compound.__init__(self, ";", arg1, args[0])
        else:
            Compound.__init__(self, ";", arg1, Conjunction(*args))

###########################################################################
#                                                                         # 
#                         ROS CONVERSION METHODS                          #
#                                                                         #
###########################################################################

# returns type reasoning_msgs.msg.Term()
def term_to_msg(term):
    full_term_msg = reasoning_msgs.msg.Term()  
    full_term_msg.root = term_to_msg2(term, full_term_msg)
    return full_term_msg

# returns type reasoning_msgs.msg.TermImpl()
def term_to_msg2(term, full_term_msg):
    term_msg = reasoning_msgs.msg.TermImpl()

    if term.is_variable():
        term_msg.type = reasoning_msgs.msg.TermImpl.VARIABLE
        term_msg.variable = term.var_name
        return term_msg
    elif term.is_constant():
        term_msg.type = reasoning_msgs.msg.TermImpl.CONSTANT
        term_msg.constant.type = reasoning_msgs.msg.Constant.STRING
        term_msg.constant.str = term.string
    elif term.is_compound():
        term_msg.type = reasoning_msgs.msg.TermImpl.COMPOUND
        term_msg.functor = term.functor
        for i in range(term.get_arity()):
            sub_term_msg = term_to_msg2(term[i], full_term_msg)
            term_msg.sub_term_ptrs.append(len(full_term_msg.sub_terms))
            full_term_msg.sub_terms.append(sub_term_msg)
    return term_msg

# takes arguments: reasoning_msgs.msg.Term()
def msg_to_term(full_term_msg):
    return msg_to_term2(full_term_msg.root, full_term_msg)

# takes arguments: reasoning_msgs.msg.Term()
def msg_to_term2(term_msg, full_term_msg):
    if term_msg.type == reasoning_msgs.msg.TermImpl.CONSTANT:
        if term_msg.constant.type == reasoning_msgs.msg.Constant.STRING:
            return Constant(term_msg.constant.str)
        elif term_msg.constant.type == reasoning_msgs.msg.Constant.NUMBER:
            return Constant(str(term_msg.constant.num))

    elif term_msg.type == reasoning_msgs.msg.TermImpl.VARIABLE:
        return Variable(term_msg.variable)

    elif term_msg.type == reasoning_msgs.msg.TermImpl.COMPOUND:
        term = Compound(term_msg.functor)
        for i in range(len(term_msg.sub_term_ptrs)):
            sub_term = msg_to_term2(full_term_msg.sub_terms[term_msg.sub_term_ptrs[i]], full_term_msg)
            term.add_argument(sub_term)
        return term

    # if term type is unknown:
    return Term()

###########################################################################
#                                                                         # 
#                           CONVERSION METHODS                            #
#                                                                         #
###########################################################################

def term_to_point(term):
    if term.functor == "point":
        if term[0].is_number() and term[0].is_number() and term[0].is_number():
            return (term[0].get_number(), term[1].get_number(), term[2].get_number())
    return ()

###########################################################################
#                                                                         # 
#                                REASONER                                 #
#                                                                         #
###########################################################################
  
if __name__ == "__main__":

    query              = rospy.ServiceProxy('/reasoner/query', reasoning_msgs.srv.Query)
    assert_knowledge   = rospy.ServiceProxy('/reasoner/assert', reasoning_msgs.srv.Assert)

    term1 = Compound("fact", "X")

    reasoning_query = reasoning_msgs.srv.QueryRequest()
    reasoning_query.term = term_to_msg(term1)

    query_result = query(reasoning_query)

    # binding_sets is a list of dictionaries
    binding_sets = []

    for binding_set_msg in query_result.binding_sets:
        binding_set = {}
        for binding_msg in binding_set_msg.bindings:
            binding_set[binding_msg.variable] = msg_to_term(binding_msg.value)
        binding_sets.append(binding_set)

    print binding_sets

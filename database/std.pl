:- dynamic type/2.

% TODO: get rid of position_list to position conversion
position(ID, POS) :-
    position_list(ID, POS_LIST), !,
    member(binding(ID, POS), POS_LIST).

object_property(ID, PROPERTY, VALUE) :-
    object_property_list(ID, PROPERTY, VALUE_LIST), !,
    member(binding(ID, VALUE), VALUE_LIST).

type(ID, CLASS) :-
    type_list(ID, CLASS_LIST), !,
    member(binding(ID, CLASS), CLASS_LIST).

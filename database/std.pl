:- dynamic type/2.

% TODO: get rid of position_list to position conversion
position(ID, POS) :-
    position_list(ID, POS_LIST), !,
    member(binding(ID, POS), POS_LIST).

type(ID, CLASS) :-
    type_list(ID, CLASS_LIST), !,
    member(binding(ID, CLASS), CLASS_LIST).

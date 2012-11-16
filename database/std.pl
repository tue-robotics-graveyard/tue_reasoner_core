% TODO: get rid of position_list to position conversion
position(ID, POS) :-
    position_list(ID, POS_LIST), !,
    member(binding(ID, POS), POS_LIST).

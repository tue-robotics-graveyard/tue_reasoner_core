extern(Query, Server) :-
    extern(Query, Server, List), !,
    member(Bindings, List),
    bind(Bindings).

bind([]).
bind([X=Y|Rest]) :-
    X = Y,
    bind(Rest).

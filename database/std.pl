% TODO: get rid of complex_query
complex_query([]).
complex_query([H|T]) :-
	call(H),
	complex_query(T).


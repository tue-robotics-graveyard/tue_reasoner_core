% STRIPS planner
% AMIGO GPSR

% 02/02/2013 AMIGO

% Very basic description of robot capablities
initial_grippers_available(amigo, 2).
initial_grippers_available(pico , 0).

base_capability(amigo, mobile).
base_capability(pico,  mobile).

grab_capable_robot(Robot):- initial_grippers_available(Robot,N),N >= 1 , base_capability(Robot,mobile).

% This predicate initialises the problem states. The first argument
% of solve/3 is the initial state, the 2nd the goal state, and the 
% third the plan that will be produced.

execute(Plan):-   
    solve([at(amigo,entrance),on(coke,desk)],
	      [holding(amigo,coke),at(amigo,entrance)],
                 Plan).

solve(State, Goal, Plan):-
    solve(State, Goal, [], Plan).

% This predicate produces the plan. Once the Goal list is a subset 
% of the current State the plan is complete and it is written to 
% the screen using write_sol/1.

solve(State, Goal, Plan, Plan):-
	is_subset(Goal, State), nl,
	write_sol(Plan).

solve(State, Goal, Sofar, Plan):-
	operator(Operator, Preconditions, Delete, Add),
	is_subset(Preconditions, State),
	\+ member(Operator, Sofar),
	delete_list(Delete, State, Remainder),
	append(Add, Remainder, NewState),
	write(Operator), nl,   % Show the operators it is trying to apply
	solve(NewState, Goal, [Operator|Sofar], Plan).

% Operator definition

operator(travel(Location_initial,Location_final),
    % Precondition list
    [at(amigo,Location_initial)],
    % Delete list
    [at(amigo,Location_initial)],
    % Add list
    [at(amigo,Location_final)]).

operator(pickup(Object,Object_location),
    % Precondition list
    [at(amigo,Object_location),
     on(Object,Object_location)],
    % Delete list
    [],
    % Add list
    [holding(amigo,Object)]).

% Utility predicates.

% Check is first list is a subset of the second

is_subset([H|T], Set):-
    member(H, Set),
    is_subset(T, Set).
is_subset([], _).

% Remove all elements of 1st list from second to create third.

delete_list([H|T], List, Final):-
    remove(H, List, Remainder),
    delete_list(T, Remainder, Final).
delete_list([], List, List).
    
remove(X, [X|T], T).
remove(X, [H|T], [H|R]):-
    remove(X, T, R).

write_sol([]).
write_sol([H|T]):-
	write_sol(T),
	write(H), nl.
                 
%append([H|T], L1, [H|L2]):-
%    append(T, L1, L2).
%append([], L, L).

%member(X, [X|_]).
%member(X, [_|T]):-
%    member(X, T).


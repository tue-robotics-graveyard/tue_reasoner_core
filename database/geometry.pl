%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %
%                            GEOMETRY                              %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % % % % % % % % % % % % % % INIT % % % % % % % % % % % % % % % %

:- dynamic base_pose/2.
:- dynamic base_pose/4.

:- dynamic point_of_interest/2.
:- dynamic point_of_interest/4.

:- dynamic environment/1.
:- dynamic challenge/1.

challenge(none).

% % % % % % % % % % % % % % % SETTERS % % % % % % % % % % % % % % % %

set_challenge(Challenge) :-
	retractall(challenge(_)),
	assert(challenge(Challenge)).

set_environment(Env) :-
	retractall(environment(_)),
	assert(environment(Env)).


% % % % % % % % % % % % % CONTEXT RULES % % % % % % % % % % % % % % % 

waypoint(Name, Pose) :-
    challenge(Challenge),
    environment(Env),
    point_of_interest(Env, Challenge, Name, Pose).    

point_of_interest(Name, Point) :-
    challenge(Challenge),
    environment(Env),
    point_of_interest(Env, Challenge, Name, Point).
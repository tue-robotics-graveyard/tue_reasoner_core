%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %
%                            GEOMETRY                              %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % % % % % % % % % % % % % % INIT % % % % % % % % % % % % % % % %

% waypoint(Env, Chal, Name, Pose2D) - For environment Env and 
%     challenge Chal there is a waypoint with Name and Pose2D
%     example: waypoint(lab, rips, registration_table, pose_2d(1, 0, -0.5))
:- dynamic waypoint/4.

:- dynamic point_of_interest/4.

:- dynamic exploration_target/4.

% state(X, Y) - X has state Y (e.g. state(entrance_door, open) )
:- dynamic state/2.

% unreachable(X) - X is a waypoint that cannot be reached
:- dynamic unreachable/1.

:- dynamic environment/1.
:- dynamic challenge/1.

:- dynamic instance_of/4

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
    waypoint(Env, Challenge, Name, Pose).    

point_of_interest(Name, Point) :-
    challenge(Challenge),
    environment(Env),
    point_of_interest(Env, Challenge, Name, Point).

exploration_target(Room, Target) :-
    challenge(Challenge),
    environment(Env),
    exploration_target(Env, Challenge, Room, Target).

instance_of(Obj, Class) :-
    challenge(Challenge),
    environment(Env),
    instance_of(Env, Challenge, Obj, Class).
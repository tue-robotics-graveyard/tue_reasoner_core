:- dynamic challenge/1.
:- dynamic environment/1.
:- dynamic state/2.
:- dynamic explored/1.

environment(tue_test_lab).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            WORLD DATA: tue_test_lab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

waypoint(tue_test_lab, clean_up, meeting_point, pose_2d(1.293, -0.445, 0.914)).
waypoint(tue_test_lab, clean_up, kitchen, pose_2d(4.315, 1.141, -0.841)).
waypoint(tue_test_lab, clean_up, living_room, pose_2d(4.315, 1.141, -0.841)).

exploration_target(tue_test_lab, clean_up, living_room, bed_cabinet_1).
exploration_target(tue_test_lab, clean_up, living_room, cabinet_expedit_1).
exploration_target(tue_test_lab, clean_up, living_room, bed_1).

base_pose(cabinet_expedit_1, pose_2d(4.952,  1.351,  1.570)).
base_pose(bed_1,             pose_2d(6.058, -1.598,  3.113)).
base_pose(bed_cabinet_1,     pose_2d(3.797, -1.240, -1.608)).

region_of_interest(cabinet_expedit_1, point_3d(4.807,  2.102, 1.000)).
region_of_interest(bed_1,             point_3d(5.009, -1.706, 1.000)).
region_of_interest(bed_cabinet_1,     point_3d(3.729, -2.286, 1.000)).
region_of_interest(trashbin,     	  point_3d(5.6,    1.9,   1.1)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  HELPER RULES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

waypoint(Name, Pose) :-
    challenge(Challenge),
    environment(Env),
    waypoint(Env, Challenge, Name, Pose).

exploration_target(Room, Target) :-
    challenge(Challenge),
    environment(Env),
    exploration_target(Env, Challenge, Room, Target).

%type(coke, drink).

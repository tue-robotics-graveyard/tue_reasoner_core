%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  TAXONOMY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% multifile: gives possibility to assign predicate values in different files.
%            Otherwise it would be overwritten by the last imported database 
%	     if in the same predicate is in that database.

:- multifile object_at_coordinates/2.

:- dynamic class/2.
:- dynamic location/2.
:- dynamic visited/1.
:- dynamic unreachable/1.

:- dynamic object_at_coordinates/2.

class(obj1, coke).
class(obj2, table).
class(obj3, armchair).
class(desk_A, waypoint).
class(desk_B, waypoint).
class(desk_A, waypoint).
class(couch_table_A, waypoint).

class(wp1, waypoint).
class(wp2, waypoint).
class(wp3, waypoint).
class(wp4, waypoint).

direct_subclass(waypoint, location).

direct_subclass(drink, beverage).
direct_subclass(beverage, food).

at_location(obj1, kitchen).
at_location(obj2, living_room).
at_location(obj3, living_room).

% locations
direct_subclass(seat, furniture).
direct_subclass(table, furniture).
direct_subclass(shelf, furniture).
direct_subclass(appliance, furniture).
direct_subclass(bin, furniture).
direct_subclass(furniture, location).

direct_subclass(sofa, seat).
direct_subclass(couch_table, table).
direct_subclass(armchair, seat).
direct_subclass(stool, seat).
direct_subclass(dinner_table, table).
direct_subclass(bookshelf, shelf).
direct_subclass(buffet, shelf).
direct_subclass(fridge, appliance).
direct_subclass(stove, appliance).
direct_subclass(microwave, appliance).
direct_subclass(kitchen_table, table).
direct_subclass(bar, shelf).
direct_subclass(couch, seat).
direct_subclass(sideboard, shelf).
direct_subclass(waste_bin, bin).
direct_subclass(bed, seat).
direct_subclass(side_table, table).
direct_subclass(footstool, seat).

% objects
direct_subclass(beverage, object).
direct_subclass(snack, beverage).
direct_subclass(food, object).
direct_subclass(bathroom_stuff, object).

direct_subclass(coke, drink).
direct_subclass(seven_up, drink).
direct_subclass(lemon_tea, drink).
direct_subclass(pepsi, drink).
direct_subclass(mineral_water, drink).
direct_subclass(gatorade, drink).
direct_subclass(orange_juice, drink).
direct_subclass(apple_juice, drink).
direct_subclass(crackers, snack).
direct_subclass(wafer, snack).
direct_subclass(candy, snack).
direct_subclass(cookies, snack).
direct_subclass(chili, food).
direct_subclass(dough, food).
direct_subclass(tomato_sauce, food).
direct_subclass(soy_sauce, food).
direct_subclass(tofu, food).
direct_subclass(tea_box, food).
direct_subclass(cotton, bathroom_stuff).
direct_subclass(disinfectant, bathroom_stuff).
direct_subclass(hair_gel, bathroom_stuff).
direct_subclass(soap, bathroom_stuff).
direct_subclass(tooth_paste, bathroom_stuff).
direct_subclass(toilet_paper, bathroom_stuff).
direct_subclass(shampoo, bathroom_stuff).
direct_subclass(pad, bathroom_stuff).

is_subclass_of(X, Y) :-
	direct_subclass(X, Y).	

is_subclass_of(X, Z) :-
	direct_subclass(X, Y),
	is_subclass_of(Y, Z).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  COORDINATES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Should some of these coordinates not be in locations_hospital_room??

object_at_coordinates(obj1,             pose_2d(9.065, 0.490, 	0)).
object_at_coordinates(obj2,             pose_2d(7.374, 0.674, 	0)).
object_at_coordinates(obj3,             pose_2d(7.346, 2.195, 	0)).
object_at_coordinates(desk_A,           pose_2d(1.33, 	-0.38, 	-1.57)).
object_at_coordinates(desk_B,           pose_2d(1.53, 	-0.38, 	-1.57)).
object_at_coordinates(couch_table_A,    pose_2d(2.00, 	0.90,	0.45)).

object_at_coordinates(wp1, pose_2d(2.67, 1.75, 0)).
object_at_coordinates(wp2, pose_2d(2.25, 1.75, 1.57)).
object_at_coordinates(wp3, pose_2d(1.75, 0.4, 2.25)).
object_at_coordinates(wp4, pose_2d(4, 2, 0)).
object_at_coordinates(wp5, pose_2d(1, 2, 3)).

find_class_coordinates(Class, Object, Coordinates) :-
    class(Object, Class),
    object_at_coordinates(Object, Coordinates).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              SEMANTIC LOCATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class_in_room(Class, Object, Room) :- 
    class(Object, Class), 
    at_location(Object, Room).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               OTHER PROPERTIES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

has_property(couch_table, object_holder).
has_property(dinner_table, object_holder).
has_property(bookshelf, object_holder).
has_property(buffet, object_holder).
has_property(fridge, object_holder).
has_property(microwave, object_holder).
has_property(kitchen_table, object_holder).
has_property(bar, object_holder).
has_property(sideboard, object_holder).
has_property(bed, object_holder).
has_property(side_table, object_holder).

has_property(id_1, position, test).
has_property(id_1, position, test2).
has_property(X, Y, Z) :-
	comp_property(X, Y, Z).


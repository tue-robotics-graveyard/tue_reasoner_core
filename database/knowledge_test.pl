%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  TAXONOMY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- dynamic type/2.
:- dynamic location/2.
:- dynamic is_class_at_coordinates/2.

type(coke, drink).
type(sprite, drink).

location(sprite, kitchen).
location(coke, living_room).
location(sprite, living_room).

% locations
is_direct_subclass_of(seat, location).
is_direct_subclass_of(table, location).
is_direct_subclass_of(shelf, location).
is_direct_subclass_of(appliance, location).
is_direct_subclass_of(bin, location).

is_direct_subclass_of(sofa, seat).
is_direct_subclass_of(couch_table, table).
is_direct_subclass_of(armchair, seat).
is_direct_subclass_of(stool, seat).
is_direct_subclass_of(dinner_table, table).
is_direct_subclass_of(bookshelf, shelf).
is_direct_subclass_of(buffet, shelf).
is_direct_subclass_of(fridge, appliance).
is_direct_subclass_of(stove, appliance).
is_direct_subclass_of(microwave, appliance).
is_direct_subclass_of(kitchen_table, table).
is_direct_subclass_of(bar, shelf).
is_direct_subclass_of(couch, seat).
is_direct_subclass_of(sideboard, shelf).
is_direct_subclass_of(waste_bin, bin).
is_direct_subclass_of(bed, seat).
is_direct_subclass_of(side_table, table).
is_direct_subclass_of(footstool, seat).

% objects
is_direct_subclass_of(drink, object).
is_direct_subclass_of(snack, object).
is_direct_subclass_of(food, object).
is_direct_subclass_of(bathroom_stuff, object).

is_direct_subclass_of(coke, drink).
is_direct_subclass_of(seven_up, drink).
is_direct_subclass_of(lemon_tea, drink).
is_direct_subclass_of(pepsi, drink).
is_direct_subclass_of(mineral_water, drink).
is_direct_subclass_of(gatorade, drink).
is_direct_subclass_of(orange_juice, drink).
is_direct_subclass_of(apple_juice, drink).
is_direct_subclass_of(crackers, snack).
is_direct_subclass_of(wafer, snack).
is_direct_subclass_of(candy, snack).
is_direct_subclass_of(cookies, snack).
is_direct_subclass_of(chili, food).
is_direct_subclass_of(dough, food).
is_direct_subclass_of(tomato_sauce, food).
is_direct_subclass_of(soy_sauce, food).
is_direct_subclass_of(tofu, food).
is_direct_subclass_of(tea_box, food).
is_direct_subclass_of(cotton, bathroom_stuff).
is_direct_subclass_of(disinfectant, bathroom_stuff).
is_direct_subclass_of(hair_gel, bathroom_stuff).
is_direct_subclass_of(soap, bathroom_stuff).
is_direct_subclass_of(tooth_paste, bathroom_stuff).
is_direct_subclass_of(toilet_paper, bathroom_stuff).
is_direct_subclass_of(shampoo, bathroom_stuff).
is_direct_subclass_of(pad, bathroom_stuff).

is_subclass_of(X, Y) :-
	is_direct_subclass_of(X, Y).	

is_subclass_of(X, Z) :-
	is_direct_subclass_of(X, Y),
	is_subclass_of(Y, Z).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  COORDINATES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

is_class_at_coordinates(exit, point(0, 1, 2)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              SEMANTIC LOCATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% living room
is_class_in_room(sofa, living_room).
is_class_in_room(couch_table, living_room).
is_class_in_room(armchair, living_room).
is_class_in_room(stool, living_room).
is_class_in_room(dinner_table, living_room).
is_class_in_room(bookshelf, living_room).
is_class_in_room(buffet, living_room).

% kitchen
is_class_in_room(fridge, kitchen).
is_class_in_room(stove, kitchen).
is_class_in_room(microwave, kitchen).
is_class_in_room(kitchen_table, kitchen).
is_class_in_room(bar, kitchen).

% lobby
is_class_in_room(couch, lobby).
is_class_in_room(sideboard, lobby).
is_class_in_room(waste_bin, lobby).

% bed_room
is_class_in_room(bed, bed_room).
is_class_in_room(side_table, bed_room).
is_class_in_room(footstool, bed_room).

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


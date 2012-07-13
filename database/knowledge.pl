%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  TAXONOMY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

is_class_at_coordinates(sofa, [9.065, 0.490, 0.000]).
is_class_at_coordinates(couch_table, [7.374, 0.674, 0.000]).
is_class_at_coordinates(armchair, [7.346, 2.195, 0.000]).
is_class_at_coordinates(stool, [6.224, 0.684, 0.000]).
is_class_at_coordinates(dinner_table, [6.653, -2.691, 0.000]).
is_class_at_coordinates(bookshelf, [9.040, -1.281, 0.000]).
is_class_at_coordinates(buffet, [4.689, -1.281, 0.000]).
is_class_at_coordinates(fridge, [8.676, -6.172, 0.000]).
is_class_at_coordinates(stove, [7.358, -7.075, 0.000]).
is_class_at_coordinates(microwave, [6.457, -7.096, 0.000]).
is_class_at_coordinates(kitchen_table, [7.210, -4.001, 0.000]).
is_class_at_coordinates(bar, [5.260, -5.461, 0.000]).
is_class_at_coordinates(couch, [2.287, 2.539, 0.000]).
is_class_at_coordinates(sideboard, [2.252, -1.503, 0.000]).
is_class_at_coordinates(waste_bin, [3.370, 2.451, 0.000]).
is_class_at_coordinates(bed, [2.813, -4.956, 0.000]).
is_class_at_coordinates(side_table, [3.597, -6.213, 0.000]).
is_class_at_coordinates(footstool, [0.465, -6.569, 0.000]).
is_class_at_coordinates(exit, [-0.369, 0.073, 0.000]).

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

% TODO: get rid of complex_query
complex_query([]).
complex_query([H|T]) :-
	call(H),
	complex_query(T).


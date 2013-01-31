%:- dynamic type/2.
:- dynamic property/3.

% multifile allows multiple definitions of a predicate over different files
%:- multifile type/2.
:- multifile property/3.

object_property(ID, PROPERTY, VALUE) :-
    writeln('object_property/3 is obsolete; use property/3 instead'),
    property(ID, PROPERTY, VALUE).

type(ID, CLASS) :-
    property(ID, class_label, CLASS).

position(ID, POS) :-
    property(ID, position, POS).

property(ID, Prop, Value) :-
    property_list(ID, Prop, ValueList),
    member(binding(ID, Value), ValueList).

% position

property(amigo, position, vector(X, Y, Z)) :-
    lookup_transform('/map', '/base', transform(vector(X, Y, Z), _)).

property(ID1, position, ID2) :-
	property(ID2, area, area_2d(Xmin, Xmax, Ymin, Ymax)),
	property(ID1, position, vector(X, Y, _)),	
    ID1 \= ID2,
	X > Xmin, X < Xmax, Y > Ymin, Y < Ymax.

% near

property(ID1, near, ID2) :-
    property(ID1, position, vector(X1, Y1, _)),
    property(ID2, position, vector(X2, Y2, _)),
    ID1 \= ID2,
    abs(X1-X2, XDiff),
    abs(Y1-Y2, YDiff),
    XDiff < 1,
    YDiff < 1.   

%:- dynamic type/2.
:- dynamic property/3.

% multifile allows multiple definitions of a predicate over different files
%:- multifile type/2.
:- multifile property/3.

object_property(ID, PROPERTY, VALUE) :-
    writeln('object_property/3 is obsolete; use property/3 instead'),
    property(ID, PROPERTY, VALUE).

type(ID, CLASS) :-
    property_expected(ID, class_label, CLASS).

position(ID, point(X, Y, Z)) :-
    property(ID, position, gaussian(vector([X, Y, Z]), _)).

property(ID, Prop, Value) :-
    property_list(ID, Prop, ValueList),
    member(binding(ID, Value), ValueList).

% position

property(amigo, position, gaussian(vector(X, Y, Z), symmetric_matrix([0,0,0,0,0,0]))) :-
    lookup_transform('/map', '/base', transform(vector(X, Y, Z), _)).

%property(ID1, position, ID2) :-
%	property(ID2, area, area_2d(Xmin, Xmax, Ymin, Ymax)),
%	property(ID1, position, vector(X, Y, _)),	
%    ID1 \= ID2,
%	X > Xmin, X < Xmax, Y > Ymin, Y < Ymax.

% near

property(ID1, near, ID2) :-
    property(ID1, position, vector(X1, Y1, _)),
    property(ID2, position, vector(X2, Y2, _)),
    ID1 \= ID2,
    abs(X1-X2, XDiff),
    abs(Y1-Y2, YDiff),
    XDiff < 1,
    YDiff < 1.

% Returns expected value for property
property_expected(X, Prop, Val) :-
    property(X, Prop, PDF),
    get_expected(PDF, Val).

% Returns expected value of a PDF
get_expected(gaussian(vector(Val), _), Val).
get_expected(discrete(Domain, Values), Val) :-
    get_expected(discrete(Domain, Values), Val, _).

get_expected(discrete(_, [p(Prob, Val)]),      Val, Prob).
get_expected(discrete(_, [p(Prob, Val)|Rest]), MaxVal, MaxProb) :-
    get_expected(discrete(_, Rest), Val2, Prob2),
    (
      Val > Val2
      ->
        MaxVal = Val,
        MaxProb = Prob
      ;
        MaxVal = Val2,
        MaxProb = Prob2
    ).

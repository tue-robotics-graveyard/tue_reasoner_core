%:- dynamic type/2.
:- dynamic property/3.

% multifile allows multiple definitions of a predicate over different files
%:- multifile type/2.
:- multifile property/3.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %
%                      DATABASE MANAGEMENT                         %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load_database(ROSPackage, RelDBFilename) :-
    get_ros_package_path(ROSPackage, Path),
    atom_concat(Path, '/', Path2),
    atom_concat(Path2, RelDBFilename, AbsDBFilename),
    load_database(AbsDBFilename).

load_database(AbsDatabaseFilename) :-
    consult(AbsDatabaseFilename),
    source_file(AbsDatabaseFilename). % check if file was loaded successfully

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                  %
%                            UTILITIES                             %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

list_facts(F) :-
    current_predicate(_, F),
    catch(clause(F, true), _, fail).

list_facts(PredicateName, F) :-
    current_predicate(PredicateName, F),
    catch(clause(F, true), _, fail).

list_rules(Head, Body) :-
    current_predicate(_, Head),
    catch(clause(Head, Body), _, fail),
    Body \= true.
    
list_rules(PredicateName, Head, Body) :-
    current_predicate(PredicateName, Head),
    catch(clause(Head, Body), _, fail),
    Body \= true.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                       SUBCLASS_OF, ETC
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subclass_of(A, B) :-
    direct_subclass_of(A, B).
subclass_of(A, C) :-
    direct_subclass_of(A, B),
    subclass_of(B, C).
                
instance_of(Obj, Class) :-
    property_expected(Obj, class_label, Class).
instance_of(Obj, Class) :-
    property_expected(Obj, class_label, A),
    subclass_of(A, Class).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

object_property(ID, PROPERTY, VALUE) :-
    writeln('object_property/3 is obsolete; use property/3 instead'),
    property(ID, PROPERTY, VALUE).

type(ID, CLASS) :-
    property_expected(ID, class_label, CLASS).

position(ID, point(X, Y, Z)) :-
    property(ID, position, gaussian(vector([X, Y, Z]), _)).

property(ID, Prop, Value) :-
    property_list(ID, Prop, '/map', Bindings),
    member(binding(ID, Prop, Value), Bindings).

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

property(ID, Prop, Value, FrameID) :-
    property_list(ID, Prop, FrameID, Bindings),
    member(binding(ID, Prop, Value), Bindings).

property_expected(ID1, position, in_front_of(amigo)) :-
    property_expected(ID1, position, [X, Y, Z]),
    transform_point('/map', [X, Y, Z], '/base_link', [Dist, HorDist, _]),
    Dist > 0,
    Dist < 2,
    abs(HorDist, HorDistAbs),
    Ratio is Dist / HorDistAbs,
    Ratio > 1.

property_expected(amigo, pose, pose_2d(X, Y, Phi)) :-
    lookup_transform('/map', '/base_link', transform(vector(X, Y, _), Quat)),
    quaternion_to_rpy(Quat, rpy(Phi, _, _)).

% Returns expected value for property
property_expected(X, Prop, Val) :-
    property(X, Prop, PDF),
    get_expected(PDF, Val2),
    (
      Val2 = exact(Val)
     ->
      true
     ;
      Val = Val2
    ).

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

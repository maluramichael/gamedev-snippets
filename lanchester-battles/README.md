# lanchester-battles

Lanchester's two attrition laws next to an agent simulation that knows nothing about them. Two armies of dots fight
on the left; on the right the differential equations are integrated with RK4 and drawn as dashed curves, with the
survivor predicted from the conserved quantity (`alpha*A^2 - beta*B^2` for ranged fire, `alpha*A - beta*B` for
one-on-one duels). The dots have a weapon, a range, a way to pick targets and legs. Chips switch between ranged
weapons and duels, the army sizes (including a split army that attacks in two waves), the strength of one B unit,
short range and the aiming rule (coordinated, nearest, random), so you can see where the simulation leaves the law and
why: units that cannot reach anybody, shots wasted on corpses, reserves that wait. Keys are shown on the chips, `R`
fights again. The math and the simulation (`logic.hpp`) have no SDL dependency and are covered by `test.cpp`, which runs
whole battles with fixed seeds and compares their averages with the formulas.

Idea and the RTS use of the laws: Stanescu, Barriga and Buro, "Combat Outcome Prediction for Real-Time Strategy Games",
in Game AI Pro 3, chapter 25.

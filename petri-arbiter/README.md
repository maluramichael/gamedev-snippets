# petri-arbiter

A squad wants one mounted gun that only one soldier can use. A small Petri net is the arbiter: places and tokens stand for the
agents and for the gun, a transition fires only when every input place holds a token, and "Assign" needs a waiting agent and
the free gun at the same time. Hit the gunner (click, key K or Auto-hit) and the gun token flows back, so the next agent
gets it. The net says when, a selection rule says who: FIFO (registration order, the naive one, it happily sends the agent
who is farthest away), Nearest, or Score (closeness weighted 0.6 against accuracy 0.4). The number under each agent is what the
rule compares, the orange ring marks who is next in line, and the card shows the average distance and walking time of the
gunners so far. The slider sets 2 to 10 agents, which also sets the arc weight of the "Start" transition. The net, the
selection and the walking are in `logic.hpp` and covered by `test.cpp`, including a run of thousands of random hits that
checks the gun is never given twice. The idea follows chapter 29 of Game AI Pro 3, "Petri Nets and AI Arbitration" by
Sergio Ocio Barriales.

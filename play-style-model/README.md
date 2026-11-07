# play-style-model

A game that measures taste instead of moves. Every move (Fight, Tactics, Story, Roleplay, Power-up) adds a small update
vector to a profile of five numbers: fighter, tactician, storyteller, role-player, power gamer. The game scores each
candidate event with a dot product against the profile and shows the best one. It starts with the player from chapter 42
of Game AI Pro 2: Grendel scores 0.81, the magic fairy 0.09. Three story moves later the fairy wins, and the card below
the list writes the winning dot product out. "Select: last move" switches to the naive rule that only looks at the latest
move, so one off-style move flips it while the profile stays put. "Forget" pulls the profile back to neutral with a
half-life of 6 seconds, so recent moves weigh more. New player starts from zeros, Chapter example restores the first
frame. The logic is in `logic.hpp` and covered by `test.cpp`, the update vectors and annotations are the demo's own choice.

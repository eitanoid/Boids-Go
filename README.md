# What are Boids?

[]


[Boids](https://en.wikipedia.org/wiki/Boids) which stands for bird-like objects, is a flocking simulation initially described by Craig Reynolds in 1986 which is a great example for emergent behaviour.

Each Boid is an individual agent following a set of rules, give rise to flocking behaviour similar to one observed by real birds, these ruels are:

- Coherence: Boids want to stick together, so they fly towards their precieved center of mass.
- Alignment: Boids want to fly in the same direction as their flock, so they match their velocity to their precieved average velocity.
- Seperation: Boids want to avoid bumping into other boids, so they fly away from boids that are too close.

Varying the coeffecients by which the boids react to these rules, as well as the radius of sight and avoidance yield different simulation results - from very orderly flocks to very chaotic individual behaviour.

# Description

An implementation of Boids in Go visualised by the `Raylib-Go` game library.


# Installation

# Prerequisites

- Go: This project requires the Go programming language. You can download it from https://golang.org/dl/.
- Raylib-Go: This project relies on the `Raylib-Go` game library found at https://github.com/gen2brain/raylib-go, to install:
  
  `go get -v -u github.com/gen2brain/raylib-go/raylib`

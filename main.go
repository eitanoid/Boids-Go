package main

import (
	"fmt"
	"math"
	"math/rand"
	"sync"
	"time"

	"log"
	"net/http"
	_ "net/http/pprof"

	rl "github.com/gen2brain/raylib-go/raylib"
)

//TODO:
//
//
//
//
//

var (
	coherence_factor float64 = 0.04 //centering
	alignment_factor float64 = 0.09 //allignment
	avoidance_factor float64 = 0.05 //seperation
	avoid_radius     float64 = 50
	coher_radius     float64 = 100
	max_speed        float64 = 20
	turn_factor      float64 = 4 //how fast to return a cell that escaped
	turn_margin      float64 = 20

	fps_target int = 60
	init_boids int = 150
)

type Boid struct {
	Id           int
	Position     Tuple // position
	Velocity     Tuple // velocity
	Avoid_Range  float64
	Follow_Range float64           // visual range
	Flock        map[*Boid]float64 // optimise assigning this

	Precieved_Center   Tuple
	Precieved_Velocity Tuple
}

type Game struct {
	FPS_Target int32
	Width      int32
	Height     int32
	Title      string
	Entities   []*Boid
	Distances  [][]float64
}

func (g *Game) Run() {

	var game_wg sync.WaitGroup

	rl.InitWindow(g.Width, g.Height, g.Title)
	defer rl.CloseWindow()

	rl.SetTargetFPS(g.FPS_Target)

	g.Init_Boids(init_boids) //

	for !rl.WindowShouldClose() {
		//fmt.Println(len(g.Entities))
		//go user_input()

		//go game()

		game_wg.Wait() // let all tasks finish before drawing

		rl.BeginDrawing()
		rl.ClearBackground(rl.Black)

		for _, boid := range g.Entities {
			Draw_Boid(boid, 15)
		}

		rl.EndDrawing()

		//now := time.Now()
		g.Evolve_Game(&g.Entities)
		//fmt.Printf("evolution took %d ms \n", time.Since(now).Milliseconds())

	}
}

func main() {

	game := Game{
		FPS_Target: int32(fps_target),
		Width:      2400,
		Height:     1400,
		Title:      "Boids",
	}

	go func() {
		log.Println(http.ListenAndServe("localhost:6060", nil))
	}()

	game.Run()

}

func (g *Game) Init_Boids(numBoids int) {
	g.Distances = make([][]float64, numBoids)
	for i := 0; i < numBoids; i++ {
		g.Distances[i] = make([]float64, numBoids) // imitiate distance matrix
		b := &Boid{
			Id:           i,
			Position:     Tuple{X: float64(rand.Intn(int(g.Width/2)) + int(g.Width)/4), Y: float64(rand.Intn(int(g.Height/2)) + int(g.Height)/4)},
			Velocity:     Tuple{X: float64(rand.Intn(40) - 20), Y: float64(rand.Intn(40) - 20)},
			Follow_Range: coher_radius,
			Avoid_Range:  avoid_radius,
			Flock:        make(map[*Boid]float64),
		}
		g.Entities = append(g.Entities, b)
	}

}

func (g *Game) Evolve_Game(boids *[]*Boid) {

	var sep, al, coh Tuple // seperation alignment and choesion vectors

	//now := time.Now()
	g.Set_Centers(boids)
	g.Set_Velocities(boids)
	//fmt.Printf("setting took %d ms", time.Since(now).Milliseconds())

	now := time.Now()
	for _, b := range *boids {

		g.Set_Flock(boids)
		sep = g.Seperation(b, avoidance_factor)
		al = Alignment(b, alignment_factor)
		coh = Cohesion(b, coherence_factor)

		b.Velocity = SumTuples(b.Velocity, (SumTuples(sep, al, coh))) // update boid

		if speed := Norm_Squared(b.Velocity); speed > max_speed*max_speed { // enforce max speed
			speed := math.Sqrt(Norm_Squared(b.Velocity))
			b.Velocity = Tuple{X: b.Velocity.X * max_speed / speed, Y: b.Velocity.Y * max_speed / speed}
		}

		b.Position = SumTuples(b.Position, b.Velocity)

		Maintain_Bounds(b, g)

	}
	fmt.Printf("updating took %d ms\n", time.Since(now).Milliseconds())

}

func (g *Game) Set_Flock(boids *[]*Boid) { // assign the distances between the boids (distance is symmetric)

	for i, b := range *boids {
		for j, a := range *boids {
			if i > j {
				r := math.Sqrt(Distance_Squared(a.Position, b.Position)) //the distance is squared to save on compute.
				g.Distances[i][j] = r
				g.Distances[j][i] = r
			}

		}
	}
}

func (g *Game) Set_Centers(boids *[]*Boid) { // sets the precieved center of mass for each boid

	for i, b := range *boids {
		c_m := Tuple{0, 0}
		m := 0
		for j, r := range g.Distances[i] {
			a := g.Entities[j]
			if r < b.Follow_Range {
				c_m.X += a.Position.X
				c_m.Y += a.Position.Y
				m++
			}
		}
		if m > 1 {
			b.Precieved_Center.X = (c_m.X - b.Position.X) / float64(m-1)
			b.Precieved_Center.Y = (c_m.Y - b.Position.Y) / float64(m-1)
		} else {
			b.Precieved_Center = b.Position // No flock, keep current position.
		}

	}
}

func (g *Game) Set_Velocities(boids *[]*Boid) { // sets the precieved average velocity for each boid // can be optimised O(n^2)

	for i, b := range *boids {
		c_v := Tuple{0, 0}
		m := 0
		for j, r := range g.Distances[i] {
			a := g.Entities[j]
			if r < b.Follow_Range {
				c_v.X += a.Velocity.X
				c_v.Y += a.Velocity.Y
				m++
			}
		}
		if m > 1 {
			b.Precieved_Velocity.X = (c_v.X - b.Velocity.X) / float64(m-1)
			b.Precieved_Velocity.Y = (c_v.Y - b.Velocity.Y) / float64(m-1)
		} else {
			b.Precieved_Velocity = b.Velocity
		}

	}

}

func Cohesion(b *Boid, f float64) Tuple { //boids stick together as flocks

	pos := b.Position
	p_c := b.Precieved_Center

	coh := Tuple{X: (p_c.X - pos.X) * f, Y: (p_c.Y - pos.Y) * f}
	return coh

}

func (g *Game) Seperation(b *Boid, f float64) Tuple { // boids cant be too cloes together

	s := Tuple{0, 0}
	i := b.Id
	for j, r := range g.Distances[i] {
		a := g.Entities[j]
		if r < b.Avoid_Range {
			s = Tuple{s.X - (a.Position.X - b.Position.X), s.Y - (a.Position.Y - b.Position.Y)}
		}
	}

	s = Tuple{s.X * f, s.Y * f}
	return s
}

func Alignment(b *Boid, f float64) Tuple { // flocks want to be aligned in the same direction

	vel := b.Velocity
	p_v := b.Precieved_Velocity
	al := Tuple{X: (p_v.X - vel.X) * f, Y: (p_v.Y - vel.Y) * f}
	return al

}

func Maintain_Bounds(b *Boid, g *Game) { // boids cannot leave the screen

	if b.Position.X < turn_margin {
		b.Velocity.X += turn_factor
	} else if b.Position.X-turn_margin > float64(g.Width) {
		b.Velocity.X -= turn_factor
	}

	if b.Position.Y < turn_margin {
		b.Velocity.Y += turn_factor
	} else if b.Position.Y-turn_margin > float64(g.Height) {
		b.Velocity.Y -= turn_factor
	}
}

func Draw_Boid(b *Boid, scale float64) { // draw triangle boids facing in the direction of motion

	x, y := b.Position.X, b.Position.Y
	alpha := math.Atan2(b.Velocity.Y, b.Velocity.X)
	rl.DrawTriangle(
		rl.Vector2{X: float32(x + scale*math.Cos(alpha)), Y: float32(y + scale*math.Sin(alpha))}, //tip
		rl.Vector2{X: float32(x + scale*math.Cos(alpha-math.Pi*4/5)), Y: float32(y + scale*math.Sin(alpha-math.Pi*4/5))},
		rl.Vector2{X: float32(x + scale*math.Cos(alpha+math.Pi*4/5)), Y: float32(y + scale*math.Sin(alpha+math.Pi*4/5))},
		rl.Green)

}

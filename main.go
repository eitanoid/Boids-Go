package main

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"net/http"
	_ "net/http/pprof"
	"runtime"
	"sync"
	"time"

	rl "github.com/gen2brain/raylib-go/raylib"
)

//TODO:
// potential concurrency
// async user input
// add spatial partitioning to Set_Flock function
// user input
// raygui???

var (

	//game adjustments
	coherence_factor float64 = 0.04 //centering
	alignment_factor float64 = 0.09 //allignment
	avoidance_factor float64 = 0.06 //seperation
	avoid_radius     float64 = 30   //30
	coher_radius     float64 = 70   //100
	max_speed        float64 = 15
	turn_factor      float64 = 1   // 2//how fast to return a cell that escaped
	turn_margin      float64 = 100 //100
	init_boids       int     = 1000

	//program adjustments
	//workers    int     = 2
	fps_target int     = 60
	scale      float64 = 2
	x_bound    float64 = float64(width) * scale
	y_bound    float64 = float64(height) * scale

	//screen
	width  int32 = 2400
	height int32 = 1400
)

type Boid struct {
	Id           int
	Position     Tuple // position
	Acceleration Tuple
	Velocity     Tuple // velocity
	Avoid_Range  float64
	Follow_Range float64 // visual range

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
			Draw_Boid(boid, 15/scale, scale)
		}

		rl.EndDrawing()
		//now := time.Now()
		g.Evolve_Game(&g.Entities)
		//fmt.Printf("evolution took %d ms \n", time.Since(now).Milliseconds())

	}
}

func main() {
	runtime.GOMAXPROCS(runtime.NumCPU())
	//workers = runtime.NumCPU() // num of threads

	game := Game{
		FPS_Target: int32(fps_target),
		Width:      width,
		Height:     height,
		Title:      "Boids",
	}

	go func() {
		log.Println(http.ListenAndServe("localhost:6060", nil))
	}()

	game.Run()

}

func (g *Game) Init_Boids(numBoids int) {
	g.Distances = make([][]float64, numBoids)

	game_width, game_height := float64(g.Width)*scale, float64(g.Height)*scale
	for i := 0; i < numBoids; i++ {
		g.Distances[i] = make([]float64, numBoids) // imitiate distance matrix
		b := &Boid{
			Id:           i,
			Position:     Tuple{X: float64(rand.Intn(int(game_width/2)) + int(game_width)/4), Y: float64(rand.Intn(int(game_height/2)) + int(game_height)/4)},
			Velocity:     Tuple{X: float64(rand.Intn(40) - 20), Y: float64(rand.Intn(40) - 20)},
			Follow_Range: coher_radius * coher_radius,
			Avoid_Range:  avoid_radius * coher_radius,
		}
		g.Entities = append(g.Entities, b)
	}
}

func (g *Game) Evolve_Game(boids *[]*Boid) {

	var sep, al, coh Tuple // seperation alignment and choesion vectors

	now := time.Now()

	// set heading and alignment
	//g.Set_Centers(boids)
	//g.Set_Velocities(boids)
	for _, b := range *boids {

		g.Set_Flock(b, boids) //, boids)
		g.Set_Prespective(b, boids)
		sep = g.Seperation(b, avoidance_factor)
		al = Alignment(b, alignment_factor)
		coh = Cohesion(b, coherence_factor)

		b.Acceleration = SumTuples(sep, al, coh)           // acceleration
		b.Velocity = SumTuples(b.Velocity, b.Acceleration) // update boid

		if speed := Norm_Squared(b.Velocity); speed > max_speed*max_speed { // enforce max speed
			speed := math.Sqrt(Norm_Squared(b.Velocity))
			b.Velocity = Tuple{X: b.Velocity.X * max_speed / speed, Y: b.Velocity.Y * max_speed / speed}
		}

		b.Position = SumTuples(b.Position, b.Velocity)

		g.Maintain_Bounds(b)

	}
	fmt.Printf("Evolution took %d us \n", time.Since(now).Microseconds())

}

/*
func (g *Game) Set_Flock(b *Boid, boids *[]*Boid) { // assign the distances between the boids (distance is symmetric) VERY SLOW multithread the entire game not just this.

	var wg sync.WaitGroup
	entities := *boids
	chunk_size := len(entities) / workers
	id := b.Id

	for i := 0; i < workers; i++ {
		start := i * chunk_size
		end := start + chunk_size
		if i == workers-1 {
			end = len(entities)
		}

		wg.Add(1)

		go func(start, end int) {
			defer wg.Done()
			for j := start; j < end; j++ {
				if j != id {
					r := Distance_Squared(entities[j].Position, b.Position)
					g.Distances[id][j] = r
					g.Distances[j][id] = r
				}
			}
		}(start, end)
	}

	wg.Wait()
} */

func (g *Game) Set_Flock(b *Boid, boids *[]*Boid) { // assign the distances between the boids (distance is symmetric)
	entities := *boids
	i := b.Id
	for j := 0; j < len(entities); j++ {
		dy := entities[j].Position.Y - b.Position.Y
		dx := entities[j].Position.X - b.Position.X
		if j != i && (b.Follow_Range > max(-dy, dy) && b.Follow_Range > max(dx, -dy)) {
			r := dx*dx + dy*dy //the distance is squared to save on compute.
			g.Distances[i][j] = r
			g.Distances[j][i] = r
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

func (g *Game) Set_Prespective(b *Boid, boids *[]*Boid) {

	p_v := Tuple{0, 0}
	p_c := Tuple{0, 0}
	m := 0
	entities := *boids
	i := b.Id
	for j := 0; j < len(entities); j++ {
		a := g.Entities[j]
		r := g.Distances[i][j]
		if r < b.Follow_Range {
			p_v.X += a.Velocity.X
			p_v.Y += a.Velocity.Y
			p_c.X += a.Position.X
			p_c.Y += a.Position.Y
			m++
		}
	}
	if m > 1 {
		b.Precieved_Velocity.X = (p_v.X - b.Velocity.X) / float64(m-1)
		b.Precieved_Velocity.Y = (p_v.Y - b.Velocity.Y) / float64(m-1)

		b.Precieved_Center.X = (p_c.X - b.Position.X) / float64(m-1)
		b.Precieved_Center.Y = (p_c.Y - b.Position.Y) / float64(m-1)
	} else {
		b.Precieved_Velocity = b.Velocity
		b.Precieved_Center = b.Position
	}

}

func Cohesion(b *Boid, f float64) Tuple { //follow others

	pos := b.Position
	p_c := b.Precieved_Center

	coh := Tuple{X: (p_c.X - pos.X) * f, Y: (p_c.Y - pos.Y) * f}
	return coh

}

func (g *Game) Seperation(b *Boid, f float64) Tuple { //avoid collisions

	s := Tuple{0, 0}
	i := b.Id

	for j, r := range g.Distances[i] {
		a := g.Entities[j]
		if r < b.Avoid_Range {

			s.X += (b.Position.X - a.Position.X)
			s.Y += (b.Position.Y - a.Position.Y)
		}
	}

	s = Tuple{s.X * f, s.Y * f}
	return s
}

func Alignment(b *Boid, f float64) Tuple { // match velocity

	vel := b.Velocity
	p_v := b.Precieved_Velocity
	al := Tuple{X: (p_v.X - vel.X) * f, Y: (p_v.Y - vel.Y) * f}
	return al

}

func (g *Game) Maintain_Bounds(b *Boid) { // boids cannot leave the screen

	if b.Position.X < turn_margin {
		b.Velocity.X += turn_factor
	} else if b.Position.X > x_bound-turn_margin {
		b.Velocity.X -= turn_factor
	}

	if b.Position.Y < turn_margin {
		b.Velocity.Y += turn_factor
	} else if b.Position.Y > y_bound-turn_margin {
		b.Velocity.Y -= turn_factor
	}
}

func Draw_Boid(b *Boid, scale float64, game_scale float64) { // draw triangle boids facing in the direction of motion

	x, y := b.Position.X/game_scale, b.Position.Y/game_scale

	alpha := math.Atan2(b.Velocity.Y, b.Velocity.X)
	rl.DrawTriangle(
		rl.Vector2{X: float32(x + scale*math.Cos(alpha)), Y: float32(y + scale*math.Sin(alpha))}, //tip
		rl.Vector2{X: float32(x + scale*math.Cos(alpha-math.Pi*4/5)), Y: float32(y + scale*math.Sin(alpha-math.Pi*4/5))},
		rl.Vector2{X: float32(x + scale*math.Cos(alpha+math.Pi*4/5)), Y: float32(y + scale*math.Sin(alpha+math.Pi*4/5))},
		rl.Green)

}

func (g *Game) Add_Boid(pos Tuple) {
	b := Boid{
		Id:                 len(g.Entities),
		Position:           pos,
		Velocity:           Tuple{0, 0},
		Precieved_Center:   pos,
		Precieved_Velocity: Tuple{0, 0},
		Follow_Range:       coher_radius * coher_radius,
		Avoid_Range:        avoid_radius * avoid_radius,
	}
	g.Entities = append(g.Entities, &b)

}

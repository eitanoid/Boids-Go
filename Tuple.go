package main

import "math"

type Tuple struct {
	X, Y float64
}

func (a *Tuple) Scale(f float64) {
	a.X *= f
	a.Y *= f
}

func Distance_Squared(a Tuple, b Tuple) float64 { // returns the distance squared
	x := b.X - a.X
	y := b.Y - a.Y
	return x*x + y*y
}

func SumTuples(tuples ...Tuple) Tuple {
	sum := Tuple{X: 0, Y: 0} // Initialize an empty Tuple to store the sum

	for _, t := range tuples {
		sum.X += t.X
		sum.Y += t.Y
	}

	return sum
}

func Normalize(a Tuple) Tuple {
	length := math.Sqrt(a.X*a.X + a.Y*a.Y)
	if length == 0 {
		return Tuple{0, 0}
	}
	a.X /= length
	a.Y /= length

	return a
}

func Norm_Squared(a Tuple) float64 {

	return a.X*a.X + a.Y*a.Y
}

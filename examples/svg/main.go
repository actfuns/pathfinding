// Example: Generate SVG visualization of a weighted grid with path overlay.
package main

import (
	"fmt"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func main() {
	matrix := [][]int{
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
	}

	g := grid.NewOrthogonalGrid(matrix)

	g.SetWeightAt(2, 0, 5.0)
	g.SetWeightAt(3, 0, 5.0)
	g.SetWeightAt(2, 1, 5.0)
	g.SetWeightAt(3, 1, 5.0)

	for x := 0; x < 7; x++ {
		g.SetWeightAt(x, 5, 0.3)
		g.SetWeightAt(x, 0, 0.3)
	}
	for y := 0; y < 6; y++ {
		g.SetWeightAt(0, y, 0.3)
		g.SetWeightAt(6, y, 0.3)
	}

	f := finder.NewAStarFinder()
	path := f.FindPath(0, 0, 6, 5, g)

	svg := g.RenderSVG(nil, path)

	fmt.Println("SVG:", svg)
	fmt.Printf("Path: %d steps\n", len(path)-1)
}

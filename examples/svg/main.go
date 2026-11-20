// Example: Generate SVG visualization of a weighted grid with path overlay.
package main

import (
	"fmt"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func main() {
	matrix := [][]int32{
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0},
	}

	g := grid.NewOrthogonalGrid(matrix)
	m := g.(grid.MutableGrid)

	m.SetWeightAt(2, 0, 5.0)
	m.SetWeightAt(3, 0, 5.0)
	m.SetWeightAt(2, 1, 5.0)
	m.SetWeightAt(3, 1, 5.0)

	for x := 0; x < 7; x++ {
		m.SetWeightAt(int32(x), 5, 0.3)
		m.SetWeightAt(int32(x), 0, 0.3)
	}
	for y := 0; y < 6; y++ {
		m.SetWeightAt(0, int32(y), 0.3)
		m.SetWeightAt(6, int32(y), 0.3)
	}

	f := finder.NewAStarFinder()
	path, err := f.FindPath(0, 0, 6, 5, g, nil)
	if err != nil {
		fmt.Println("Search failed:", err)
		return
	}

	svg := g.RenderSVG(nil, path)

	fmt.Println("SVG:", svg)
	fmt.Printf("Path: %d steps\n", len(path)-1)
}

// Example: Weighted terrain pathfinding.
// Shows how per-tile weights affect A* path choice vs the shortest path.
package main

import (
	"fmt"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func main() {
	// 10×8 grid with varied terrain weights:
	//   Road (0.3)   — perimeter, cheap
	//   Grass (1.0)  — baseline
	//   Swamp (5.0)  — expensive center
	matrix := grid.NewMatrix(10, 8)

	g := grid.NewOrthogonalGrid(matrix)

	// Terrain weights
	weights := [][]float64{
		{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
		{0.3, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0, 1.0, 0.3},
		{0.3, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 0.3},
		{0.3, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 0.3},
		{0.3, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 0.3},
		{0.3, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 0.3},
		{0.3, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0, 1.0, 0.3},
		{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
	}

	for y := range weights {
		for x := range weights[y] {
			g.SetWeightAt(x, y, weights[y][x])
		}
	}

	// A* WITH weight awareness — avoids swamp by taking the perimeter
	fWeighted := finder.NewAStarFinder()
	pathWeighted := fWeighted.FindPath(1, 0, 7, 7, g)

	// A* WITHOUT weights (all tiles cost 1) — cuts through swamp
	gPlain := grid.NewOrthogonalGrid(matrix)
	fPlain := finder.NewAStarFinder()
	pathPlain := fPlain.FindPath(1, 0, 7, 7, gPlain)

	fmt.Printf("With weights:    %d steps, cost=%.1f\n", len(pathWeighted)-1, finder.PathLength(pathWeighted))
	fmt.Printf("Without weights: %d steps, cost=%.1f\n\n", len(pathPlain)-1, finder.PathLength(pathPlain))

	fmt.Println("Weighted path (detours around swamp):")
	for _, p := range pathWeighted {
		w := g.GetWeightAt(p[0], p[1])
		label := "."
		switch {
		case w <= 0.3:
			label = "R" // road
		case w <= 1.0:
			label = "G" // grass
		case w <= 5.0:
			label = "S" // swamp
		}
		fmt.Printf("  (%d,%d) [%s w=%.1f]\n", p[0], p[1], label, w)
	}

	fmt.Println("\nUnweighted path (shortest, through swamp):")
	for _, p := range pathPlain {
		w := g.GetWeightAt(p[0], p[1])
		label := "."
		switch {
		case w <= 0.3:
			label = "R"
		case w <= 1.0:
			label = "G"
		case w <= 5.0:
			label = "S"
		}
		fmt.Printf("  (%d,%d) [%s w=%.1f]\n", p[0], p[1], label, w)
	}
}
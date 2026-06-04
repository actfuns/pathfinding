// Example: Basic A* pathfinding on a grid with obstacles.
package main

import (
	"fmt"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func main() {
	// Create a 5x5 grid with a wall in the middle
	matrix := [][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0},
	}

	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewAStarFinder(finder.WithDiagonal(finder.DiagonalNever))

	path := f.FindPath(0, 0, 4, 4, g)
	if path == nil {
		fmt.Println("No path found!")
		return
	}

	fmt.Printf("Path found: %d steps\n", len(path)-1)
	for _, p := range path {
		fmt.Printf("  (%d, %d)\n", p[0], p[1])
	}
}

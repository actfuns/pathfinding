// Command compare runs all finders against test scenarios and validates paths.
// It also outputs Go paths in a format comparable with the JS reference output.
package main

import (
	"encoding/json"
	"fmt"
	"os"

	"github.com/parasol/pathfinding/core"
	"github.com/parasol/pathfinding/finder"
)

var scenarios = []struct {
	startX, startY, endX, endY int
	matrix                     [][]int
}{
	{
		startX: 0, startY: 0, endX: 1, endY: 1,
		matrix: [][]int{
			{0, 0},
			{1, 0},
		},
	},
	{
		startX: 1, startY: 1, endX: 4, endY: 4,
		matrix: [][]int{
			{0, 0, 0, 0, 0},
			{1, 0, 1, 1, 0},
			{1, 0, 1, 0, 0},
			{0, 1, 0, 0, 0},
			{1, 0, 1, 1, 0},
			{0, 0, 1, 0, 0},
		},
	},
	{
		startX: 0, startY: 3, endX: 3, endY: 3,
		matrix: [][]int{
			{0, 0, 0, 0, 0},
			{0, 0, 1, 1, 0},
			{0, 0, 1, 0, 0},
			{0, 0, 1, 0, 0},
			{1, 0, 1, 1, 0},
			{0, 0, 0, 0, 0},
		},
	},
	{
		startX: 4, startY: 4, endX: 19, endY: 19,
		matrix: func() [][]int {
			m := make([][]int, 20)
			for i := range m {
				m[i] = make([]int, 20)
			}
			return m
		}(),
	},
}

type finderDef struct {
	name   string
	finder core.Finder
}

func main() {
	finders := []finderDef{
		{"AStar", finder.NewAStarFinder(nil)},
		{"BestFirst", finder.NewBestFirstFinder(nil)},
		{"Dijkstra", finder.NewDijkstraFinder(nil)},
		{"BreadthFirst", finder.NewBreadthFirstFinder(nil)},
		{"BiAStar", finder.NewBiAStarFinder(nil)},
		{"BiBestFirst", finder.NewBiBestFirstFinder(nil)},
		{"BiDijkstra", finder.NewBiDijkstraFinder(nil)},
		{"BiBreadthFirst", finder.NewBiBreadthFirstFinder(nil)},
		{"IDAStar", finder.NewIDAStarFinder(nil)},
		{"JPFAtMostOne", finder.JumpPointFinder(&finder.FinderOptions{DiagonalMovement: core.DiagonalIfAtMostOneObstacle})},
		{"JPFNever", finder.JumpPointFinder(&finder.FinderOptions{DiagonalMovement: core.DiagonalNever})},
		{"JPFAlways", finder.JumpPointFinder(&finder.FinderOptions{DiagonalMovement: core.DiagonalAlways})},
		{"JPFOnlyWhenNoObstacles", finder.JumpPointFinder(&finder.FinderOptions{DiagonalMovement: core.DiagonalOnlyWhenNoObstacles})},
	}

	allPassed := true

	for _, f := range finders {
		for i, s := range scenarios {
			grid := core.NewGrid(s.matrix)
			path := f.finder.FindPath(s.startX, s.startY, s.endX, s.endY, grid)

			if path == nil || len(path) == 0 {
				fmt.Printf("FAIL: %s scenario %d: no path found\n", f.name, i+1)
				allPassed = false
				continue
			}

			if path[0][0] != s.startX || path[0][1] != s.startY {
				fmt.Printf("FAIL: %s scenario %d: start (%d,%d) != (%d,%d)\n",
					f.name, i+1, path[0][0], path[0][1], s.startX, s.startY)
				allPassed = false
				continue
			}

			if path[len(path)-1][0] != s.endX || path[len(path)-1][1] != s.endY {
				fmt.Printf("FAIL: %s scenario %d: end (%d,%d) != (%d,%d)\n",
					f.name, i+1, path[len(path)-1][0], path[len(path)-1][1], s.endX, s.endY)
				allPassed = false
				continue
			}

			valid := true
			for j := 0; j < len(path); j++ {
				x, y := path[j][0], path[j][1]
				if !grid.IsWalkableAt(x, y) {
					fmt.Printf("FAIL: %s scenario %d: step %d (%d,%d) is not walkable\n",
						f.name, i+1, j, x, y)
					valid = false
					break
				}
				if j > 0 {
					px, py := path[j-1][0], path[j-1][1]
					dx, dy := x-px, y-py
					if dx < -1 || dx > 1 || dy < -1 || dy > 1 || (dx == 0 && dy == 0) {
						fmt.Printf("FAIL: %s scenario %d: step %d->%d: (%d,%d)->(%d,%d) invalid move\n",
							f.name, i+1, j-1, j, px, py, x, y)
						valid = false
						break
					}
				}
			}
			if !valid {
				allPassed = false
				continue
			}
		}
	}

	// Output in JS-comparable format
	for _, f := range finders {
		for i, s := range scenarios {
			grid := core.NewGrid(s.matrix)
			path := f.finder.FindPath(s.startX, s.startY, s.endX, s.endY, grid)
			if path == nil {
				path = [][2]int{}
			}
			b, _ := json.Marshal(path)
			fmt.Printf("%s|%d|%s\n", f.name, i, string(b))
		}
	}

	if !allPassed {
		os.Exit(1)
	}
}

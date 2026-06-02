package grid

import (
	"math/rand"
	"strings"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

// gridForSVG is any grid type that can render SVG.
type gridForSVG interface {
	RenderSVG(path [][2]int, startX, startY, endX, endY int) string
}

// svgFinders lists all finder variants used in SVG rendering tests.
var svgFinders = []struct {
	name string
	f    finder.Finder
}{
	{"AStar", finder.NewAStarFinder()},
	{"Dijkstra", finder.NewDijkstraFinder()},
	{"BFS", finder.NewBreadthFirstFinder()},
	{"BestFirst", finder.NewBestFirstFinder()},
	{"BiAStar", finder.NewBiAStarFinder()},
	{"BiDijkstra", finder.NewBiDijkstraFinder()},
	{"BiBFS", finder.NewBiBreadthFirstFinder()},
	{"BiBestFirst", finder.NewBiBestFirstFinder()},
	{"IDAStar", finder.NewIDAStarFinder()},
	{"JPFNever", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))},
	{"JPFAlways", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalAlways))},
	{"JPFNoObstacles", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalOnlyWhenNoObstacles))},
	{"JPFAtMostOne", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalIfAtMostOneObstacle))},
}

// svgScenario defines one scenario for SVG rendering tests.
type svgScenario struct {
	name                       string
	startX, startY, endX, endY int
	matrix                     [][]int
}

// runSVGTest runs all svgScenario x svgFinder combinations and validates SVG output.
// Set NAVPATH_DUMP_SVG=1 to write SVG files to disk instead of validating.
func runSVGTest(t *testing.T, gridType string, scenarios []svgScenario, newGrid func(matrix [][]int) gridForSVG) {
	for _, s := range scenarios {
		for _, ft := range svgFinders {
			name := gridType + "/" + s.name + "_" + ft.name + ".svg"
			t.Run(name, func(t *testing.T) {
				g := newGrid(s.matrix)
				path := ft.f.FindPath(s.startX, s.startY, s.endX, s.endY, g.(finder.Grid))
				svg := g.RenderSVG(path, s.startX, s.startY, s.endX, s.endY)

				if !strings.Contains(svg, "<svg") {
					t.Error("SVG missing <svg tag")
				}
				if !strings.Contains(svg, "</svg>") {
					t.Error("SVG missing </svg>")
				}
				if !strings.Contains(svg, "#00cc44") {
					t.Error("SVG missing start marker")
				}
				if !strings.Contains(svg, "#cc0000") {
					t.Error("SVG missing end marker")
				}
				if path != nil && !strings.Contains(svg, "#0066cc") {
					t.Error("SVG missing path when path is present")
				}
			})
		}
	}
}

// generateGrid creates a height x width grid with random obstacles at the given probability.
func generateGrid(width, height int, obstacleProb float64) [][]int {
	m := make([][]int, height)
	rng := rand.New(rand.NewSource(42))
	for y := range m {
		row := make([]int, width)
		for x := range row {
			if rng.Float64() < obstacleProb {
				row[x] = 1
			}
		}
		m[y] = row
	}
	return m
}

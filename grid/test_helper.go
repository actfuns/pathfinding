package grid

import (
	"math/rand"
	"os"
	"strings"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

// gridForSVG is any grid type that can render SVG.
type gridForSVG interface {
	RenderSVG(cfg *SVGOpts, paths ...[][2]int) string
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
	{"JPSPlus", finder.NewJPSPlusFinder()},
	{"JPSPlusNever", finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))},
	{"JPSPlusNoObstacles", finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalOnlyWhenNoObstacles))},
	{"JPSPlusAtMostOne", finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalIfAtMostOneObstacle))},
}

// svgScenario defines one scenario for SVG rendering tests.
type svgScenario struct {
	name                       string
	startX, startY, endX, endY int
	matrix                     [][]int
	weights                    [][]float64 // optional per-tile weights
}

// runSVGTest runs all svgScenario x svgFinder combinations and validates SVG output.
// Set NAVPATH_DUMP_SVG=1 to write SVG files to disk instead of validating.
func runSVGTest(t *testing.T, gridType string, scenarios []svgScenario, newGrid func(matrix [][]int) gridForSVG) {
	for _, s := range scenarios {
		for _, ft := range svgFinders {
			name := gridType + "/" + s.name + "_" + ft.name + ".svg"
			t.Run(name, func(t *testing.T) {
				g := newGrid(s.matrix)

				// Apply optional per-tile weights
				if s.weights != nil {
					for y := range s.weights {
						for x := range s.weights[y] {
							if gw, ok := g.(interface{ SetWeightAt(int, int, float64) }); ok {
								gw.SetWeightAt(x, y, s.weights[y][x])
							}
						}
					}
				}
				if p, ok := ft.f.(interface{ Precompute(finder.Grid) }); ok {
					p.Precompute(g.(finder.Grid))
				}
				path := ft.f.FindPath(s.startX, s.startY, s.endX, s.endY, g.(finder.Grid))
				svg := g.RenderSVG(nil, path)

				if os.Getenv("NAVPATH_DUMP_SVG") != "" {
					fname := "testdata/svg/" + strings.ReplaceAll(strings.ReplaceAll(name, "/", "_"), " ", "_")
					os.MkdirAll("testdata/svg", 0755)
					_ = os.WriteFile(fname, []byte(svg), 0644)
				}

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

package grid

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/actfuns/navpath/finder"
)

// TestRenderSVG_Orthogonal generates SVG renders for each test scenario
// with each finder. Set NAVPATH_DUMP_SVG=1 to generate SVG files.
func TestRenderSVG_Orthogonal(t *testing.T) {
	scenarios := []struct {
		name                       string
		startX, startY, endX, endY int
		matrix                     [][]int
	}{
		{
			name: "simple 2x2", startX: 0, startY: 0, endX: 1, endY: 1,
			matrix: [][]int{{0, 0}, {1, 0}},
		},
		{
			name: "5x6 maze", startX: 1, startY: 1, endX: 4, endY: 4,
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
			name: "all open 10x10", startX: 0, startY: 0, endX: 9, endY: 9,
			matrix: func() [][]int {
				m := make([][]int, 10)
				for i := range m {
					m[i] = make([]int, 10)
				}
				return m
			}(),
		},
	}

	runSVGTest(t, "orthogonal", scenarios, func(matrix [][]int) gridForSVG {
		return NewOrthogonalGrid(matrix)
	})
}

// TestRenderSVG_Hex generates SVG renders for hex grids with each finder.
func TestRenderSVG_Hex(t *testing.T) {
	scenarios := []struct {
		name                       string
		startX, startY, endX, endY int
		matrix                     [][]int
	}{
		{
			name: "simple 3x3", startX: 0, startY: 0, endX: 2, endY: 2,
			matrix: [][]int{
				{0, 0, 0},
				{0, 1, 0},
				{0, 0, 0},
			},
		},
		{
			name: "4x3 with obstacles", startX: 0, startY: 0, endX: 3, endY: 2,
			matrix: [][]int{
				{0, 0, 0, 0},
				{0, 1, 1, 0},
				{0, 0, 0, 0},
			},
		},
		{
			name: "all open 8x6", startX: 0, startY: 0, endX: 7, endY: 5,
			matrix: func() [][]int {
				m := make([][]int, 6)
				for i := range m {
					m[i] = make([]int, 8)
				}
				return m
			}(),
		},
	}

	t.Run("pointy-top", func(t *testing.T) {
		runSVGTest(t, "hex_pointy", scenarios, func(matrix [][]int) gridForSVG {
			return NewHexGrid(matrix, 64, 64, 32, false, false)
		})
	})

	t.Run("flat-top", func(t *testing.T) {
		runSVGTest(t, "hex_flat", scenarios, func(matrix [][]int) gridForSVG {
			return NewHexGrid(matrix, 64, 64, 32, true, false)
		})
	})
}

// TestRenderSVG_Staggered generates SVG renders for staggered grids with each finder.
func TestRenderSVG_Staggered(t *testing.T) {
	scenarios := []struct {
		name                       string
		startX, startY, endX, endY int
		matrix                     [][]int
	}{
		{
			name: "simple 3x3", startX: 0, startY: 0, endX: 2, endY: 2,
			matrix: [][]int{
				{0, 0, 0},
				{0, 1, 0},
				{0, 0, 0},
			},
		},
		{
			name: "4x3 with obstacles", startX: 0, startY: 0, endX: 3, endY: 2,
			matrix: [][]int{
				{0, 0, 0, 0},
				{0, 1, 1, 0},
				{0, 0, 0, 0},
			},
		},
		{
			name: "all open 8x6", startX: 0, startY: 0, endX: 7, endY: 5,
			matrix: func() [][]int {
				m := make([][]int, 6)
				for i := range m {
					m[i] = make([]int, 8)
				}
				return m
			}(),
		},
	}

	runSVGTest(t, "staggered", scenarios, func(matrix [][]int) gridForSVG {
		return NewStaggeredGrid(matrix, 64, 32, "y", "odd")
	})
}

func TestRenderSVG_NoPath(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
		{0, 1, 0},
		{0, 1, 0},
		{0, 1, 0},
	})
	svg := g.RenderSVG(nil, 0, 0, 2, 0)
	if !strings.Contains(svg, "<svg") {
		t.Error("SVG missing <svg tag")
	}
	if strings.Contains(svg, "#0066cc") {
		t.Error("SVG should not contain path when nil path given")
	}
}

// --- helpers ---

// gridForSVG is any grid type that can render SVG.
type gridForSVG interface {
	RenderSVG(path [][2]int, startX, startY, endX, endY int) string
}

// runSVGTest runs all scenario x finder combinations and dumps SVG if NAVPATH_DUMP_SVG is set.
func runSVGTest(t *testing.T, gridType string, scenarios []struct {
	name                       string
	startX, startY, endX, endY int
	matrix                     [][]int
}, newGrid func(matrix [][]int) gridForSVG) {
	finders := []struct {
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
		{"JPFNever", finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))},
		{"JPFAlways", finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"JPFNoObstacles", finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalOnlyWhenNoObstacles))},
		{"JPFAtMostOne", finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalIfAtMostOneObstacle))},
	}

	dump := os.Getenv("NAVPATH_DUMP_SVG")
	if dump == "" {
		dump = "0"
	}

	for _, s := range scenarios {
		for _, ft := range finders {
			name := gridType + "/" + s.name + "_" + ft.name + ".svg"
			t.Run(name, func(t *testing.T) {
				g := newGrid(s.matrix)
				path := ft.f.FindPath(s.startX, s.startY, s.endX, s.endY, g.(finder.Grid))
				svg := g.RenderSVG(path, s.startX, s.startY, s.endX, s.endY)

				if dump != "0" {
					dir := filepath.Join("testdata", "svg", gridType)
					if err := os.MkdirAll(dir, 0755); err != nil {
						t.Fatal(err)
					}
					if err := os.WriteFile(filepath.Join(dir, s.name+"_"+ft.name+".svg"), []byte(svg), 0644); err != nil {
						t.Fatal(err)
					}
					return
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

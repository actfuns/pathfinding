package finder_test

import (
	"errors"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

// testScenarios are shared across all finder tests.
var testScenarios = []struct {
	name                       string
	startX, startY, endX, endY int32
	matrix                     [][]int32
}{
	{
		name: "simple 2x2", startX: 0, startY: 0, endX: 1, endY: 1,
		matrix: [][]int32{{0, 0}, {1, 0}},
	},
	{
		name: "5x6 maze", startX: 1, startY: 1, endX: 4, endY: 4,
		matrix: [][]int32{
			{0, 0, 0, 0, 0},
			{1, 0, 1, 1, 0},
			{1, 0, 1, 0, 0},
			{0, 1, 0, 0, 0},
			{1, 0, 1, 1, 0},
			{0, 0, 1, 0, 0},
		},
	},
	{
		name: "5x6 maze B", startX: 0, startY: 3, endX: 3, endY: 3,
		matrix: [][]int32{
			{0, 0, 0, 0, 0},
			{0, 0, 1, 1, 0},
			{0, 0, 1, 0, 0},
			{0, 0, 1, 0, 0},
			{1, 0, 1, 1, 0},
			{0, 0, 0, 0, 0},
		},
	},
	{
		name: "large open 20x20", startX: 4, startY: 4, endX: 19, endY: 19,
		matrix: func() [][]int32 {
			m := make([][]int32, 20)
			for i := range m {
				m[i] = make([]int32, 20)
			}
			return m
		}(),
	},
}

func testFinderAgainstJS(t *testing.T, name string, newFinder func() finder.Finder, want [][][2]int32) {
	t.Helper()
	for i, s := range testScenarios {
		g := grid.NewOrthogonalGrid(s.matrix)
		f := newFinder()
		path, _ := f.FindPath(s.startX, s.startY, s.endX, s.endY, g, nil)
		if path == nil {
			t.Errorf("%s scenario %d: path not found", name, i+1)
			continue
		}
		if !pathEqual(path, want[i]) {
			t.Errorf("%s scenario %d:\n  got:  %v\n  want: %v", name, i+1, path, want[i])
		}
	}
}

func pathEqual(a, b [][2]int32) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// --- Edge-case tests shared across finders ---

func TestFinderUnreachablePath(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0, 1, 0}})
	finders := []struct {
		name string
		f    finder.Finder
	}{
		{"AStar", finder.NewAStarFinder()},
		{"Dijkstra", finder.NewDijkstraFinder()},
		{"BreadthFirst", finder.NewBreadthFirstFinder()},
		{"BestFirst", finder.NewBestFirstFinder()},
		{"BiAStar", finder.NewBiAStarFinder()},
		{"BiDijkstra", finder.NewBiDijkstraFinder()},
		{"BiBreadthFirst", finder.NewBiBreadthFirstFinder()},
		{"BiBestFirst", finder.NewBiBestFirstFinder()},
		{"JPFNever", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))},
	}
	for _, ft := range finders {
		t.Run(ft.name, func(t *testing.T) {
			path, _ := ft.f.FindPath(0, 0, 2, 0, g, nil)
			if path != nil {
				t.Errorf("expected nil for unreachable, got %v", path)
			}
		})
	}
}

func TestFinderUnwalkableEnd(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0, 1}})
	finders := []struct {
		name string
		f    finder.Finder
	}{
		{"AStar", finder.NewAStarFinder()},
		{"Dijkstra", finder.NewDijkstraFinder()},
		{"BreadthFirst", finder.NewBreadthFirstFinder()},
		{"BestFirst", finder.NewBestFirstFinder()},
		{"BiAStar", finder.NewBiAStarFinder()},
		{"BiDijkstra", finder.NewBiDijkstraFinder()},
		{"BiBreadthFirst", finder.NewBiBreadthFirstFinder()},
		{"BiBestFirst", finder.NewBiBestFirstFinder()},
		{"JPFNever", finder.NewJumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))},
	}
	for _, ft := range finders {
		t.Run(ft.name, func(t *testing.T) {
			path, err := ft.f.FindPath(0, 0, 1, 0, g, nil)
			if !errors.Is(err, finder.ErrNotWalkable) {
				t.Fatalf("FindPath error = %v, want ErrNotWalkable", err)
			}
			if path != nil {
				t.Errorf("expected nil path for unwalkable end, got %v", path)
			}
		})
	}
}

func TestFinderSingleCell(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0}})
	finders := []finder.Finder{
		finder.NewAStarFinder(),
		finder.NewDijkstraFinder(),
		finder.NewBreadthFirstFinder(),
	}
	for i, f := range finders {
		path, _ := f.FindPath(0, 0, 0, 0, g, nil)
		if path == nil {
			t.Errorf("finder %d (%T): expected path for single cell start==end", i, f)
		}
	}
}

func TestFinder1xN(t *testing.T) {
	for i, f := range []finder.Finder{
		finder.NewAStarFinder(),
		finder.NewDijkstraFinder(),
		finder.NewBreadthFirstFinder(),
	} {
		g := grid.NewOrthogonalGrid([][]int32{{0, 0, 0}})
		path, _ := f.FindPath(0, 0, 2, 0, g, nil)
		if path == nil {
			t.Errorf("finder %d (%T): expected path on 1xN grid", i, f)
		}
	}
}

// Every finder must reject an endpoint outside the grid with ErrOutOfBounds
// instead of letting GetNodeAt panic on an out-of-range index.
func TestAllFindersRejectOutOfBounds(t *testing.T) {
	m := grid.NewMatrix(16, 16)
	for y := range m {
		for x := range m[y] {
			m[y][x] = 0
		}
	}
	g := grid.NewOrthogonalGrid(m)

	jpsPlus := finder.NewJPSPlusFinder()
	_ = jpsPlus.Precompute(g)
	for name, f := range map[string]finder.Finder{
		"AStar":          finder.NewAStarFinder(),
		"BreadthFirst":   finder.NewBreadthFirstFinder(),
		"Dijkstra":       finder.NewDijkstraFinder(),
		"BestFirst":      finder.NewBestFirstFinder(),
		"IDAStar":        finder.NewIDAStarFinder(),
		"BiAStar":        finder.NewBiAStarFinder(),
		"BiBreadthFirst": finder.NewBiBreadthFirstFinder(),
		"JPSNever":       finder.NewJumpPointFinder(),
		"JPSPlus":        jpsPlus,
	} {
		t.Run(name, func(t *testing.T) {
			for _, tc := range []struct{ sx, sy, ex, ey int32 }{
				{0, 0, 999, 999}, // end out of bounds
				{999, 999, 0, 0}, // start out of bounds
				{-1, 0, 1, 1},    // negative start
				{0, 0, 0, -1},    // negative end
			} {
				path, err := f.FindPath(tc.sx, tc.sy, tc.ex, tc.ey, g, nil)
				if !errors.Is(err, finder.ErrOutOfBounds) {
					t.Errorf("FindPath(%d,%d,%d,%d) = (%v, %v), want ErrOutOfBounds",
						tc.sx, tc.sy, tc.ex, tc.ey, path, err)
				}
				if path != nil {
					t.Errorf("FindPath(%d,%d,%d,%d) must not return a path", tc.sx, tc.sy, tc.ex, tc.ey)
				}
			}
		})
	}
}

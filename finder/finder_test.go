package finder_test

import (
	"testing"

	"github.com/actfuns/navpath/finder"
	"github.com/actfuns/navpath/grid"
)

// testScenarios are shared across all finder tests.
var testScenarios = []struct {
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
		name: "5x6 maze B", startX: 0, startY: 3, endX: 3, endY: 3,
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
		name: "large open 20x20", startX: 4, startY: 4, endX: 19, endY: 19,
		matrix: func() [][]int {
			m := make([][]int, 20)
			for i := range m {
				m[i] = make([]int, 20)
			}
			return m
		}(),
	},
}

func testFinderAgainstJS(t *testing.T, name string, newFinder func() finder.Finder, want [][][2]int) {
	t.Helper()
	for i, s := range testScenarios {
		g := grid.NewOrthogonalGrid(s.matrix)
		f := newFinder()
		path := f.FindPath(s.startX, s.startY, s.endX, s.endY, g)
		if path == nil {
			t.Errorf("%s scenario %d: path not found", name, i+1)
			continue
		}
		if !pathEqual(path, want[i]) {
			t.Errorf("%s scenario %d:\n  got:  %v\n  want: %v", name, i+1, path, want[i])
		}
	}
}

func pathEqual(a, b [][2]int) bool {
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
	g := grid.NewOrthogonalGrid([][]int{{0, 1, 0}})
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
		{"JPFNever", finder.JumpPointFinder(finder.WithDiagonal(finder.DiagonalNever))},
	}
	for _, ft := range finders {
		t.Run(ft.name, func(t *testing.T) {
			path := ft.f.FindPath(0, 0, 2, 0, g)
			if path != nil {
				t.Errorf("expected nil for unreachable, got %v", path)
			}
		})
	}
}

func TestFinderSingleCell(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{{0}})
	finders := []finder.Finder{
		finder.NewAStarFinder(),
		finder.NewDijkstraFinder(),
		finder.NewBreadthFirstFinder(),
	}
	for i, f := range finders {
		path := f.FindPath(0, 0, 0, 0, g)
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
		g := grid.NewOrthogonalGrid([][]int{{0, 0, 0}})
		path := f.FindPath(0, 0, 2, 0, g)
		if path == nil {
			t.Errorf("finder %d (%T): expected path on 1xN grid", i, f)
		}
	}
}

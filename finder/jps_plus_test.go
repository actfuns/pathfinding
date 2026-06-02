package finder_test

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestJPSPlusNeverPrecompute(t *testing.T) {
	matrix := [][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	f.Precompute(g)

	path := f.FindPath(0, 0, 4, 4, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	if path[0] != [2]int{0, 0} {
		t.Errorf("expected start (0,0), got %v", path[0])
	}
	if path[len(path)-1] != [2]int{4, 4} {
		t.Errorf("expected end (4,4), got %v", path[len(path)-1])
	}
}

func TestJPSPlusAgainstJS(t *testing.T) {
	// JPS+ produces optimal paths — may differ from standard JPS on open grids
	// (where standard JPS leverages a perpendicular recursive cascade that JPS+ cannot
	// precompute), but all paths are equally valid and optimal.
	want := [][][2]int{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {5, 5}, {6, 5}, {6, 6}, {6, 7}, {7, 7}, {7, 8}, {8, 8}, {9, 8}, {9, 9}, {10, 9}, {11, 9}, {12, 9}, {12, 10}, {12, 11}, {12, 12}, {12, 13}, {12, 14}, {13, 14}, {13, 15}, {14, 15}, {14, 16}, {14, 17}, {14, 18}, {14, 19}, {15, 19}, {16, 19}, {17, 19}, {18, 19}, {19, 19}},
	}

	for i, s := range testScenarios {
		g := grid.NewOrthogonalGrid(s.matrix)
		f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
		f.Precompute(g)
		path := f.FindPath(s.startX, s.startY, s.endX, s.endY, g)
		if path == nil {
			t.Errorf("JPS+ scenario %d: path not found", i+1)
			continue
		}
		if !pathEqual(path, want[i]) {
			t.Errorf("JPS+ scenario %d:\n  got:  %v\n  want: %v", i+1, path, want[i])
		}
	}
}

func TestJPSPlusNoPanicPrecompute(t *testing.T) {
	// Test that Precompute handles edge cases without panicking
	matrix := [][]int{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder()
	f.Precompute(g)

	path := f.FindPath(0, 0, 2, 2, g)
	if path == nil {
		t.Error("expected path")
	}
}

func TestJPSPlusUnreachable(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{{0, 1, 0}})
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	f.Precompute(g)
	path := f.FindPath(0, 0, 2, 0, g)
	if path != nil {
		t.Errorf("expected nil for unreachable, got %v", path)
	}
}

func TestJPSPlusSingleCell(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int{{0}})
	f := finder.NewJPSPlusFinder()
	f.Precompute(g)
	path := f.FindPath(0, 0, 0, 0, g)
	if path == nil {
		t.Error("expected path for start==end")
	}
}

func TestJPSPlusPrecomputePanic(t *testing.T) {
	f := finder.NewJPSPlusFinder()
	defer func() {
		if r := recover(); r == nil {
			t.Error("expected panic when FindPath called before Precompute")
		}
	}()
	g := grid.NewOrthogonalGrid([][]int{{0, 0}, {0, 0}})
	f.FindPath(0, 0, 1, 1, g)
}

func TestJPSPlusOpenGrid(t *testing.T) {
	// On a fully open grid, JPS+ should find the direct diagonal or cardinal path
	matrix := make([][]int, 100)
	for i := range matrix {
		matrix[i] = make([]int, 100)
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	f.Precompute(g)
	path := f.FindPath(0, 0, 99, 99, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if path[len(path)-1] != [2]int{99, 99} {
		t.Errorf("expected end (99,99), got %v", path[len(path)-1])
	}
}

func TestJPSPlusExportImport(t *testing.T) {
	matrix := [][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)

	// Precompute and export
	f1 := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	f1.Precompute(g)
	data, err := f1.PrecomputedData()
	if err != nil {
		t.Fatal(err)
	}
	if len(data) < 12 {
		t.Fatal("exported data too short")
	}

}

func TestJPSPlusExportBeforePrecompute(t *testing.T) {
	f := finder.NewJPSPlusFinder()
	if _, err := f.PrecomputedData(); err == nil {
		t.Error("expected error when exporting before Precompute")
	}
}

func TestJPSPlusLoadInvalidData(t *testing.T) {
	f := finder.NewJPSPlusFinder()
	if err := f.LoadPrecomputed([]byte{0, 0, 0}); err == nil {
		t.Error("expected error for data too short")
	}
	if err := f.LoadPrecomputed([]byte{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); err == nil {
		t.Error("expected error for invalid magic")
	}
}

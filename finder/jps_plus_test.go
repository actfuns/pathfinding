package finder_test

import (
	"errors"
	"math/rand"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestJPSPlusDiagonalAlwaysMatchesAStar(t *testing.T) {
	const size = 16
	rng := rand.New(rand.NewSource(31))
	option := finder.WithDiagonal(finder.DiagonalAlways)
	astar := finder.NewAStarFinder(option)

	for _, density := range []int{0, 10, 30, 55} {
		for sample := 0; sample < 8; sample++ {
			matrix := make([][]int32, size)
			for y := range matrix {
				matrix[y] = make([]int32, size)
				for x := range matrix[y] {
					if rng.Intn(100) < density {
						matrix[y][x] = 1
					}
				}
			}
			g := grid.NewOrthogonalGrid(matrix)
			plus := finder.NewJPSPlusFinder(option)
			if err := plus.Precompute(g); err != nil {
				t.Fatal(err)
			}
			data, err := plus.PrecomputedData()
			if err != nil {
				t.Fatal(err)
			}
			reloaded := finder.NewJPSPlusFinder(option)
			if err := reloaded.LoadPrecomputed(data); err != nil {
				t.Fatal(err)
			}

			for query := 0; query < 100; query++ {
				start := [2]int32{rng.Int31n(size), rng.Int31n(size)}
				end := [2]int32{rng.Int31n(size), rng.Int31n(size)}
				if !g.IsWalkableAt(end[0], end[1]) {
					continue
				}
				want, err := astar.FindPath(start[0], start[1], end[0], end[1], g, nil)
				if err != nil {
					t.Fatal(err)
				}
				got, err := reloaded.FindPath(start[0], start[1], end[0], end[1], g, nil)
				if err != nil || (got == nil) != (want == nil) {
					t.Fatalf("density %d map %d, %v to %v: got %v, err %v, want %v", density, sample, start, end, got, err, want)
				}
				if got == nil {
					continue
				}
				if got[0] != start || got[len(got)-1] != end {
					t.Fatalf("wrong endpoints for %v to %v: %v", start, end, got)
				}
				steps := func(path [][2]int32) (cardinal, diagonal int) {
					for i := 1; i < len(path); i++ {
						prev, next := path[i-1], path[i]
						dx, dy := finder.AbsInt(next[0]-prev[0]), finder.AbsInt(next[1]-prev[1])
						if max(dx, dy) != 1 || !g.IsWalkableAt(next[0], next[1]) {
							t.Fatalf("invalid step %v to %v", prev, next)
						}
						if dx != 0 && dy != 0 {
							diagonal++
						} else {
							cardinal++
						}
					}
					return cardinal, diagonal
				}
				gotCardinal, gotDiagonal := steps(got)
				wantCardinal, wantDiagonal := steps(want)
				if gotCardinal != wantCardinal || gotDiagonal != wantDiagonal {
					t.Fatalf("density %d map %d, %v to %v: got %d cardinal and %d diagonal steps, want %d and %d",
						density, sample, start, end, gotCardinal, gotDiagonal, wantCardinal, wantDiagonal)
				}
			}
		}
	}
}

func TestJPSPlusBlockedStart(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{1, 0, 0}})
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalAlways))
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	path, err := f.FindPath(0, 0, 2, 0, g, nil)
	if err != nil || len(path) != 3 || path[1] != [2]int32{1, 0} {
		t.Fatalf("got %v, %v, want path out of blocked start", path, err)
	}
}

func TestJPSPlusRejectsMismatchedWalkability(t *testing.T) {
	option := finder.WithDiagonal(finder.DiagonalAlways)
	open := grid.NewOrthogonalGrid([][]int32{{0, 0, 0, 0}})
	blocked := grid.NewOrthogonalGrid([][]int32{{0, 0, 1, 0}})
	f := finder.NewJPSPlusFinder(option)
	if err := f.Precompute(open); err != nil {
		t.Fatal(err)
	}
	data, err := f.PrecomputedData()
	if err != nil {
		t.Fatal(err)
	}
	loaded := finder.NewJPSPlusFinder(option)
	if err := loaded.LoadPrecomputed(data); err != nil {
		t.Fatal(err)
	}
	for _, search := range []*finder.JPSPlusFinder{f, loaded} {
		if path, err := search.FindPath(0, 0, 3, 0, blocked, nil); path != nil || !errors.Is(err, finder.ErrPrecomputedMismatch) {
			t.Fatalf("different map: got %v, %v, want ErrPrecomputedMismatch", path, err)
		}
	}
	edited, err := open.CloneWith(func(g grid.MutableGrid) { g.SetWalkableAt(2, 0, false) })
	if err != nil {
		t.Fatal(err)
	}
	if path, err := loaded.FindPath(0, 0, 3, 0, edited, nil); path != nil || !errors.Is(err, finder.ErrPrecomputedMismatch) {
		t.Fatalf("edited clone: got %v, %v, want ErrPrecomputedMismatch", path, err)
	}
	identical := grid.NewOrthogonalGrid([][]int32{{0, 0, 0, 0}})
	if path, err := loaded.FindPath(0, 0, 3, 0, identical, nil); len(path) != 4 || err != nil {
		t.Fatalf("same layout: got %v, %v, want a path", path, err)
	}
}

func TestJPSPlusNeverPrecompute(t *testing.T) {
	matrix := [][]int32{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	_ = f.Precompute(g)
	path, _ := f.FindPath(0, 0, 4, 4, g, nil)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	if path[0] != [2]int32{0, 0} {
		t.Errorf("expected start (0,0), got %v", path[0])
	}
	if path[len(path)-1] != [2]int32{4, 4} {
		t.Errorf("expected end (4,4), got %v", path[len(path)-1])
	}
}

func TestJPSPlusAgainstJS(t *testing.T) {
	// JPS+ produces optimal paths — may differ from standard JPS on open grids
	// (where standard JPS leverages a perpendicular recursive cascade that JPS+ cannot
	// precompute), but all paths are equally valid and optimal.
	want := [][][2]int32{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {5, 5}, {6, 5}, {6, 6}, {6, 7}, {7, 7}, {7, 8}, {8, 8}, {9, 8}, {9, 9}, {10, 9}, {11, 9}, {12, 9}, {12, 10}, {12, 11}, {12, 12}, {12, 13}, {12, 14}, {13, 14}, {13, 15}, {14, 15}, {14, 16}, {14, 17}, {14, 18}, {14, 19}, {15, 19}, {16, 19}, {17, 19}, {18, 19}, {19, 19}},
	}

	for i, s := range testScenarios {
		g := grid.NewOrthogonalGrid(s.matrix)
		f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
		_ = f.Precompute(g)
		path, _ := f.FindPath(s.startX, s.startY, s.endX, s.endY, g, nil)
		if path == nil {
			t.Errorf("JPS+ scenario %d: path not found", i+1)
			continue
		}
		if len(path) != len(want[i]) {
			t.Errorf("JPS+ scenario %d: path length %d, want optimal length %d\n  got: %v", i+1, len(path), len(want[i]), path)
			continue
		}
		if path[0] != want[i][0] || path[len(path)-1] != want[i][len(want[i])-1] {
			t.Errorf("JPS+ scenario %d: endpoints %v..%v, want %v..%v", i+1, path[0], path[len(path)-1], want[i][0], want[i][len(want[i])-1])
		}
	}
}

func TestJPSPlusNoPanicPrecompute(t *testing.T) {
	// Test that Precompute handles edge cases without panicking
	matrix := [][]int32{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder()
	_ = f.Precompute(g)
	path, _ := f.FindPath(0, 0, 2, 2, g, nil)
	if path == nil {
		t.Error("expected path")
	}
}

func TestJPSPlusUnreachable(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0, 1, 0}})
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	_ = f.Precompute(g)
	path, _ := f.FindPath(0, 0, 2, 0, g, nil)
	if path != nil {
		t.Errorf("expected nil for unreachable, got %v", path)
	}
}

func TestJPSPlusSingleCell(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0}})
	f := finder.NewJPSPlusFinder()
	_ = f.Precompute(g)
	path, _ := f.FindPath(0, 0, 0, 0, g, nil)
	if path == nil {
		t.Error("expected path for start==end")
	}
}

func TestJPSPlusUnwalkableEnd(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0, 1}})
	f := finder.NewJPSPlusFinder()
	_ = f.Precompute(g)
	path, err := f.FindPath(0, 0, 1, 0, g, nil)
	if !errors.Is(err, finder.ErrNotWalkable) {
		t.Fatalf("FindPath error = %v, want ErrNotWalkable", err)
	}
	if path != nil {
		t.Errorf("expected nil path for unwalkable end, got %v", path)
	}
}

func TestJPSPlusPrecomputePanic(t *testing.T) {
	f := finder.NewJPSPlusFinder()
	defer func() {
		if r := recover(); r == nil {
			t.Error("expected panic when FindPath called before Precompute")
		}
	}()
	g := grid.NewOrthogonalGrid([][]int32{{0, 0}, {0, 0}})
	f.FindPath(0, 0, 1, 1, g, nil)
}

func TestJPSPlusOpenGrid(t *testing.T) {
	// On a fully open grid, JPS+ should find the direct diagonal or cardinal path
	matrix := make([][]int32, 100)
	for i := range matrix {
		matrix[i] = make([]int32, 100)
	}
	g := grid.NewOrthogonalGrid(matrix)
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	_ = f.Precompute(g)
	path, _ := f.FindPath(0, 0, 99, 99, g, nil)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if path[len(path)-1] != [2]int32{99, 99} {
		t.Errorf("expected end (99,99), got %v", path[len(path)-1])
	}
}

func TestJPSPlusExportImport(t *testing.T) {
	matrix := [][]int32{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)

	// Precompute and export
	f1 := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalNever))
	_ = f1.Precompute(g)
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

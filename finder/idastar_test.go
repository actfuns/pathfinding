package finder_test

import (
	"errors"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestIDAStarUsesDiscountedTerrainLowerBound(t *testing.T) {
	g := grid.NewOrthogonalGrid(grid.NewMatrix(5, 2))
	for x := int32(0); x < 5; x++ {
		g.(grid.MutableGrid).SetWeightAt(x, 1, 0.1)
	}
	path, err := finder.NewIDAStarFinder().FindPath(0, 0, 4, 0, g, nil)
	if err != nil || len(path) != 7 {
		t.Fatalf("got %v, %v, want the seven-point detour", path, err)
	}
	var cost float32
	for _, tile := range path[1:] {
		cost += g.GetWeightAt(tile[0], tile[1])
	}
	if math.Abs(float64(cost)-1.5) > 1e-6 {
		t.Fatalf("got cost %g, want 1.5", cost)
	}
}

func TestIDAStarAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "IDAStar", func() finder.Finder { return finder.NewIDAStarFinder() }, [][][2]int32{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {6, 4}, {7, 4}, {8, 4}, {9, 4}, {10, 4}, {11, 4}, {12, 4}, {13, 4}, {14, 4}, {15, 4}, {16, 4}, {17, 4}, {18, 4}, {19, 4}, {19, 5}, {19, 6}, {19, 7}, {19, 8}, {19, 9}, {19, 10}, {19, 11}, {19, 12}, {19, 13}, {19, 14}, {19, 15}, {19, 16}, {19, 17}, {19, 18}, {19, 19}},
	})
}

// A TimeLimit that expires must end the search, not spin forever: the cutoff can
// no longer advance once every call bails at once, so the outer loop needs to be
// told the search gave up.
func TestIDATimeLimitGivesUp(t *testing.T) {
	rng := rand.New(rand.NewSource(3))
	m := make([][]int32, 120)
	for y := range m {
		row := make([]int32, 120)
		for x := range row {
			if rng.Float64() < 0.2 {
				row[x] = 1
			}
		}
		m[y] = row
	}
	m[0][0], m[119][119] = 0, 0
	g := grid.NewOrthogonalGrid(m)

	for _, limit := range []float64{0.001, 0.01} {
		f := finder.NewIDAStarFinder(finder.WithTimeLimit(limit))
		type result struct {
			path [][2]int32
			err  error
		}
		out := make(chan result, 1)
		go func() {
			p, err := f.FindPath(0, 0, 119, 119, g, nil)
			out <- result{p, err}
		}()

		select {
		case r := <-out:
			if !errors.Is(r.err, finder.ErrTimeLimit) {
				t.Fatalf("TimeLimit=%v: want ErrTimeLimit, got path len=%d err=%v",
					limit, len(r.path), r.err)
			}
			if r.path != nil {
				t.Errorf("TimeLimit=%v: timed-out search must not return a path", limit)
			}
		case <-time.After(5 * time.Second):
			t.Fatalf("TimeLimit=%v: FindPath never returned (infinite loop)", limit)
		}
	}
}

// Giving up is not the same as "no path": the caller must be able to tell them
// apart, or a busy map would permanently strand a unit that has a valid route.
func TestNoPathIsNotAnError(t *testing.T) {
	matrix := [][]int32{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)

	f := finder.NewAStarFinder()
	path, err := f.FindPath(0, 0, 2, 2, g, nil)
	if err != nil {
		t.Fatalf("a failed search must report nil error, got %v", err)
	}
	if len(path) == 0 {
		t.Fatal("expected a path around the obstacle")
	}

	// Truly unreachable: nil path, nil error.
	blocked := [][]int32{{0, 1, 0}, {0, 1, 0}, {0, 1, 0}}
	gb := grid.NewOrthogonalGrid(blocked)
	path, err = f.FindPath(0, 0, 2, 0, gb, nil)
	if err != nil {
		t.Fatalf("unreachable target must be (nil, nil), got err=%v", err)
	}
	if path != nil {
		t.Fatalf("expected nil path for an unreachable target, got %v", path)
	}
}

// IDA* must terminate with (nil, nil) when no route exists instead of looping
// forever on cycles with an ever-increasing cutoff.
func TestIDAStarUnreachableTerminates(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 1, 0},
		{0, 1, 0},
		{0, 1, 0},
	})
	f := finder.NewIDAStarFinder()

	done := make(chan struct{})
	var path [][2]int32
	var err error
	go func() {
		path, err = f.FindPath(0, 0, 2, 0, g, nil)
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("FindPath never returned for an unreachable target")
	}
	if err != nil {
		t.Fatalf("unreachable target must be (nil, nil), got err=%v", err)
	}
	if path != nil {
		t.Fatalf("expected nil path, got %v", path)
	}
}

// Routes longer than the old fixed 512-entry buffer must work.
func TestIDAStarLongCorridor(t *testing.T) {
	const n = 600
	row := make([]int32, n)
	g := grid.NewOrthogonalGrid([][]int32{row})
	f := finder.NewIDAStarFinder()

	path, err := f.FindPath(0, 0, n-1, 0, g, nil)
	if err != nil {
		t.Fatalf("FindPath error: %v", err)
	}
	if len(path) != n {
		t.Fatalf("path length = %d, want %d", len(path), n)
	}
	if path[len(path)-1] != [2]int32{n - 1, 0} {
		t.Errorf("path ends at %v, want (%d,0)", path[len(path)-1], n-1)
	}
}

// (0,0) is a real coordinate, not a terminator: a path ending there must
// include it.
func TestIDAStarEndAtOrigin(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{{0, 0, 0}})
	f := finder.NewIDAStarFinder()

	path, err := f.FindPath(2, 0, 0, 0, g, nil)
	if err != nil {
		t.Fatalf("FindPath error: %v", err)
	}
	want := [][2]int32{{2, 0}, {1, 0}, {0, 0}}
	if !pathEqual(path, want) {
		t.Errorf("path = %v, want %v", path, want)
	}
}

// Regression: IDA* recurses, so each depth needs its own neighbor buffer.
// Sharing one buffer made a parent iterate a list that a child had overwritten,
// producing paths with non-adjacent steps.
func TestIDAStarRecursiveNeighborBuffers(t *testing.T) {
	matrix := [][]int32{
		{0, 1, 0},
		{0, 0, 0},
		{0, 0, 0},
		{1, 0, 1},
		{0, 0, 0},
		{0, 1, 0},
		{1, 1, 0},
	}
	g := grid.NewOrthogonalGrid(matrix)

	for _, diag := range []finder.DiagonalMovement{
		finder.DiagonalNever,
		finder.DiagonalIfAtMostOneObstacle,
		finder.DiagonalOnlyWhenNoObstacles,
		finder.DiagonalAlways,
	} {
		opts := []finder.Option{finder.WithDiagonal(diag)}
		astar := finder.NewAStarFinder(opts...)
		ida := finder.NewIDAStarFinder(opts...)

		wantPath, wantErr := astar.FindPath(2, 0, 2, 4, g, nil)
		gotPath, gotErr := ida.FindPath(2, 0, 2, 4, g, nil)
		if (wantErr != nil) != (gotErr != nil) {
			t.Fatalf("diag=%v: error mismatch: a=%v i=%v", diag, wantErr, gotErr)
		}
		if len(wantPath) == 0 || len(gotPath) == 0 {
			continue
		}
		for i := 1; i < len(gotPath); i++ {
			dx := gotPath[i][0] - gotPath[i-1][0]
			if dx < 0 {
				dx = -dx
			}
			dy := gotPath[i][1] - gotPath[i-1][1]
			if dy < 0 {
				dy = -dy
			}
			if dx > 1 || dy > 1 || (dx == 0 && dy == 0) {
				t.Fatalf("diag=%v: non-adjacent step %v -> %v in %v",
					diag, gotPath[i-1], gotPath[i], gotPath)
			}
		}
		if wc, gc := tilePathCost(wantPath), tilePathCost(gotPath); wc != gc {
			t.Errorf("diag=%v: cost A*=%.4f IDA*=%.4f\nA=%v\nI=%v",
				diag, wc, gc, wantPath, gotPath)
		}
	}
}

func tilePathCost(path [][2]int32) float64 {
	var c float64
	for i := 1; i < len(path); i++ {
		if path[i][0] == path[i-1][0] || path[i][1] == path[i-1][1] {
			c++
		} else {
			c += 1.4142135623730951
		}
	}
	return c
}

package finder_test

import (
	"errors"
	"math"
	"math/rand"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

// JPS/JPS+ assume uniform step costs and ignore tile weights. They return the
// step-shortest path; A*/Dijkstra are the cost-aware options.
func TestJPSIgnoresTileWeights(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})
	g.SetWeightAt(2, 1, 100)

	bfs, _ := finder.NewBreadthFirstFinder().FindPath(0, 1, 4, 1, g, nil)
	if len(bfs) != 5 {
		t.Fatalf("BFS step path len=%d, want 5", len(bfs))
	}
	astar, _ := finder.NewAStarFinder().FindPath(0, 1, 4, 1, g, nil)
	if len(astar) == len(bfs) {
		t.Fatalf("A* should detour around the expensive tile: %v", astar)
	}

	jpsPath, err := finder.NewJumpPointFinder().FindPath(0, 1, 4, 1, g, nil)
	if err != nil {
		t.Fatalf("JPS: %v", err)
	}
	plus := finder.NewJPSPlusFinder()
	_ = plus.Precompute(g)
	plusPath, err := plus.FindPath(0, 1, 4, 1, g, nil)
	if err != nil {
		t.Fatalf("JPS+: %v", err)
	}

	for name, p := range map[string][][2]int32{"JPS": jpsPath, "JPS+": plusPath} {
		if len(p) != len(bfs) {
			t.Errorf("%s steps=%d, want %d (tile weights must be ignored)\nBFS=%v\n%s=%v",
				name, len(p), len(bfs), bfs, name, p)
		}
	}
}

// JPS+ must not cut corners when the configured rule forbids it.
func TestJPSPlusRespectsDiagonalRules(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 1},
		{1, 0},
	})

	for _, mode := range []finder.DiagonalMovement{
		finder.DiagonalOnlyWhenNoObstacles,
		finder.DiagonalIfAtMostOneObstacle,
		finder.DiagonalAlways,
	} {
		astar := finder.NewAStarFinder(finder.WithDiagonal(mode))
		want, _ := astar.FindPath(0, 0, 1, 1, g, nil)

		plus := finder.NewJPSPlusFinder(finder.WithDiagonal(mode))
		_ = plus.Precompute(g)
		got, _ := plus.FindPath(0, 0, 1, 1, g, nil)

		if (len(want) == 0) != (len(got) == 0) {
			t.Errorf("mode=%v: A* path=%v, JPS+ path=%v", mode, want, got)
		}
	}

	// Sanity: only DiagonalAlways may cross two blocked cardinals.
	plus := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalAlways))
	_ = plus.Precompute(g)
	if p, _ := plus.FindPath(0, 0, 1, 1, g, nil); len(p) == 0 {
		t.Error("DiagonalAlways should allow the diagonal move")
	}
}

func TestJPSDiagonalHeuristicDefaults(t *testing.T) {
	base := finder.NewJumpPointFinderBase(finder.WithDiagonal(finder.DiagonalAlways))
	if !sameFunc(base.Heuristic, finder.Octile) {
		t.Error("JPS with diagonals should default to Octile")
	}

	custom := finder.NewJumpPointFinderBase(
		finder.WithDiagonal(finder.DiagonalAlways),
		finder.WithHeuristic(finder.Euclidean),
	)
	if !sameFunc(custom.Heuristic, finder.Euclidean) {
		t.Error("JPS should keep a caller-supplied heuristic")
	}

	plus := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalIfAtMostOneObstacle))
	if !sameFunc(plus.Heuristic, finder.Octile) {
		t.Error("JPS+ with diagonals should default to Octile")
	}
}

// WithAllowDiagonal alone must enable diagonal movement (not fall back to
// cardinal-only), for both JPS and JPS+.
func TestJPSAllowDiagonalOption(t *testing.T) {
	g := grid.NewOrthogonalGrid(grid.NewMatrix(10, 10))

	jps := finder.NewJumpPointFinder(finder.WithAllowDiagonal(false))
	p1, err := jps.FindPath(0, 0, 5, 5, g, nil)
	if err != nil {
		t.Fatalf("JPS error: %v", err)
	}
	if len(p1) != 6 {
		t.Errorf("JPS diagonal path length = %d, want 6", len(p1))
	}

	plus := finder.NewJPSPlusFinder(finder.WithAllowDiagonal(false))
	_ = plus.Precompute(g)
	p2, err := plus.FindPath(0, 0, 5, 5, g, nil)
	if err != nil {
		t.Fatalf("JPS+ error: %v", err)
	}
	if len(p2) != 6 {
		t.Errorf("JPS+ diagonal path length = %d, want 6", len(p2))
	}
}

// Variant constructors must pin their own diagonal rule even without options,
// otherwise the heuristic and movement model can be wrong.
func TestJPSVariantConstructorsSetMovement(t *testing.T) {
	never := finder.NewJPFNeverMoveDiagonally()
	always := finder.NewJPFAlwaysMoveDiagonally()
	atMostOne := finder.NewJPFMoveDiagonallyIfAtMostOneObstacle()
	noObstacles := finder.NewJPFMoveDiagonallyIfNoObstacles()

	if never.DiagonalMovement != finder.DiagonalNever {
		t.Errorf("never variant movement = %v", never.DiagonalMovement)
	}
	if always.DiagonalMovement != finder.DiagonalAlways {
		t.Errorf("always variant movement = %v", always.DiagonalMovement)
	}
	if atMostOne.DiagonalMovement != finder.DiagonalIfAtMostOneObstacle {
		t.Errorf("atMostOne variant movement = %v", atMostOne.DiagonalMovement)
	}
	if noObstacles.DiagonalMovement != finder.DiagonalOnlyWhenNoObstacles {
		t.Errorf("noObstacles variant movement = %v", noObstacles.DiagonalMovement)
	}

	if !sameFunc(never.Heuristic, finder.Manhattan) {
		t.Error("cardinal-only JPS should keep Manhattan")
	}
	for name, h := range map[string]finder.HeuristicFunc{
		"always":      always.Heuristic,
		"atMostOne":   atMostOne.Heuristic,
		"noObstacles": noObstacles.Heuristic,
	} {
		if !sameFunc(h, finder.Octile) {
			t.Errorf("%s variant should default to Octile", name)
		}
	}

	g := grid.NewOrthogonalGrid([][]int32{{0, 1}, {1, 0}})
	if p, _ := always.FindPath(0, 0, 1, 1, g, nil); len(p) == 0 {
		t.Error("always variant should allow the diagonal move")
	}
	if p, _ := noObstacles.FindPath(0, 0, 1, 1, g, nil); len(p) != 0 {
		t.Errorf("noObstacles variant must not cut the corner: %v", p)
	}
}

// The jump table is relaxed with one-step turns at runtime, so every diagonal
// mode must match A* on these previously-misbehaving maps.
func TestJPSPlusDiagonalModesAreOptimal(t *testing.T) {
	g := grid.NewOrthogonalGrid([][]int32{
		{0, 0, 1, 0, 0, 0},
		{1, 0, 0, 0, 1, 1},
		{1, 0, 0, 0, 0, 0},
		{0, 1, 1, 0, 0, 0},
	})

	for _, mode := range []finder.DiagonalMovement{
		finder.DiagonalIfAtMostOneObstacle,
		finder.DiagonalAlways,
	} {
		want, _ := finder.NewAStarFinder(finder.WithDiagonal(mode)).FindPath(1, 2, 3, 3, g, nil)
		plus := finder.NewJPSPlusFinder(finder.WithDiagonal(mode))
		_ = plus.Precompute(g)
		got, err := plus.FindPath(1, 2, 3, 3, g, nil)
		if err != nil {
			t.Fatalf("mode=%v: %v", mode, err)
		}
		if wc, gc := tilePathCost(want), tilePathCost(got); wc != gc {
			t.Errorf("mode=%v: cost A*=%.4f JPS+=%.4f\nA=%v\nP=%v", mode, wc, gc, want, got)
		}
	}
}

// JPS+ must agree with A* (presence and cost) for every diagonal mode. The
// precomputed table is augmented at runtime with one-step turns because some
// jump points depend on the goal and cannot be encoded statically.
func TestJPSPlusMatchesAStarRandom(t *testing.T) {
	rng := rand.New(rand.NewSource(21))
	modes := []finder.DiagonalMovement{
		finder.DiagonalNever,
		finder.DiagonalOnlyWhenNoObstacles,
		finder.DiagonalIfAtMostOneObstacle,
		finder.DiagonalAlways,
	}
	for it := 0; it < 500; it++ {
		w := 2 + rng.Intn(6)
		h := 2 + rng.Intn(6)
		m := make([][]int, h)
		for y := range m {
			m[y] = make([]int, w)
			for x := range m[y] {
				if rng.Float64() < 0.3 {
					m[y][x] = 1
				}
			}
		}
		sx, sy := rng.Intn(w), rng.Intn(h)
		ex, ey := rng.Intn(w), rng.Intn(h)
		m[sy][sx], m[ey][ex] = 0, 0
		g := grid.NewOrthogonalGrid(m)

		for _, mode := range modes {
			want, _ := finder.NewAStarFinder(finder.WithDiagonal(mode)).FindPath(sx, sy, ex, ey, g, nil)
			plus := finder.NewJPSPlusFinder(finder.WithDiagonal(mode))
			_ = plus.Precompute(g)
			got, err := plus.FindPath(sx, sy, ex, ey, g, nil)
			if err != nil {
				t.Fatalf("it=%d mode=%v: %v", it, mode, err)
			}
			if (len(want) == 0) != (len(got) == 0) {
				t.Fatalf("it=%d mode=%v presence mismatch\n m=%v\n A=%v\n P=%v",
					it, mode, m, want, got)
			}
			if len(want) > 0 {
				wc, gc := tilePathCost(want), tilePathCost(got)
				if math.Abs(wc-gc) > 1e-9 {
					t.Fatalf("it=%d mode=%v cost mismatch a=%.4f p=%.4f\n m=%v\n A=%v\n P=%v",
						it, mode, wc, gc, m, want, got)
				}
			}
		}
	}
}

func TestJPSPlusKnownRegressions(t *testing.T) {
	cases := []struct {
		name           string
		m              [][]int
		sx, sy, ex, ey int
		mode           finder.DiagonalMovement
	}{
		{
			name: "never-presence",
			m: [][]int32{
				{0, 0, 0, 0, 1},
				{0, 0, 1, 0, 0},
				{0, 0, 0, 0, 0},
				{0, 0, 0, 0, 1},
				{0, 0, 0, 0, 0},
				{0, 0, 1, 0, 1},
				{1, 1, 1, 0, 0},
			},
			sx: 0, sy: 4, ex: 2, ey: 3,
			mode: finder.DiagonalNever,
		},
		{
			name: "never-detour",
			m: [][]int32{
				{0, 0, 1, 1, 0, 0, 0},
				{1, 0, 0, 0, 0, 0, 0},
				{1, 0, 0, 0, 1, 1, 1},
				{0, 0, 0, 0, 0, 1, 0},
				{0, 1, 0, 0, 0, 0, 0},
				{0, 0, 0, 1, 0, 0, 0},
			},
			sx: 2, sy: 1, ex: 5, ey: 4,
			mode: finder.DiagonalNever,
		},
		{
			name: "no-obstacles-turn",
			m: [][]int32{
				{0, 1, 1, 0},
				{0, 0, 1, 1},
				{0, 0, 0, 0},
				{0, 0, 0, 0},
				{0, 1, 1, 0},
			},
			sx: 0, sy: 1, ex: 2, ey: 2,
			mode: finder.DiagonalOnlyWhenNoObstacles,
		},
	}

	for _, tc := range cases {
		g := grid.NewOrthogonalGrid(tc.m)
		want, _ := finder.NewAStarFinder(finder.WithDiagonal(tc.mode)).FindPath(tc.sx, tc.sy, tc.ex, tc.ey, g, nil)
		plus := finder.NewJPSPlusFinder(finder.WithDiagonal(tc.mode))
		_ = plus.Precompute(g)
		got, err := plus.FindPath(tc.sx, tc.sy, tc.ex, tc.ey, g, nil)
		if err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if (len(want) == 0) != (len(got) == 0) {
			t.Fatalf("%s: presence mismatch A=%v P=%v", tc.name, want, got)
		}
		if len(want) > 0 {
			if wc, gc := tilePathCost(want), tilePathCost(got); math.Abs(wc-gc) > 1e-9 {
				t.Errorf("%s: cost A*=%.4f JPS+=%.4f\nA=%v\nP=%v", tc.name, wc, gc, want, got)
			}
		}
	}
}

// JPS/JPS+ are axis-aligned algorithms. Using them on hex/staggered grids is a
// caller error, not a silent A* fallback.
func TestJPSRejectsNonOrthogonal(t *testing.T) {
	hex := grid.NewHexGrid([][]int32{
		{0, 0, 0, 0, 0, 0},
		{0, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{0, 0, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0, 0},
	}, grid.WithTileSize(55, 64), grid.WithHexSide(32))

	stag := grid.NewStaggeredGrid([][]int32{
		{0, 0, 0, 0, 0, 0},
		{0, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{0, 0, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0, 0},
	}, grid.WithTileSize(64, 32))

	grids := []struct {
		name string
		g    finder.Grid
	}{{"hex", hex}, {"staggered", stag}}

	for _, gg := range grids {
		if _, err := finder.NewJumpPointFinder().FindPath(0, 0, 5, 5, gg.g, nil); !errors.Is(err, finder.ErrUnsupportedGrid) {
			t.Errorf("%s JPS error = %v, want ErrUnsupportedGrid", gg.name, err)
		}
		for _, mode := range []finder.DiagonalMovement{
			finder.DiagonalNever,
			finder.DiagonalOnlyWhenNoObstacles,
			finder.DiagonalIfAtMostOneObstacle,
			finder.DiagonalAlways,
		} {
			// No Precompute: unsupported topology is rejected before that check.
			f := finder.NewJPSPlusFinder(finder.WithDiagonal(mode))
			if _, err := f.FindPath(0, 0, 5, 5, gg.g, nil); !errors.Is(err, finder.ErrUnsupportedGrid) {
				t.Errorf("%s mode=%v JPS+ error = %v, want ErrUnsupportedGrid", gg.name, mode, err)
			}
		}
	}
}

// Hex movement costs 1 per step. A*/BiA*/IDA* must therefore match BFS steps.
func TestHexFindersMatchBFS(t *testing.T) {
	rng := rand.New(rand.NewSource(4))
	for it := 0; it < 60; it++ {
		w := 3 + rng.Intn(3)
		h := 3 + rng.Intn(3)
		m := make([][]int, h)
		for y := range m {
			m[y] = make([]int, w)
			for x := range m[y] {
				if rng.Float64() < 0.25 {
					m[y][x] = 1
				}
			}
		}
		sx, sy := rng.Intn(w), rng.Intn(h)
		ex, ey := rng.Intn(w), rng.Intn(h)
		m[sy][sx], m[ey][ex] = 0, 0
		g := grid.NewHexGrid(m, grid.WithTileSize(55, 64), grid.WithHexSide(32))

		bfs, _ := finder.NewBreadthFirstFinder().FindPath(sx, sy, ex, ey, g, nil)
		finders := []struct {
			name string
			f    finder.Finder
		}{
			{"AStar", finder.NewAStarFinder()},
			{"BiAStar", finder.NewBiAStarFinder()},
			{"IDAStar", finder.NewIDAStarFinder()},
		}
		for _, ft := range finders {
			got, err := ft.f.FindPath(sx, sy, ex, ey, g, nil)
			if err != nil {
				t.Fatalf("it=%d %s: %v", it, ft.name, err)
			}
			if (len(bfs) == 0) != (len(got) == 0) || (len(bfs) > 0 && len(bfs) != len(got)) {
				t.Fatalf("it=%d %s steps=%d, BFS=%d\nm=%v\nBFS=%v\nG=%v",
					it, ft.name, len(got), len(bfs), m, bfs, got)
			}
		}
	}
}

// Precompute rejects non-axis-aligned grids immediately, rather than letting
// them build a meaningless table and fail later in FindPath.
func TestJPSPlusPrecomputeRejectsNonOrthogonal(t *testing.T) {
	hex := grid.NewHexGrid(grid.NewMatrix(6, 6))
	stag := grid.NewStaggeredGrid(grid.NewMatrix(6, 6))
	for name, g := range map[string]finder.Grid{"hex": hex, "staggered": stag} {
		err := finder.NewJPSPlusFinder().Precompute(g)
		if !errors.Is(err, finder.ErrUnsupportedGrid) {
			t.Errorf("%s: Precompute error = %v, want ErrUnsupportedGrid", name, err)
		}
	}
}

func TestJPSPlusPrecomputedMismatch(t *testing.T) {
	g1 := grid.NewOrthogonalGrid(grid.NewMatrix(5, 5))
	f := finder.NewJPSPlusFinder()
	if err := f.Precompute(g1); err != nil {
		t.Fatal(err)
	}
	g2 := grid.NewOrthogonalGrid(grid.NewMatrix(6, 6))
	if _, err := f.FindPath(0, 0, 5, 5, g2, nil); !errors.Is(err, finder.ErrPrecomputedMismatch) {
		t.Errorf("FindPath error = %v, want ErrPrecomputedMismatch", err)
	}
}

// Adaptive JPS+ switches to A* on dense maps and must still return the
// optimal path.
func TestJPSPlusAdaptive(t *testing.T) {
	rng := rand.New(rand.NewSource(13))
	const n = 20
	m := make([][]int, n)
	walls := 0
	for y := range m {
		m[y] = make([]int, n)
		for x := range m[y] {
			if rng.Float64() < 0.3 {
				m[y][x] = 1
				walls++
			}
		}
	}
	m[0][0], m[n-1][n-1] = 0, 0
	g := grid.NewOrthogonalGrid(m)
	if float64(walls)/(n*n) < finder.AdaptiveDensityThreshold {
		t.Fatalf("test map is not dense enough: %.2f", float64(walls)/(n*n))
	}

	mode := finder.DiagonalOnlyWhenNoObstacles
	plus := finder.NewJPSPlusFinder(finder.WithAdaptive(), finder.WithDiagonal(mode))
	if err := plus.Precompute(g); err != nil {
		t.Fatal(err)
	}
	got, err := plus.FindPath(0, 0, n-1, n-1, g, nil)
	if err != nil {
		t.Fatal(err)
	}
	want, _ := finder.NewAStarFinder(finder.WithDiagonal(mode)).FindPath(0, 0, n-1, n-1, g, nil)
	if (len(want) == 0) != (len(got) == 0) {
		t.Fatalf("presence mismatch: A=%v P=%v", want, got)
	}
	if len(want) > 0 {
		if wc, gc := tilePathCost(want), tilePathCost(got); math.Abs(wc-gc) > 1e-9 {
			t.Errorf("cost mismatch: A*=%.4f JPS+=%.4f", wc, gc)
		}
	}
}

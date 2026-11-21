package finder_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

type eastboundGrid struct{ finder.Grid }

func (g eastboundGrid) GetNeighbors(node *finder.Node, _ finder.DiagonalMovement, dst []*finder.Node) []*finder.Node {
	if g.IsWalkableAt(node.X+1, node.Y) {
		return append(dst[:0], g.GetNodeAt(node.X+1, node.Y))
	}
	return dst[:0]
}

func TestBidirectionalFindersOnDirectedGrid(t *testing.T) {
	g := eastboundGrid{grid.NewOrthogonalGrid(grid.NewMatrix(4, 1))}
	for _, f := range []finder.Finder{finder.NewBiAStarFinder(), finder.NewBiDijkstraFinder(), finder.NewBiBreadthFirstFinder()} {
		path, err := f.FindPath(0, 0, 3, 0, g, nil)
		if err != nil || len(path) != 4 {
			t.Fatalf("%T: eastbound path %v, err %v", f, path, err)
		}
		path, err = f.FindPath(3, 0, 0, 0, g, nil)
		if err != nil || path != nil {
			t.Fatalf("%T: westbound path %v, err %v", f, path, err)
		}
	}
}

func TestBidirectionalWeightedSearchMatchesDijkstra(t *testing.T) {
	rng := rand.New(rand.NewSource(83))
	for _, mode := range []finder.DiagonalMovement{
		finder.DiagonalNever, finder.DiagonalAlways,
		finder.DiagonalIfAtMostOneObstacle, finder.DiagonalOnlyWhenNoObstacles,
	} {
		option := finder.WithDiagonal(mode)
		reference := finder.NewDijkstraFinder(option)
		algorithms := []finder.Finder{finder.NewBiAStarFinder(option), finder.NewBiDijkstraFinder(option)}
		for _, makeGrid := range []func([][]int32, ...grid.GridOption) grid.Grid{
			grid.NewOrthogonalGrid, grid.NewHexGrid, grid.NewStaggeredGrid,
		} {
			const size = 9
			for sample := 0; sample < 8; sample++ {
				matrix := grid.NewMatrix(size, size)
				for _, row := range matrix {
					for x := range row {
						if rng.Intn(4) == 0 {
							row[x] = 1
						}
					}
				}
				g := makeGrid(matrix)
				for y := int32(0); y < size; y++ {
					for x := int32(0); x < size; x++ {
						weights := [...]float32{0.25, 1, 2, 5}
						g.(grid.MutableGrid).SetWeightAt(x, y, weights[rng.Intn(len(weights))])
					}
				}
				for query := 0; query < 24; query++ {
					sx, sy, ex, ey := rng.Int31n(size), rng.Int31n(size), rng.Int31n(size), rng.Int31n(size)
					if !g.IsWalkableAt(ex, ey) {
						continue
					}
					want, err := reference.FindPath(sx, sy, ex, ey, g, nil)
					if err != nil {
						t.Fatal(err)
					}
					for _, alg := range algorithms {
						got, err := alg.FindPath(sx, sy, ex, ey, g, nil)
						if err != nil || (got == nil) != (want == nil) {
							t.Fatalf("%T on %T mode %v, (%d,%d) to (%d,%d): got %v, %v, want %v",
								alg, g, mode, sx, sy, ex, ey, got, err, want)
						}
						if got == nil {
							continue
						}
						if got[0] != [2]int32{sx, sy} || got[len(got)-1] != [2]int32{ex, ey} {
							t.Fatalf("incorrect endpoints: %v", got)
						}
						cost := weightedRouteCost(t, g, mode, got)
						wantCost := weightedRouteCost(t, g, mode, want)
						if math.Abs(cost-wantCost) > 1e-5 {
							t.Fatalf("%T on %T mode %v, (%d,%d) to (%d,%d): cost %g, want %g",
								alg, g, mode, sx, sy, ex, ey, cost, wantCost)
						}
					}
				}
			}
		}
	}
}

func weightedRouteCost(t *testing.T, g grid.Grid, mode finder.DiagonalMovement, path [][2]int32) float64 {
	t.Helper()
	var cost float64
	var neighbors [8]*finder.Node
	for i := 1; i < len(path); i++ {
		a, b := path[i-1], path[i]
		valid := false
		for _, n := range g.GetNeighbors(g.GetNodeAt(a[0], a[1]), mode, neighbors[:0]) {
			if n.X == b[0] && n.Y == b[1] {
				valid = true
				break
			}
		}
		if !valid {
			t.Fatalf("invalid edge %v to %v", a, b)
		}
		dx, dy := b[0]-a[0], b[1]-a[1]
		step := float32(1)
		if topology, ok := g.(finder.StepCostGrid); ok {
			step = topology.StepCost(dx, dy)
		} else if dx != 0 && dy != 0 {
			step = finder.SQRT2
		}
		cost += float64(step) * float64(g.GetWeightAt(b[0], b[1]))
	}
	return cost
}

func TestBiAStarAgainstJS(t *testing.T) {
	testFinderAgainstJS(t, "BiAStar", func() finder.Finder { return finder.NewBiAStarFinder() }, [][][2]int32{
		{{0, 0}, {1, 0}, {1, 1}},
		{{1, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}},
		{{0, 3}, {1, 3}, {1, 4}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {4, 4}, {4, 3}, {3, 3}},
		{{4, 4}, {5, 4}, {6, 4}, {7, 4}, {8, 4}, {9, 4}, {10, 4}, {11, 4}, {12, 4}, {13, 4}, {14, 4}, {15, 4}, {16, 4}, {17, 4}, {18, 4}, {19, 4}, {19, 5}, {19, 6}, {19, 7}, {19, 8}, {19, 9}, {19, 10}, {19, 11}, {19, 12}, {19, 13}, {19, 14}, {19, 15}, {19, 16}, {19, 17}, {19, 18}, {19, 19}},
	})
}

package grid

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func TestFindPathWorldAcrossMapsMatchesDijkstra(t *testing.T) {
	const width, height = 13, 9
	rng := rand.New(rand.NewSource(73))
	modes := []finder.DiagonalMovement{
		finder.DiagonalNever,
		finder.DiagonalAlways,
		finder.DiagonalOnlyWhenNoObstacles,
		finder.DiagonalIfAtMostOneObstacle,
	}
	for _, mode := range modes {
		option := finder.WithDiagonal(mode)
		dijkstra := finder.NewDijkstraFinder(option)
		for _, density := range []int{0, 20, 45} {
			for sample := 0; sample < 4; sample++ {
				matrix := NewMatrix(width, height)
				for _, row := range matrix {
					for x := range row {
						if rng.Intn(100) < density {
							row[x] = 1
						}
					}
				}
				g := NewOrthogonalGrid(matrix, WithTileSize(16, 24), WithFinder(finder.NewAStarFinder(option)))
				mutable := g.(MutableGrid)
				for y := int32(0); y < height; y++ {
					for x := int32(0); x < width; x++ {
						weights := [...]float32{0.25, 1, 1, 3}
						mutable.SetWeightAt(x, y, weights[rng.Intn(len(weights))])
					}
				}

				buff := NewPathBuff(16, 16)
				for query := 0; query < 16; query++ {
					sx, sy := rng.Int31n(width), rng.Int31n(height)
					ex, ey := rng.Int31n(width), rng.Int31n(height)
					if !g.IsWalkableAt(ex, ey) {
						continue
					}
					want, err := dijkstra.FindPath(sx, sy, ex, ey, g, nil)
					if err != nil {
						t.Fatal(err)
					}
					wx1, wy1 := g.TileToWorld(sx, sy)
					wx2, wy2 := g.TileToWorld(ex, ey)
					got, err := g.FindPathWorld(wx1, wy1, wx2, wy2, buff)
					if err != nil || (got == nil) != (want == nil) {
						t.Fatalf("mode %v density %d sample %d (%d,%d) to (%d,%d): got %v, err %v, want %v",
							mode, density, sample, sx, sy, ex, ey, got, err, want)
					}
					if got == nil {
						continue
					}
					if len(got) != len(buff.Tile) || got[0] != [2]float32{wx1, wy1} || got[len(got)-1] != [2]float32{wx2, wy2} {
						t.Fatalf("unexpected world path %v or tile path %v", got, buff.Tile)
					}
					for i, point := range got {
						x, y := g.WorldToTile(point[0], point[1])
						if buff.Tile[i] != [2]int32{x, y} {
							t.Fatalf("world point %v maps to (%d,%d), want %v", point, x, y, buff.Tile[i])
						}
					}
					gotCost := checkedWeightedPathCost(t, g, mode, buff.Tile)
					wantCost := checkedWeightedPathCost(t, g, mode, want)
					if math.Abs(gotCost-wantCost) > 1e-5 {
						t.Fatalf("mode %v density %d sample %d (%d,%d) to (%d,%d): got cost %g, want %g",
							mode, density, sample, sx, sy, ex, ey, gotCost, wantCost)
					}
				}
			}
		}
	}
}

func checkedWeightedPathCost(t *testing.T, g Grid, mode finder.DiagonalMovement, path [][2]int32) float64 {
	t.Helper()
	var cost float64
	var neighbors [8]*finder.Node
	for i := 1; i < len(path); i++ {
		from, to := path[i-1], path[i]
		next := g.GetNodeAt(to[0], to[1])
		if !slices.Contains(g.GetNeighbors(g.GetNodeAt(from[0], from[1]), mode, neighbors[:0]), next) {
			t.Fatalf("mode %v: illegal step %v to %v", mode, from, to)
		}
		step := float64(next.Weight)
		if from[0] != to[0] && from[1] != to[1] {
			step *= math.Sqrt2
		}
		cost += step
	}
	return cost
}

func TestPathBuffReuse(t *testing.T) {
	matrix := NewMatrix(20, 20)
	for y := 0; y < 20; y++ {
		matrix[y][10] = 1
	}
	matrix[10][10] = 0
	g := NewOrthogonalGrid(matrix)
	buff := NewPathBuff(1, 1)

	p1, err := g.FindPathWorld(0, 0, 19, 19, buff)
	if err != nil || p1 == nil {
		t.Fatalf("first path: %v %v", p1, err)
	}
	if len(buff.Tile) != len(p1) {
		t.Fatalf("tile header not updated: %d vs %d", len(buff.Tile), len(p1))
	}
	if len(buff.World) != len(p1) {
		t.Fatalf("world header not updated: %d vs %d", len(buff.World), len(p1))
	}

	p2, err := g.FindPathWorld(19, 19, 0, 0, buff)
	if err != nil || p2 == nil {
		t.Fatalf("second path: %v %v", p2, err)
	}
	if len(p2) != len(p1) {
		t.Fatalf("second path length: %d vs %d", len(p2), len(p1))
	}

	buff.Reset()
	if len(buff.Tile) != 0 || len(buff.World) != 0 {
		t.Fatal("Reset must clear lengths")
	}
}

func TestFindPathWorldOpenGridRespectsDiagonalRule(t *testing.T) {
	f := finder.NewAStarFinder(finder.WithDiagonal(finder.DiagonalNever))
	g := NewOrthogonalGrid(NewMatrix(3, 3), WithFinder(f))
	buff := NewPathBuff(5, 5)

	path, err := g.FindPathWorld(0, 0, 2, 2, buff)
	if err != nil || len(path) != 5 || len(buff.Tile) != len(path) {
		t.Fatalf("got world %v, tiles %v, err %v, want five points", path, buff.Tile, err)
	}
	for i := 1; i < len(path); i++ {
		a, b := path[i-1], path[i]
		if (a[0] == b[0]) == (a[1] == b[1]) {
			t.Fatalf("non-cardinal step from %v to %v", a, b)
		}
	}
}

func TestFindPathWorldOpenGridRespectsWeights(t *testing.T) {
	g := NewOrthogonalGrid(NewMatrix(5, 3))
	g.(MutableGrid).SetWeightAt(2, 1, 100)
	buff := NewPathBuff(8, 8)

	path, err := g.FindPathWorld(0, 1, 4, 1, buff)
	if err != nil || len(path) != 7 || len(buff.Tile) != len(path) {
		t.Fatalf("got world %v, tiles %v, err %v, want the seven-point detour", path, buff.Tile, err)
	}
	for _, tile := range buff.Tile {
		if tile == [2]int32{2, 1} {
			t.Fatalf("path uses the expensive tile: %v", buff.Tile)
		}
	}
}

func TestFindPathWorldKeepsDistinctEndpointsInSameTile(t *testing.T) {
	g := NewOrthogonalGrid(NewMatrix(1, 1), WithTileSize(16, 16))
	buff := NewPathBuff(1, 2)

	path, err := g.FindPathWorld(2, 3, 5, 7, buff)
	if err != nil || len(path) != 2 || path[0] != [2]float32{2, 3} || path[1] != [2]float32{5, 7} {
		t.Fatalf("got %v, %v, want both original world endpoints", path, err)
	}
	if len(buff.World) != 2 || len(buff.Tile) != 1 || buff.Tile[0] != [2]int32{0, 0} {
		t.Fatalf("unexpected buffer contents: %+v", buff)
	}
}

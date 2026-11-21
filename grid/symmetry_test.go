package grid

import (
	"math/rand"
	"slices"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func TestBuiltInOffsetGridsHaveSymmetricNeighbors(t *testing.T) {
	layouts := []struct {
		name string
		make func([][]int32, ...GridOption) Grid
		opts []GridOption
	}{
		{"hex/pointy-odd", NewHexGrid, nil},
		{"hex/pointy-even", NewHexGrid, []GridOption{WithHexEvenStagger()}},
		{"hex/flat-odd", NewHexGrid, []GridOption{WithHexFlatTop()}},
		{"hex/flat-even", NewHexGrid, []GridOption{WithHexFlatTop(), WithHexEvenStagger()}},
		{"staggered/x-odd", NewStaggeredGrid, []GridOption{WithStaggerAxis("x")}},
		{"staggered/x-even", NewStaggeredGrid, []GridOption{WithStaggerAxis("x"), WithStaggerEvenIndex()}},
		{"staggered/y-odd", NewStaggeredGrid, []GridOption{WithStaggerAxis("y")}},
		{"staggered/y-even", NewStaggeredGrid, []GridOption{WithStaggerAxis("y"), WithStaggerEvenIndex()}},
	}
	for _, layout := range layouts {
		t.Run(layout.name, func(t *testing.T) {
			rng := rand.New(rand.NewSource(91))
			for sample := 0; sample < 16; sample++ {
				matrix := NewMatrix(8, 8)
				if sample != 0 {
					for _, row := range matrix {
						for x := range row {
							if rng.Intn(4) == 0 {
								row[x] = 1
							}
						}
					}
				}
				g := layout.make(matrix, layout.opts...)
				for _, mode := range []finder.DiagonalMovement{
					finder.DiagonalNever, finder.DiagonalAlways,
					finder.DiagonalOnlyWhenNoObstacles, finder.DiagonalIfAtMostOneObstacle,
				} {
					var forward, reverse [8]*finder.Node
					for y := int32(0); y < g.Height(); y++ {
						for x := int32(0); x < g.Width(); x++ {
							if !g.IsWalkableAt(x, y) {
								continue
							}
							source := g.GetNodeAt(x, y)
							for _, target := range g.GetNeighbors(source, mode, forward[:0]) {
								if !slices.Contains(g.GetNeighbors(target, mode, reverse[:0]), source) {
									t.Fatalf("sample %d mode %v: (%d,%d) to (%d,%d) has no reverse edge", sample, mode, x, y, target.X, target.Y)
								}
							}
						}
					}
				}
			}
		})
	}
}

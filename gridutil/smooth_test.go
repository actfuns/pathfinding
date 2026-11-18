package gridutil

import "testing"

// denseTestGrid has widely spaced tile origins so the old sampling loop
// (t < steps) never reached the destination tile at f == 1.
type denseTestGrid struct {
	blocked map[[2]int]bool
}

func (g *denseTestGrid) Width() int  { return 2 }
func (g *denseTestGrid) Height() int { return 1 }
func (g *denseTestGrid) IsWalkableAt(x, y int) bool {
	return !g.blocked[[2]int{x, y}]
}
func (g *denseTestGrid) TileToWorld(tx, ty int) (float32, float32) {
	return float32(tx * 100), float32(ty * 100)
}
func (g *denseTestGrid) WorldToTile(wx, wy float32) (int, int) {
	return int(wx) / 100, int(wy) / 100
}
func (g *denseTestGrid) TileWidth() int  { return 50 }
func (g *denseTestGrid) TileHeight() int { return 50 }

func TestDenseLOSChecksDestination(t *testing.T) {
	g := &denseTestGrid{blocked: map[[2]int]bool{{1, 0}: true}}

	if HasLineOfSightDense(g, 0, 0, 1, 0) {
		t.Error("HasLineOfSightDense reported LOS into a blocked destination")
	}
	if denseLineOfSight(g, [2]int{0, 0}, [2]int{1, 0}) {
		t.Error("denseLineOfSight reported LOS into a blocked destination")
	}

	g.blocked = map[[2]int]bool{}
	if !HasLineOfSightDense(g, 0, 0, 1, 0) {
		t.Error("HasLineOfSightDense reported no LOS on an open line")
	}
}

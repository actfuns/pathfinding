package grid

import "github.com/actfuns/pathfinding/finder"

// findPathWorld is shared by the built-in grids. It always runs the configured
// Finder (there is no open-grid shortcut), applies smoothing, then maps tile
// centres to world space while preserving the caller's original start and end points.
func findPathWorld(g Grid, smoother Smoother, wx1, wy1, wx2, wy2 float32, buff *PathBuff) ([][2]float32, error) {
	var dst [][2]float32
	var tileDst [][2]int32
	if buff != nil {
		dst = buff.World[:0]
		tileDst = buff.Tile[:0]
	}
	f := g.Finder()
	if f == nil {
		return nil, nil
	}
	sx, sy := g.WorldToTile(wx1, wy1)
	ex, ey := g.WorldToTile(wx2, wy2)
	if !g.IsInside(sx, sy) || !g.IsInside(ex, ey) {
		return nil, finder.ErrOutOfBounds
	}
	path, err := f.FindPath(sx, sy, ex, ey, g, tileDst)
	if err != nil {
		return nil, err
	}
	if len(path) == 0 {
		return nil, nil
	}
	if smoother != nil {
		path = smoother(path)
	}
	if len(path) == 0 {
		return nil, nil
	}
	if buff != nil {
		buff.Tile = path
	}
	worldLen := len(path)
	if worldLen == 1 && (wx1 != wx2 || wy1 != wy2) {
		// Distinct world positions in one tile still have two endpoints.
		worldLen = 2
	}
	var out [][2]float32
	if cap(dst) >= worldLen {
		out = dst[:worldLen]
	} else {
		out = make([][2]float32, worldLen)
	}
	for i, p := range path {
		out[i][0], out[i][1] = g.TileToWorld(p[0], p[1])
	}
	out[0][0], out[0][1] = wx1, wy1
	out[len(out)-1][0], out[len(out)-1][1] = wx2, wy2
	if buff != nil {
		buff.World = out
	}
	return out, nil
}

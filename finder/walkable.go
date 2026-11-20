package finder

// nodesView is an optional fast-read capability implemented by grids that
// own a contiguous node slice. It is read-only by contract; callers must not
// mutate the returned nodes.
type nodesView interface {
	NodesView() []Node
}

// walkableView avoids per-check interface dispatch for JPS-style searches when
// the grid exposes its node slice.
type walkableView struct {
	grid   Grid
	nodes  []Node
	width  int32
	height int32
}

func newWalkableView(g Grid) walkableView {
	v := walkableView{grid: g, width: g.Width(), height: g.Height()}
	if nv, ok := g.(nodesView); ok {
		v.nodes = nv.NodesView()
	}
	return v
}

func (v *walkableView) at(x, y int32) bool {
	if uint32(x) >= uint32(v.width) || uint32(y) >= uint32(v.height) {
		return false
	}
	if v.nodes != nil {
		return v.nodes[int(y)*int(v.width)+int(x)].Walkable
	}
	return v.grid.IsWalkableAt(x, y)
}

func (v *walkableView) nodeAt(x, y int32) *Node {
	if v.nodes != nil {
		return &v.nodes[int(y)*int(v.width)+int(x)]
	}
	return v.grid.GetNodeAt(x, y)
}

package finder

// Node represents a single node on a grid. Search state is kept outside the
// node (see searchStates) so grid clones and concurrent searches stay cheap.
type Node struct {
	X        int32
	Y        int32
	Walkable bool
	// Weight is the per-tile movement cost multiplier. All walkable tiles have
	// a weight >= 1. A weight of 2.0 means moving through this tile costs twice
	// as much as moving through a default tile. Used by cost-aware pathfinders
	// (A*, Dijkstra, IDA*). Defaults to 1.0. Set via NewNode or
	// grid.Grid.SetWeightAt.
	Weight float32
}

// NewNode creates a node with default weight 1.0.
func NewNode(x, y int32) *Node {
	return &Node{X: x, Y: y, Walkable: true, Weight: 1.0}
}

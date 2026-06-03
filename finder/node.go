package finder

// Node represents a single node on a grid.
type Node struct {
	X        int
	Y        int
	Walkable bool
	// Weight is the per-tile movement cost multiplier. All walkable tiles have
	// a weight >= 1. A weight of 2.0 means moving through this tile costs twice
	// as much as moving through a default tile. Used by cost-aware pathfinders
	// (A*, Dijkstra, JPS, etc.). Defaults to 1.0. Set via NewNode or
	// grid.Grid.SetWeightAt.
	Weight float64

	// Pathfinding state (reused across searches)
	G, H, F float64
	Parent  *Node
	Opened  int // 0=unopened, 1=BY_START, 2=BY_END (for bidirectional)
	Closed  bool
	By      int // 1=BY_START, 2=BY_END (for bidirectional BFS)

	// JPF tracking
	Tested      bool
	RetainCount int

	// heap index
	HeapIndex int

	// SearchID matches the finder's search sequence to avoid resetting per node.
	// 0 = initial state, updated on first use by each finder call.
	SearchID int
}

// NewNode creates a node with default weight 1.0.
func NewNode(x, y int) *Node {
	return &Node{X: x, Y: y, Walkable: true, Weight: 1.0}
}

// ResetSearch resets all search-related fields if seq doesn't match.
// This allows repeated use of the same node across multiple FindPath calls
// without cloning the entire grid.
func (n *Node) ResetSearch(seq int) {
	if n.SearchID != seq {
		n.SearchID = seq
		n.G = 0
		n.H = 0
		n.F = 0
		n.Parent = nil
		n.Opened = 0
		n.Closed = false
		n.By = 0
		n.Tested = false
		n.RetainCount = 0
		n.HeapIndex = 0
	}
}

package finder

// Grid is the interface that all pathfinding algorithms depend on.
// Different grid layouts (orthogonal, hex, staggered) implement this interface
// with their own neighbor topology.
type Grid interface {
	// Width returns the number of tiles horizontally in the grid.
	Width() int
	// Height returns the number of tiles vertically in the grid.
	Height() int
	// IsInside reports whether the tile (x, y) is within the grid bounds.
	IsInside(x, y int) bool
	// IsWalkableAt reports whether the tile (x, y) is walkable.
	// Tiles outside the grid are reported as not walkable.
	IsWalkableAt(x, y int) bool
	// SetWalkableAt sets the walkability of the tile (x, y).
	SetWalkableAt(x, y int, walkable bool)
	// GetNodeAt returns the node at tile (x, y). Panics if outside the grid.
	GetNodeAt(x, y int) *Node
	// GetNeighbors returns the walkable, adjacent neighbors of the given node.
	GetNeighbors(node *Node, diagonal DiagonalMovement, buffer []*Node) []*Node
	// Clone returns a copy of the grid with its own node storage, but sharing
	// the Finder with the original. The Finder keeps reusable search buffers,
	// so the copy must not be searched at the same time as the original — it is
	// for snapshotting a map, not for running two searches against one map.
	Clone() Grid
}

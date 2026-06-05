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
	// GetNodeAt returns the node at tile (x, y). Panics if outside the grid.
	GetNodeAt(x, y int) *Node
	// GetNeighbors returns the walkable, adjacent neighbors of the given node.
	GetNeighbors(node *Node, diagonal DiagonalMovement, buffer []*Node) []*Node
	// Clone returns a copy of the grid.
	Clone() Grid
}

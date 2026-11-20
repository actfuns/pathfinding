package finder

// Grid is the interface that all pathfinding algorithms depend on.
// Different grid layouts (orthogonal, hex, staggered) implement this interface
// with their own neighbor topology.
type Grid interface {
	// Width returns the number of tiles horizontally in the grid.
	Width() int32
	// Height returns the number of tiles vertically in the grid.
	Height() int32
	// IsInside reports whether the tile (x, y) is within the grid bounds.
	IsInside(x, y int32) bool
	// IsWalkableAt reports whether the tile (x, y) is walkable.
	// Tiles outside the grid are reported as not walkable.
	IsWalkableAt(x, y int32) bool
	// SetWalkableAt sets the walkability of the tile (x, y).
	SetWalkableAt(x, y int32, walkable bool)
	// GetNodeAt returns the node at tile (x, y). Panics if outside the grid.
	GetNodeAt(x, y int32) *Node
	// GetNeighbors returns the walkable, adjacent neighbors of the given node.
	GetNeighbors(node *Node, diagonal DiagonalMovement, buffer []*Node) []*Node
	// Clone returns a copy of the grid with its own node storage, but sharing
	// the Finder with the original. Standard finders keep per-call scratch and
	// are safe for concurrent read-only searches; custom finders must provide
	// the same guarantee.
	Clone() Grid
}

// StepCostGrid is optionally implemented by grids whose step costs differ from
// the square-grid default (cardinal = 1, diagonal = sqrt(2)).
type StepCostGrid interface {
	StepCost(dx, dy int32) float32
}

// heuristicGrid is optionally implemented by grids that need a topology-specific
// default heuristic (for example, hex grids with unit-cost moves).
type heuristicGrid interface {
	DefaultHeuristic(diagonal DiagonalMovement) HeuristicFunc
}

// gridStepCost returns the grid's step-cost function, or nil when the
// square-grid default applies. Returning nil keeps the inline fast path.
func gridStepCost(grid Grid) func(dx, dy int32) float32 {
	if g, ok := grid.(StepCostGrid); ok {
		return g.StepCost
	}
	return nil
}

// heuristicFor returns the grid's default heuristic unless the caller supplied
// an explicit one.
func heuristicFor(grid Grid, diagonal DiagonalMovement, fallback HeuristicFunc, custom bool) HeuristicFunc {
	if !custom {
		if g, ok := grid.(heuristicGrid); ok {
			return g.DefaultHeuristic(diagonal)
		}
	}
	return fallback
}

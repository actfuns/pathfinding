package finder

// DiagonalMovement represents allowed diagonal movement types.
type DiagonalMovement int

const (
	// DiagonalAlways allows diagonal movement regardless of obstacles.
	DiagonalAlways DiagonalMovement = 1
	// DiagonalNever forbids diagonal movement entirely.
	DiagonalNever DiagonalMovement = 2
	// DiagonalIfAtMostOneObstacle allows diagonal movement if at most one of the
	// two adjacent cardinal cells is blocked.
	DiagonalIfAtMostOneObstacle DiagonalMovement = 3
	// DiagonalOnlyWhenNoObstacles only allows diagonal movement when both
	// adjacent cardinal cells are walkable.
	DiagonalOnlyWhenNoObstacles DiagonalMovement = 4
)

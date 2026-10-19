package finder

import "errors"

// ErrTimeLimit is returned when a search gives up because it ran past the
// TimeLimit set with WithTimeLimit.
//
// It means "could not determine", not "no path exists": the two are distinct
// results. A nil error with a nil path means the search completed and found no
// route; a non-nil error means the search never finished, so the answer is
// unknown and retrying with more time may succeed.
var ErrTimeLimit = errors.New("finder: search exceeded its time limit")

// ErrOutOfBounds is returned when a path request names an endpoint outside the
// grid. It is a caller mistake rather than a property of the map, so it is
// reported as an error instead of the "no route" result.
var ErrOutOfBounds = errors.New("finder: endpoint is outside the grid")

// Finder is the interface for all pathfinding algorithms.
//
// Results come in three shapes, and the pair (path, err) tells them apart:
//
//	path, nil  a route was found
//	nil, nil   the search completed and no route exists
//	nil, err   the search could not complete; err is why (e.g. ErrTimeLimit)
//
// A nil path with a nil error is the normal answer for an unreachable target,
// so most callers only need to check len(path).
//
// The returned path is backed by an internal buffer and is only valid until the
// next FindPath call on the same Finder. If you need to persist the result,
// copy it (e.g., append([][2]int{}, path...)).
type Finder interface {
	FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error)
}
